"""AidVoice SenseVoice NPU ASR 封装

批量模式：累积音频 → 静音 → write() + stop() → 回调 FINAL。
reset() 时自动重建 ASR 实例（原实例 stop() 后不可复用）。
"""

import ctypes
import logging
import re
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

_CALLBACK_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
_ERROR_CALLBACK_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p, ctypes.c_void_p)

TYPE_PARTIAL = 0
TYPE_FINAL = 1
TYPE_SENSEVOICE = 2
TYPE_ASR = 1
MODE_NOSTREAM = 1

_MIN_CJK_CHARS = 1
_SPEECH_ENERGY_THRESHOLD = 0.02
_SILENCE_CHUNKS = 8

_CJK_RE = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff]")
_PUNCTUATIONS = set(",.!?;:。，！？；：、 ")


def _count_cjk(text: str) -> int:
    return len(_CJK_RE.findall(text))


def _strip_punctuation(text: str) -> str:
    return "".join(c for c in text if c not in _PUNCTUATIONS)


def _load_library():
    bridge_path = Path(__file__).parent / "asr_bridge.so"
    if not bridge_path.is_file():
        raise FileNotFoundError(f"asr_bridge.so 未找到: {bridge_path}")
    lib = ctypes.CDLL(str(bridge_path))

    lib.asr_bridge_create.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.asr_bridge_create.restype = ctypes.c_void_p

    lib.asr_bridge_init.argtypes = [ctypes.c_void_p]
    lib.asr_bridge_init.restype = ctypes.c_int

    lib.asr_bridge_set_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.asr_bridge_set_mode.restype = ctypes.c_int

    lib.asr_bridge_set_callback.argtypes = [
        ctypes.c_void_p, _CALLBACK_FUNC, _ERROR_CALLBACK_FUNC, ctypes.c_void_p,
    ]
    lib.asr_bridge_set_callback.restype = ctypes.c_int

    lib.asr_bridge_write_float.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int,
    ]
    lib.asr_bridge_write_float.restype = ctypes.c_int

    lib.asr_bridge_stop.argtypes = [ctypes.c_void_p]
    lib.asr_bridge_stop.restype = ctypes.c_int

    lib.asr_bridge_destroy.argtypes = [ctypes.c_void_p]
    lib.asr_bridge_destroy.restype = ctypes.c_int

    return lib


class AidVoiceASR:
    """AidVoice SenseVoice NPU 语音识别器（批量模式，自动重建实例）"""

    def __init__(
        self,
        model_type: int = TYPE_SENSEVOICE,
        min_cjk_chars: int = _MIN_CJK_CHARS,
        speech_threshold: float = _SPEECH_ENERGY_THRESHOLD,
        silence_chunks: int = _SILENCE_CHUNKS,
    ) -> None:
        self._lib = _load_library()
        self._model_type = model_type
        self._min_cjk_chars = min_cjk_chars
        self._speech_threshold = speech_threshold
        self._silence_chunks = silence_chunks

        self._lock = threading.Lock()
        self._handle: Optional[int] = None
        self._latest_text = ""
        self._sentence_ready = False
        self._callback_received = False
        self._error_message = ""
        self._active = False
        self._submitted = False
        self._silence_count = 0
        self._has_speech = False
        self._buffer: list[float] = []

        self._on_result_cb = _CALLBACK_FUNC(self._on_result)
        self._on_error_cb = _ERROR_CALLBACK_FUNC(self._on_error)

        self._create_instance()

    # ── 实例管理 ───────────────────────────────────────────────

    def _create_instance(self) -> None:
        self._handle = self._lib.asr_bridge_create(self._model_type, TYPE_ASR)
        if not self._handle:
            raise RuntimeError("AidVoice ASR 创建失败")
        self._lib.asr_bridge_set_mode(self._handle, MODE_NOSTREAM)
        self._lib.asr_bridge_set_callback(
            self._handle, self._on_result_cb, self._on_error_cb, ctypes.c_void_p(0),
        )
        ret = self._lib.asr_bridge_init(self._handle)
        if ret != 0:
            raise RuntimeError(f"AidVoice ASR init 失败: {ret}")
        logger.debug("AidVoice ASR 实例已创建")

    def _destroy_instance(self) -> None:
        if self._handle is not None:
            self._lib.asr_bridge_destroy(self._handle)
            self._handle = None

    # ── 回调 ──────────────────────────────────────────────────

    def _on_result(self, status: int, text: bytes, _sid: int, _userdata) -> None:
        text_str = text.decode("utf-8") if text else ""
        stripped = text_str.strip()

        with self._lock:
            if not self._active or self._sentence_ready:
                return
            if status == TYPE_FINAL:
                self._callback_received = True
                clean = _strip_punctuation(stripped)
                cjk = _count_cjk(clean)
                logger.info("AidVoice FINAL: %r (cjk=%d)", stripped, cjk)
                if cjk >= self._min_cjk_chars:
                    self._latest_text = stripped
                    self._sentence_ready = True

    def _on_error(self, code: int, message: bytes, _userdata) -> None:
        msg = message.decode("utf-8") if message else ""
        with self._lock:
            self._error_message = msg
        logger.error("AidVoice ASR 错误 (code=%d): %s", code, msg)

    # ── 公开接口 ──────────────────────────────────────────────

    def process(self, samples: np.ndarray) -> None:
        with self._lock:
            if not self._active or self._submitted:
                return

        data = np.asarray(samples, dtype=np.float32).ravel().tolist()
        self._buffer.extend(data)

        with self._lock:
            self._check_vad(np.asarray(data, dtype=np.float32))

    def get_partial(self) -> str:
        return ""

    def detect_endpoint(self) -> bool:
        should_submit = False
        with self._lock:
            if self._sentence_ready:
                return True
            if not self._submitted and self._should_stop():
                should_submit = True
                self._submitted = True
            if self._submitted and self._callback_received:
                return True
            if not should_submit:
                return False

        self._do_submit()
        return False

    def _do_submit(self) -> None:
        buf = np.array(self._buffer, dtype=np.float32)
        ptr = buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        n = buf.shape[0]
        logger.info("AidVoice 提交识别 (%d samples, %.2fs)", n, n / 16000.0)
        self._lib.asr_bridge_write_float(self._handle, ptr, n)
        self._lib.asr_bridge_stop(self._handle)
        logger.info("AidVoice stop() 已调用")

    def finalize(self) -> str:
        with self._lock:
            text = self._latest_text
            self._latest_text = ""
            self._sentence_ready = False
            self._callback_received = False
            self._active = False
            self._submitted = False
            self._silence_count = 0
            self._has_speech = False
            self._buffer.clear()
        logger.info("AidVoice ASR finalize: %s", text)
        return text

    def reset(self) -> None:
        with self._lock:
            self._latest_text = ""
            self._sentence_ready = False
            self._callback_received = False
            self._submitted = False
            self._active = True
            self._silence_count = 0
            self._has_speech = False
            self._buffer.clear()
            self._destroy_instance()
            self._create_instance()
        logger.info("AidVoice ASR 已复位（重建实例）")

    def force_submit(self) -> None:
        with self._lock:
            if not self._submitted and self._has_speech:
                self._submitted = True
            else:
                return
        self._do_submit()

    @property
    def error_message(self) -> str:
        with self._lock:
            return self._error_message

    def destroy(self) -> None:
        self._destroy_instance()
        logger.info("AidVoice ASR 已销毁")

    # ── VAD + submit ──────────────────────────────────────────

    def _check_vad(self, samples: np.ndarray) -> None:
        energy = float(np.sqrt(np.mean(samples ** 2)))
        # 将麦克风输入放大 5 倍后再判断阈值（与 KWS 增益策略一致）
        energy *= 5.0
        if energy > self._speech_threshold:
            if not self._has_speech:
                logger.debug("VAD 检测到语音 (energy=%.6f)", energy)
            self._has_speech = True
            self._silence_count = 0
        elif self._has_speech:
            self._silence_count += 1
            if self._silence_count >= self._silence_chunks:
                logger.debug("VAD 检测到静音 %d/%d (energy=%.6f)",
                            self._silence_count, self._silence_chunks, energy)

    def _should_stop(self) -> bool:
        return self._has_speech and self._silence_count >= self._silence_chunks


