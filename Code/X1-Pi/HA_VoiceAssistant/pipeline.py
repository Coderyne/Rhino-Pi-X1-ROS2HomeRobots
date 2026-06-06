import logging
import queue
import threading
import time
from typing import Optional

import numpy as np

from asr_aidvoice import AidVoiceASR
from audio import AudioCapture
from config import AssistantConfig
from ha_client import HAAssistClient
from state import State, StateMachine
from tts import TTSEngine
from wakeword import WakeWordDetector

logger = logging.getLogger(__name__)

_SENTENCE_DELIMITERS = set("。？！\n.?!;；,，")
_GIF_FIFO = "/tmp/gif_cmd"


def _detect_playback_device(device_name: str, playback_device: str = "") -> Optional[int]:
    try:
        import sounddevice as sd

        target = playback_device if playback_device else device_name
        if ":" not in target:
            return None

        card_idx = int(target.split(":")[1].split(",")[0])

        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            if dev["max_output_channels"] > 0 and f"hw:{card_idx}," in dev["name"]:
                logger.info("播放设备: %d (%s)", i, dev["name"])
                return i
    except Exception:
        logger.warning("无法检测播放设备，使用系统默认")

    return None


class Pipeline:

    def __init__(self, config: AssistantConfig) -> None:
        self._config = config
        self._state = StateMachine(State.IDLE)

        self._audio = AudioCapture(
            device_name=config.device_name,
            sample_rate=config.sample_rate,
        )

        self._wakeword = WakeWordDetector(
            encoder=config.kws_encoder,
            decoder=config.kws_decoder,
            joiner=config.kws_joiner,
            tokens=config.kws_tokens,
            keywords_file=config.kws_keywords_file,
            num_threads=config.kws_num_threads,
            keywords_threshold=config.kws_keywords_threshold,
            keywords_score=config.kws_keywords_score,
            sample_rate=config.sample_rate,
        )

        self._asr = AidVoiceASR()

        self._ha = HAAssistClient(
            ha_url=config.ha_url,
            token=config.ha_token,
            agent_id=config.ha_agent_id,
            timeout=config.ha_timeout,
        )

        self._tts = TTSEngine(
            model=config.tts_model,
            tokens=config.tts_tokens,
            lexicon=config.tts_lexicon,
            rule_fsts=config.tts_rule_fsts,
            num_threads=config.tts_num_threads,
            speed=config.tts_speed,
            sid=config.tts_sid,
            volume_gain=config.tts_volume_gain,
            playback_device=_detect_playback_device(config.device_name, config.playback_device),
        )

        self._audio_queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=200)
        self._tts_queue: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
        self._cooldown_until: float = 0.0

        self._process_thread: Optional[threading.Thread] = None

        logger.info("流水线已创建")

    # ── 公开接口 ─────────────────────────────────────────────────────

    def start(self) -> None:
        logger.info("正在启动流水线 ...")
        self._stop_event.clear()
        self._audio.start(self._on_audio_samples)

        self._process_thread = threading.Thread(
            target=self._processing_loop,
            daemon=True,
            name="pipeline-main",
        )
        self._process_thread.start()
        logger.info("流水线已启动 (状态=IDLE，等待唤醒词)")

    def stop(self) -> None:
        logger.info("正在停止流水线 ...")
        self._stop_event.set()
        self.interrupt_tts()
        self._audio.stop()

        try:
            self._audio_queue.put_nowait(np.zeros(1, dtype=np.float32))
        except queue.Full:
            pass

        if self._process_thread is not None:
            self._process_thread.join(timeout=3.0)
            self._process_thread = None

        logger.info("流水线已停止")

    def interrupt_tts(self) -> None:
        if self._state.is_speaking():
            logger.info("正在打断 TTS 播放")
            self._tts.interrupt()
            while not self._tts_queue.empty():
                try:
                    self._tts_queue.get_nowait()
                except queue.Empty:
                    break
            self._state.transition(State.IDLE)
            self._send_gif_cmd("play standby 0 0")

    @property
    def state(self) -> State:
        return self._state.current

    # ── 音频回调 ──────────────────────────────────────────────────

    def _on_audio_samples(self, samples: np.ndarray) -> None:
        try:
            self._audio_queue.put_nowait(samples)
        except queue.Empty:
            logger.warning("音频队列已满，丢弃样本")

    # ── 主处理循环 ──────────────────────────────────────────────

    def _processing_loop(self) -> None:
        logger.info("处理循环已启动")

        while not self._stop_event.is_set():
            try:
                samples = self._audio_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            state = self._state.current

            if state == State.IDLE:
                self._handle_idle(samples)
            elif state == State.LISTENING:
                self._handle_listening(samples)
            elif state == State.SPEAKING:
                self._handle_speaking(samples)

        logger.info("处理循环已退出")

    # ── 文本有效性检查 ───────────────────────────────────────────

    @staticmethod
    def _is_meaningful_text(text: str) -> bool:
        stripped = text.strip()
        if not stripped:
            return False
        clean = "".join(
            c for c in stripped
            if c not in (".?!,;:，。！？；：、 ")
        )
        return len(clean) >= 3

    @staticmethod
    def _split_sentences(text: str) -> list[str]:
        sentences = []
        buf = ""
        for ch in text:
            buf += ch
            if ch in _SENTENCE_DELIMITERS:
                stripped = buf.strip()
                if stripped:
                    sentences.append(stripped)
                buf = ""
        remaining = buf.strip()
        if remaining:
            sentences.append(remaining)
        return sentences

    @staticmethod
    def _send_gif_cmd(cmd: str) -> None:
        try:
            with open(_GIF_FIFO, "w") as f:
                f.write(cmd + "\n")
        except (FileNotFoundError, OSError):
            pass

    # ── 各状态处理器 ─────────────────────────────────────────────

    def _handle_idle(self, samples: np.ndarray) -> None:
        if time.time() < self._cooldown_until:
            return

        keyword = self._wakeword.process(samples)
        if keyword is not None:
            logger.info("唤醒词触发: %s", keyword)
            self._asr.reset()
            self._state.transition(State.LISTENING)
            self._send_gif_cmd("play voice 0 0")
            self._asr.process(samples)
            print(f"\n[唤醒] {keyword}")

    def _handle_listening(self, samples: np.ndarray) -> None:
        self._asr.process(samples)

        # 每隔约 2 秒输出 ASR 内部状态（辅助排查 VAD 不触发的问题）
        if not hasattr(self, '_asr_debug_tick'):
            self._asr_debug_tick = 0
        self._asr_debug_tick += 1
        if self._asr_debug_tick % 20 == 0:
            logger.debug("ASR 状态: has_speech=%s silence=%d/%d submitted=%s ready=%s",
                        self._asr._has_speech, self._asr._silence_count,
                        self._asr._silence_chunks, self._asr._submitted,
                        self._asr._sentence_ready)

        if self._asr.detect_endpoint():
            text = self._asr.finalize()
            if text and self._is_meaningful_text(text):
                logger.info("用户说了: %s", text)
                print(f"\r[用户] {text}")
                self._state.transition(State.PROCESSING)

                assist_thread = threading.Thread(
                    target=self._run_ha_assist,
                    args=(text,),
                    daemon=True,
                    name="ha-assist",
                )
                assist_thread.start()
            else:
                logger.debug("ASR 结果为空或无意义，继续监听")
                self._asr.reset()

    def _handle_speaking(self, samples: np.ndarray) -> None:
        keyword = self._wakeword.process(samples)
        if keyword is not None:
            logger.info("播报中检测到唤醒词，执行打断")
            print(f"\n[打断] {keyword}")
            self.interrupt_tts()

    # ── HA Assist + TTS 流水线 ─────────────────────────────────

    def _run_ha_assist(self, user_text: str) -> None:
        self._state.transition(State.PROCESSING)
        print("[助手] ", end="", flush=True)

        try:
            reply = self._ha.process(user_text)
        except Exception as e:
            logger.exception("HA 请求失败")
            print(f"\n[错误] {e}")
            self._state.transition(State.IDLE)
            return

        print(reply)

        self._state.transition(State.SPEAKING)
        self._send_gif_cmd("play enjoy 1")

        if self._config.tts_mode == "full":
            self._tts.speak(reply)
        else:
            sentences = self._split_sentences(reply)
            for sentence in sentences:
                if self._stop_event.is_set() or self._tts.is_interrupted:
                    break
                self._tts.speak(sentence)

        print()

        if not self._tts.is_interrupted:
            self._cooldown_until = time.time() + self._config.asr_cooldown_ms / 1000.0
            self._send_gif_cmd("play standby 0 0")
            self._state.transition(State.IDLE)
            logger.info("TTS 播放完毕，进入冷却期 %dms，回到 IDLE",
                        self._config.asr_cooldown_ms)
