"""语音合成引擎：TTS 生成 + 流式播放"""

import logging
import queue
import threading
from typing import Optional

import numpy as np
import sherpa_onnx

logger = logging.getLogger(__name__)


def _resample(samples: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    """线性插值重采样

    当 TTS 输出采样率与播放设备不一致时使用。
    """
    if src_rate == dst_rate:
        return samples
    ratio = dst_rate / src_rate
    n = int(len(samples) * ratio)
    x_old = np.arange(len(samples))
    x_new = np.linspace(0, len(samples) - 1, n)
    return np.interp(x_new, x_old, samples).astype(np.float32)


class TTSEngine:
    """语音合成引擎，支持流式生成与播放

    采用回调机制：TTS 生成的音频 chunk 实时送入播放队列，
    无需等待整句合成完成即可开始播放，降低延迟。

    Args:
        model: TTS 模型路径 (.onnx)
        tokens: tokens.txt 路径
        lexicon: lexicon.txt 路径
        rule_fsts: 文本归一化 FST 文件（逗号分隔）
        num_threads: 推理线程数
        speed: 语速因子 (>1 更快)
        sid: 说话人 ID（多说话人模型）
        playback_device: sounddevice 输出设备索引，None 使用默认
    """

    def __init__(
        self,
        model: str,
        tokens: str,
        lexicon: str = "",
        rule_fsts: str = "",
        num_threads: int = 1,
        speed: float = 1.0,
        sid: int = 0,
        volume_gain: float = 1.0,
        playback_device: Optional[int] = None,
    ) -> None:
        import sounddevice as sd  # 延迟导入，避免在未安装时影响模块加载

        self._sd = sd
        self._playback_device = playback_device

        # 构建 TTS 配置
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=model,
                    lexicon=lexicon,
                    tokens=tokens,
                ),
                num_threads=num_threads,
                debug=False,
                provider="cpu",
            ),
            rule_fsts=rule_fsts if rule_fsts else "",
            max_num_sentences=1,
        )

        if not tts_config.validate():
            raise RuntimeError("TTS 配置校验失败")

        self._tts = sherpa_onnx.OfflineTts(tts_config)
        self._tts_sample_rate = self._tts.sample_rate  # 模型输出采样率（如 22050）
        self._speed = speed
        self._sid = sid
        self._volume_gain = volume_gain

        # 检测播放设备采样率，必要时重采样
        self._playback_sample_rate = self._detect_device_sample_rate()
        self._need_resample = (self._tts_sample_rate != self._playback_sample_rate)

        if self._need_resample:
            logger.info("重采样已启用: %d Hz -> %d Hz",
                        self._tts_sample_rate, self._playback_sample_rate)

        # 播放状态
        self._buffer: queue.Queue[np.ndarray] = queue.Queue()
        self._playback_event = threading.Event()
        self._playback_thread: Optional[threading.Thread] = None
        self._stopped = False

        # 打断标志
        self._interrupt_requested = threading.Event()

        logger.info(
            "TTS 引擎已初始化 (model=%s, tts_rate=%d, playback_rate=%d, speed=%.1f)",
            model, self._tts_sample_rate, self._playback_sample_rate, speed,
        )

    def _detect_device_sample_rate(self) -> int:
        """查询播放设备的默认采样率"""
        try:
            if self._playback_device is not None:
                info = self._sd.query_devices(self._playback_device)
            else:
                info = self._sd.query_devices(kind="output")
            rate = int(info["default_samplerate"])
            if rate > 0:
                return rate
        except Exception:
            logger.warning("无法检测设备采样率，使用 TTS 原始采样率")
        return self._tts_sample_rate

    def speak(self, text: str) -> None:
        """合成文本并同步播放

        阻塞直到播放完成或被 interrupt() 打断。
        """
        if not text.strip():
            return

        self._interrupt_requested.clear()
        self._stopped = False
        self._playback_event.clear()

        # 清空上次残留的音频
        while not self._buffer.empty():
            try:
                self._buffer.get_nowait()
            except queue.Empty:
                break

        # 启动播放线程
        self._playback_thread = threading.Thread(
            target=self._playback_loop, daemon=True, name="tts-playback"
        )
        self._playback_thread.start()

        # 生成音频（回调函数将 chunk 送入 self._buffer）
        logger.info("TTS 正在生成: %s", text[:80])
        gen_config = sherpa_onnx.GenerationConfig()
        gen_config.sid = self._sid
        gen_config.speed = self._speed
        gen_config.silence_scale = 0.2

        self._tts.generate(
            text,
            gen_config,
            callback=self._audio_callback,
        )

        # 生成完毕，通知播放线程（不主动 set event，由 _sd_callback 在 buffer 消费完后触发）
        self._stopped = True

        # 等待播放线程播完剩余音频
        if self._playback_thread is not None:
            self._playback_thread.join()
            self._playback_thread = None

    def interrupt(self) -> None:
        """请求打断当前 TTS 播放

        可从任意线程安全调用。speak() 会在打断后尽快返回。
        """
        if not self._interrupt_requested.is_set():
            self._interrupt_requested.set()
            logger.info("TTS 打断已请求")

    @property
    def is_interrupted(self) -> bool:
        """是否已请求打断"""
        return self._interrupt_requested.is_set()

    @property
    def sample_rate(self) -> int:
        """播放设备采样率"""
        return self._playback_sample_rate

    def _audio_callback(self, samples: np.ndarray, progress: float) -> int:
        """TTS 生成过程中的回调（由 C++ 调用）

        Returns:
            1 继续生成，0 停止生成
        """
        if self._interrupt_requested.is_set():
            return 0
        # 按需重采样到播放设备采样率
        if self._need_resample:
            samples = _resample(samples, self._tts_sample_rate, self._playback_sample_rate)
        # 应用音量增益
        if self._volume_gain != 1.0:
            samples = np.clip(samples * self._volume_gain, -1.0, 1.0)
        self._buffer.put(samples)
        return 1

    def _playback_loop(self) -> None:
        """后台播放线程：从队列取音频并通过 sounddevice 输出"""
        sample_rate = self._playback_sample_rate

        def _sd_callback(
            outdata: np.ndarray,
            frames: int,
            time_info,
            status: self._sd.CallbackFlags,
        ) -> None:
            # 打断且队列已空 → 结束播放
            if self._interrupt_requested.is_set() and self._buffer.empty():
                self._playback_event.set()
                outdata.fill(0)
                return

            # 从队列取音频填充输出缓冲区
            n = 0
            while n < frames and not self._buffer.empty():
                remaining = frames - n
                try:
                    chunk = self._buffer.queue[0]
                except IndexError:
                    break

                k = chunk.shape[0]
                if remaining <= k:
                    outdata[n : n + remaining, 0] = chunk[:remaining]
                    self._buffer.queue[0] = chunk[remaining:]
                    if self._buffer.queue[0].shape[0] == 0:
                        self._buffer.get()
                    n = frames
                    break

                outdata[n : n + k, 0] = chunk
                self._buffer.get()
                n += k

            # 剩余部分填静音
            if n < frames:
                outdata[n:, 0] = 0

            # 生成完毕且队列已空 → 通知主线程
            if self._stopped and self._buffer.empty():
                self._playback_event.set()

        try:
            with self._sd.OutputStream(
                channels=1,
                callback=_sd_callback,
                dtype="float32",
                samplerate=sample_rate,
                blocksize=1024,
                device=self._playback_device,
            ):
                self._playback_event.wait()
        except Exception:
            logger.exception("播放错误")
