"""ALSA 麦克风音频采集模块"""

import logging
import threading
from typing import Callable, Optional

import numpy as np
import sherpa_onnx

logger = logging.getLogger(__name__)


class AudioCapture:
    """后台线程从 ALSA 设备持续读取音频

    每次读取一个 chunk（默认 100ms），通过回调函数传递给调用者。
    音频格式：1-D float32 numpy 数组，16kHz 采样率。

    Args:
        device_name: ALSA 设备名，如 "plughw:2,0"
        sample_rate: 采样率 (Hz)
        chunk_duration: 每次读取时长 (秒)
    """

    def __init__(
        self,
        device_name: str,
        sample_rate: int = 16000,
        chunk_duration: float = 0.1,
    ) -> None:
        self._device_name = device_name
        self._sample_rate = sample_rate
        self._samples_per_read = int(chunk_duration * sample_rate)
        self._alsa: Optional[sherpa_onnx.Alsa] = None
        self._thread: Optional[threading.Thread] = None
        self._stopped = threading.Event()

    def start(self, callback: Callable[[np.ndarray], None]) -> None:
        """启动采集线程

        Args:
            callback: 每读取一个音频 chunk 就调用此回调
        """
        logger.info("正在打开 ALSA 设备: %s", self._device_name)
        self._alsa = sherpa_onnx.Alsa(self._device_name)
        self._stopped.clear()

        def _run() -> None:
            logger.info("音频采集线程已启动")
            while not self._stopped.is_set():
                try:
                    samples = self._alsa.read(self._samples_per_read)
                    callback(samples)
                except Exception:
                    logger.exception("音频采集异常")
                    break
            logger.info("音频采集线程已停止")

        self._thread = threading.Thread(target=_run, daemon=True, name="audio-capture")
        self._thread.start()

    def stop(self) -> None:
        """通知采集线程停止并等待其退出"""
        self._stopped.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    @property
    def is_running(self) -> bool:
        """采集线程是否正在运行"""
        return self._thread is not None and self._thread.is_alive()
