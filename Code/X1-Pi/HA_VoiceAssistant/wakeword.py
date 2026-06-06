"""唤醒词检测模块"""

import logging
from typing import Optional

import numpy as np
import sherpa_onnx

logger = logging.getLogger(__name__)


class WakeWordDetector:
    """基于 sherpa_onnx.KeywordSpotter 的唤醒词检测器

    持续接收音频流，检测到唤醒词时返回关键词文本。

    Args:
        encoder: KWS encoder ONNX 模型路径
        decoder: KWS decoder ONNX 模型路径
        joiner: KWS joiner ONNX 模型路径
        tokens: tokens.txt 路径
        keywords_file: keywords.txt 路径
        num_threads: 推理线程数
        keywords_threshold: 触发阈值，越低越灵敏
        keywords_score: 关键词 token 加分值
        sample_rate: 音频采样率 (Hz)
    """

    def __init__(
        self,
        encoder: str,
        decoder: str,
        joiner: str,
        tokens: str,
        keywords_file: str,
        num_threads: int = 1,
        keywords_threshold: float = 0.25,
        keywords_score: float = 1.0,
        sample_rate: int = 16000,
        gain: float = 3.0,
    ) -> None:
        self._gain = gain
        self._spotter = sherpa_onnx.KeywordSpotter(
            tokens=tokens,
            encoder=encoder,
            decoder=decoder,
            joiner=joiner,
            keywords_file=keywords_file,
            num_threads=num_threads,
            sample_rate=sample_rate,
            keywords_score=keywords_score,
            keywords_threshold=keywords_threshold,
        )

        self._stream = self._spotter.create_stream()
        self._sample_rate = sample_rate
        logger.info("唤醒词检测器已初始化 (keywords_file=%s)", keywords_file)

    def process(self, samples: np.ndarray) -> Optional[str]:
        """送入音频样本，返回检测到的唤醒词文本，未检测到返回 None"""
        samples = np.asarray(samples, dtype=np.float32)
        if self._gain != 1.0:
            samples = np.clip(samples * self._gain, -1.0, 1.0)
        self._stream.accept_waveform(self._sample_rate, samples)

        if self._spotter.is_ready(self._stream):
            self._spotter.decode_stream(self._stream)

        result = self._spotter.get_result(self._stream)

        if result:
            self._spotter.reset_stream(self._stream)
            logger.info("检测到唤醒词: %s", result)
            return result

        return None

    def reset(self) -> None:
        """重置内部流状态"""
        self._spotter.reset_stream(self._stream)
