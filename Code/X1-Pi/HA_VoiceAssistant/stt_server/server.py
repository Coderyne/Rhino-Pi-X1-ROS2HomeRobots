"""AidVoice NPU SenseVoice Wyoming STT 服务端

将 AidVoiceASR 封装为 Wyoming 协议 STT 服务器，
使 Home Assistant 的 Assist 管道可以将其选为语音转文字引擎。

协议流程：
  Client → Server: Describe
  Server → Client: Info (声明 ASR 服务能力)
  Client → Server: Transcribe + AudioStart + AudioChunk* + AudioStop
  Server → Client: Transcript (返回识别文本)

运行方式：
  python3 stt_server/server.py --uri tcp://0.0.0.0:10300
"""

import argparse
import asyncio
import logging
import os
import sys
from typing import Optional

import numpy as np

from wyoming.asr import Transcribe, Transcript
from wyoming.audio import AudioChunk, AudioStart, AudioStop
from wyoming.event import async_read_event, async_write_event
from wyoming.info import AsrModel, AsrProgram, Attribution, Describe, Info
from wyoming.server import AsyncEventHandler, AsyncServer

# 添加项目根目录到 sys.path（追加末尾，避免覆盖系统 wyoming 包）
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.append(_PROJECT_ROOT)

from asr_aidvoice import AidVoiceASR, TYPE_SENSEVOICE

logger = logging.getLogger(__name__)

# ── 常量 ─────────────────────────────────────────────────────────────
_SERVER_NAME = "aidvoice-sensevoice"
_SERVER_VERSION = "1.0.0"
_DESCRIPTION = "AidVoice NPU SenseVoice 语音识别（支持中英日韩粤）"

# HA 默认的音频格式：16kHz, 16-bit 有符号整数, 单声道
_EXPECTED_RATE = 16000
_EXPECTED_WIDTH = 2
_EXPECTED_CHANNELS = 1


def _pcm_s16le_to_float32(audio_bytes: bytes) -> np.ndarray:
    """将 PCM s16le 字节流转为 float32 numpy 数组（归一化到 [-1, 1]）

    Args:
        audio_bytes: PCM 16-bit 有符号整数字节流

    Returns:
        float32 音频样本，shape=(n_samples,)
    """
    samples = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32)
    samples /= 32768.0
    return samples


class AidVoiceSTTHandler(AsyncEventHandler):
    """每个 Wyoming 客户端连接对应一个处理器实例"""

    def __init__(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        model_type: int = TYPE_SENSEVOICE,
    ) -> None:
        super().__init__(reader, writer)
        self._model_type = model_type

        # 当前转录会话状态
        self._asr: Optional[AidVoiceASR] = None
        self._audio_rate: int = 0
        self._audio_width: int = 0
        self._audio_channels: int = 0
        self._loop = asyncio.get_running_loop()

    # ── 事件分发 ──────────────────────────────────────────────────

    async def handle_event(self, event) -> bool:
        """根据事件类型分发处理

        Returns:
            False 表示断开连接，True 继续等待下一个事件
        """
        if Describe.is_type(event.type):
            await self._handle_describe()
            return True

        if Transcribe.is_type(event.type):
            await self._handle_transcribe(Transcribe.from_event(event))
            return True

        if AudioStart.is_type(event.type):
            await self._handle_audio_start(AudioStart.from_event(event))
            return True

        if AudioChunk.is_type(event.type):
            await self._handle_audio_chunk(event)
            return True

        if AudioStop.is_type(event.type):
            await self._handle_audio_stop()
            return False  # 转录完成，断开连接

        logger.warning("未知事件类型: %s", event.type)
        return True

    # ── Describe / Info ──────────────────────────────────────────

    async def _handle_describe(self) -> None:
        """响应 Describe 请求：声明本服务支持的 ASR 能力"""
        info = Info(
            asr=[
                AsrProgram(
                    name=_SERVER_NAME,
                    description=_DESCRIPTION,
                    attribution=Attribution(
                        name="aidlux / AidVoice",
                        url="https://github.com/aidlux",
                    ),
                    installed=True,
                    version=_SERVER_VERSION,
                    models=[
                        AsrModel(
                            name="sensevoice",
                            description="SenseVoice 多语言语音识别",
                            attribution=Attribution(
                                name="SenseVoice",
                                url="https://github.com/FunAudioLLM/SenseVoice",
                            ),
                            installed=True,
                            version="1.0",
                            languages=["zh", "en", "ja", "ko", "yue"],
                        ),
                    ],
                ),
            ],
        )
        await self.write_event(info.event())
        logger.info("已回复 Info（ASR 服务声明）")

    # ── Transcribe ──────────────────────────────────────────────

    async def _handle_transcribe(self, transcribe: Transcribe) -> None:
        """收到转录请求：创建 AidVoiceASR 实例"""
        logger.info(
            "收到 Transcribe 请求 (name=%s, language=%s)",
            transcribe.name, transcribe.language,
        )
        # 确保上一个会话已清理
        self._cleanup_asr()

        # 在线程池中创建 ASR 实例（避免阻塞事件循环）
        self._asr = await self._loop.run_in_executor(
            None,
            lambda: AidVoiceASR(model_type=self._model_type),
        )
        logger.info("AidVoiceASR 实例已创建")

    # ── AudioStart ──────────────────────────────────────────────

    async def _handle_audio_start(self, start: AudioStart) -> None:
        """收到音频流开始：记录音频格式"""
        self._audio_rate = start.rate
        self._audio_width = start.width
        self._audio_channels = start.channels
        logger.info(
            "音频流开始: %dHz, %d-bit, %dch",
            start.rate, start.width * 8, start.channels,
        )

        # 验证音频格式是否兼容
        if start.rate != _EXPECTED_RATE:
            logger.warning("采样率 %d 与预期 %d 不同，仍尝试处理", start.rate, _EXPECTED_RATE)
        if start.width != _EXPECTED_WIDTH:
            logger.warning(
                "位宽 %d 字节与预期 %d 不同，转换可能出错",
                start.width, _EXPECTED_WIDTH,
            )

    # ── AudioChunk ──────────────────────────────────────────────

    async def _handle_audio_chunk(self, event) -> None:
        """收到音频数据块：转换格式并送入 ASR 处理"""
        if self._asr is None:
            logger.warning("收到 AudioChunk 但没有活跃的 ASR 实例，丢弃")
            return

        # 解析 AudioChunk 中的二进制音频数据
        chunk = AudioChunk.from_event(event)
        audio_bytes = chunk.audio
        if not audio_bytes:
            return

        # PCM s16le → float32 转换
        samples = _pcm_s16le_to_float32(audio_bytes)

        # 在线程池中处理（AidVoiceASR.process 包含文件 I/O 和 NPU 调用）
        await self._loop.run_in_executor(None, self._asr.process, samples)

    # ── AudioStop ──────────────────────────────────────────────

    async def _handle_audio_stop(self) -> None:
        """收到音频流结束：强制提交识别并返回结果"""
        if self._asr is None:
            logger.warning("收到 AudioStop 但没有活跃的 ASR 实例")
            return

        logger.info("音频流结束，开始识别...")

        # 强制提交累积的音频到 NPU 推理
        await self._loop.run_in_executor(None, self._asr.force_submit)

        # 轮询等待识别结果（在事件循环中异步等待，不阻塞）
        text = ""
        timeout = 30.0  # 最长等待 30 秒
        poll_interval = 0.05
        elapsed = 0.0

        while elapsed < timeout:
            # 检查 endpoint 就绪状态（在线程池中执行，因涉及锁和 C 回调）
            ready = await self._loop.run_in_executor(
                None, self._asr.detect_endpoint,
            )
            if ready:
                break
            await asyncio.sleep(poll_interval)
            elapsed += poll_interval

        if elapsed >= timeout:
            logger.error("ASR 识别超时（%ds）", timeout)
            text = ""
        else:
            # 获取最终识别文本
            text = await self._loop.run_in_executor(None, self._asr.finalize)

        # 发送转录结果回客户端
        if text:
            logger.info("识别结果: %s", text)
            await self.write_event(Transcript(text=text).event())
        else:
            logger.warning("识别结果为空")
            await self.write_event(Transcript(text="").event())

        # 清理 ASR 实例
        self._cleanup_asr()

    # ── 辅助方法 ────────────────────────────────────────────────

    def _cleanup_asr(self) -> None:
        """安全销毁 ASR 实例"""
        if self._asr is not None:
            try:
                self._asr.destroy()
            except Exception:
                logger.exception("销毁 ASR 实例时出错")
            self._asr = None

    async def disconnect(self) -> None:
        """客户端断开时清理资源"""
        self._cleanup_asr()
        logger.info("客户端已断开，资源已清理")


# ── 服务器入口 ─────────────────────────────────────────────────────


def main() -> None:
    """启动 AidVoice Wyoming STT 服务器"""
    parser = argparse.ArgumentParser(
        description="AidVoice NPU SenseVoice Wyoming STT 服务器",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--uri", default="tcp://0.0.0.0:10300",
        help="监听地址，支持 tcp://host:port 或 unix:///path/to/socket",
    )
    parser.add_argument(
        "--model-type", type=int, default=TYPE_SENSEVOICE,
        help="ASR 模型类型（2=SenseVoice）",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="日志级别",
    )
    args = parser.parse_args()

    logging.basicConfig(
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        level=getattr(logging, args.log_level),
    )

    logger.info("=" * 50)
    logger.info("AidVoice Wyoming STT 服务器启动")
    logger.info("  监听地址: %s", args.uri)
    logger.info("  模型类型: %d (SenseVoice)", args.model_type)
    logger.info("=" * 50)

    # 创建并运行异步服务器
    async def _run() -> None:
        server = AsyncServer.from_uri(args.uri)

        def handler_factory(reader, writer):
            return AidVoiceSTTHandler(
                reader, writer, model_type=args.model_type,
            )

        await server.run(handler_factory)

    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        logger.info("服务器已停止")


if __name__ == "__main__":
    main()
