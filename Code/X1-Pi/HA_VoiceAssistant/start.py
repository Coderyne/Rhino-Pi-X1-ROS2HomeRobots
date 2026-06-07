#!/usr/bin/env python3
"""HA 语音助手入口 — 外部管道模式

解析命令行参数，构建配置，启动流水线（KWS → ASR → HA API → TTS）。
"""
import argparse
import logging
import os
import signal
import sys
import time

from config import AssistantConfig
from pipeline import Pipeline

_MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
_KEYS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keys.txt")


def _read_token_from_file(path: str) -> str:
    """从 keys.txt 读取 HA_TOKEN 行"""
    if not os.path.isfile(path):
        return ""
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("HA_TOKEN="):
                return line[len("HA_TOKEN="):].strip()
    return ""


def get_args() -> argparse.Namespace:
    """解析全部命令行参数"""
    parser = argparse.ArgumentParser(
        description="HA 语音助手: KWS -> ASR -> HA Assist -> TTS",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # ── KWS 关键词唤醒 ────────────────────────────────────────────────
    g = parser.add_argument_group("KWS (关键词唤醒)")
    g.add_argument("--kws-encoder", required=True, help="KWS encoder ONNX 路径")
    g.add_argument("--kws-decoder", required=True, help="KWS decoder ONNX 路径")
    g.add_argument("--kws-joiner", required=True, help="KWS joiner ONNX 路径")
    g.add_argument("--kws-tokens", required=True, help="KWS tokens.txt 路径")
    g.add_argument("--kws-keywords", required=True, help="keywords.txt 路径")
    g.add_argument("--kws-num-threads", type=int, default=1, help="KWS 推理线程数")
    g.add_argument("--kws-threshold", type=float, default=0.10, help="唤醒灵敏度阈值")
    g.add_argument("--kws-score", type=float, default=1.5, help="关键词加分值")

    # ── TTS 语音合成 ──────────────────────────────────────────────────
    g = parser.add_argument_group("TTS (VITS-Piper 语音合成)")
    g.add_argument("--tts-model", required=True, help="TTS 模型 ONNX 路径")
    g.add_argument("--tts-tokens", required=True, help="TTS tokens.txt 路径")
    g.add_argument("--tts-lexicon", default="", help="lexicon.txt 路径")
    g.add_argument("--tts-rule-fsts", default="",
                    help="文本归一化 FST 文件（逗号分隔）")
    g.add_argument("--tts-num-threads", type=int, default=1, help="TTS 推理线程数")
    g.add_argument("--tts-speed", type=float, default=1.0, help="语速 (>1 更快)")
    g.add_argument("--volume-gain", type=float, default=1.0, help="音量增益")
    g.add_argument("--tts-mode", default="sentence", choices=["sentence", "full"],
                    help="TTS 播报模式 (sentence=逐句, full=整段)")

    # ── HA Conversation API ──────────────────────────────────────────
    g = parser.add_argument_group("HA Conversation API")
    g.add_argument("--ha-url", default="http://localhost:8123",
                    help="HA 服务器地址")
    g.add_argument("--ha-token", default="",
                    help="HA 长期访问令牌（若未提供则从 keys.txt 读取）")
    g.add_argument("--ha-agent-id", default="conversation.qwen3_4b_8550",
                    help="对话助手 agent_id")
    g.add_argument("--ha-timeout", type=int, default=30, help="HTTP 请求超时 (秒)")

    # ── ASR 参数 ──────────────────────────────────────────────────────
    g = parser.add_argument_group("ASR (AidVoice NPU SenseVoice)")
    g.add_argument("--asr-stable-threshold", type=int, default=10,
                    help="连续稳定次数阈值（判定说完）")
    g.add_argument("--asr-min-query-chars", type=int, default=2,
                    help="最短有效查询长度（码点数）")
    g.add_argument("--asr-cooldown-ms", type=int, default=1200,
                    help="TTS 回复后的冷却期（毫秒）")

    # ── 音频设备 ─────────────────────────────────────────────────────
    g = parser.add_argument_group("音频设备")
    g.add_argument("--device-name", default="plughw:2,0",
                    help="ALSA 录音设备名")
    g.add_argument("--playback-device", default="",
                    help="ALSA 播放设备名，留空自动检测")

    return parser.parse_args()


def build_config(args: argparse.Namespace) -> AssistantConfig:
    """将命令行参数映射到 AssistantConfig"""
    # 优先使用命令行传入的 token，否则从 keys.txt 读取
    token = args.ha_token or _read_token_from_file(_KEYS_FILE)

    return AssistantConfig(
        kws_encoder=args.kws_encoder,
        kws_decoder=args.kws_decoder,
        kws_joiner=args.kws_joiner,
        kws_tokens=args.kws_tokens,
        kws_keywords_file=args.kws_keywords,
        kws_num_threads=args.kws_num_threads,
        kws_keywords_threshold=args.kws_threshold,
        kws_keywords_score=args.kws_score,
        asr_stable_threshold=args.asr_stable_threshold,
        asr_min_query_chars=args.asr_min_query_chars,
        asr_cooldown_ms=args.asr_cooldown_ms,
        tts_model=args.tts_model,
        tts_tokens=args.tts_tokens,
        tts_lexicon=args.tts_lexicon,
        tts_rule_fsts=args.tts_rule_fsts,
        tts_num_threads=args.tts_num_threads,
        tts_speed=args.tts_speed,
        tts_volume_gain=args.volume_gain,
        tts_mode=args.tts_mode,
        ha_url=args.ha_url,
        ha_token=token,
        ha_agent_id=args.ha_agent_id,
        ha_timeout=args.ha_timeout,
        device_name=args.device_name,
        playback_device=args.playback_device,
    )


def main() -> None:
    """启动入口：解析参数 → 构建配置 → 启动流水线 → 等待 Ctrl+C"""
    args = get_args()

    logging.basicConfig(
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        level=logging.INFO,
    )

    config = build_config(args)
    config.validate()

    pipeline = Pipeline(config)

    def _signal_handler(sig, frame):
        print("\n[退出] 正在停止 ...")
        pipeline.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)

    pipeline.start()

    print("=" * 50)
    print("  HA 语音助手已启动")
    print("  说唤醒词开始对话，按 Ctrl+C 退出")
    print("=" * 50)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
