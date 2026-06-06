#!/bin/bash
# ============================================================
#  AidVoice NPU SenseVoice Wyoming STT 服务器 - 启动脚本
#  启动后将接收 HA Assist 管道的音频流并返回识别文本
# ============================================================

WORK_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "${WORK_DIR}" || exit 1

# ── 监听地址 ─────────────────────────────────────────────────
# 使用 host 网络模式，HA 可通过 localhost:10300 访问
LISTEN_URI="tcp://0.0.0.0:10300"

# ── ASR 模型类型 ─────────────────────────────────────────────
# 2 = SenseVoice（支持中/英/日/韩/粤）
MODEL_TYPE=2

# ── 日志级别 ─────────────────────────────────────────────────
LOG_LEVEL="INFO"

echo "=========================================="
echo "  AidVoice Wyoming STT 服务器"
echo "=========================================="
echo "  监听地址: ${LISTEN_URI}"
echo "  模型类型: ${MODEL_TYPE} (SenseVoice)"
echo "  工作目录: ${WORK_DIR}"
echo "=========================================="

exec python3 stt_server/server.py \
    --uri "${LISTEN_URI}" \
    --model-type "${MODEL_TYPE}" \
    --log-level "${LOG_LEVEL}"
