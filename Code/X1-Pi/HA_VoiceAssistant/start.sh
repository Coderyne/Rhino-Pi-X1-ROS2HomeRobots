#!/bin/bash
# ============================================================
#  HA 语音助手 - 一键启动脚本
#  基于 AidVoice NPU SenseVoice ASR + HA Conversation API
# ============================================================

WORK_DIR="/home/aidlux/MIC_ASR/HA_VoiceAssistant"

# ── HA 配置 ─────────────────────────────────────────────────
HA_URL="http://localhost:8123"
HA_AGENT_ID="conversation.extended_openai_conversation"

# ── 音频设备 (自动查找 USB 麦克风) ─────────────────────────
detect_mic() {
    local device=""
    device=$(arecord -l 2>/dev/null | grep -i "respeaker\|ArrayUAC\|ReSpeaker" | head -1 | sed -n 's/.*card \([0-9]*\).*device \([0-9]*\).*/plughw:\1,\2/p')
    if [ -n "$device" ]; then
        echo "$device"
        return
    fi
    device=$(arecord -l 2>/dev/null | grep -i "usb" | head -1 | sed -n 's/.*card \([0-9]*\).*device \([0-9]*\).*/plughw:\1,\2/p')
    if [ -n "$device" ]; then
        echo "$device"
        return
    fi
    echo "plughw:0,0"
}

DEVICE_NAME=$(detect_mic)

# ── 音频输出设备 ────────────────────────────────────────────
PLAYBACK_DEVICE="plughw:2,0"

# ── 模型路径 ────────────────────────────────────────────────
MODELS_DIR="${WORK_DIR}/models"

# KWS 唤醒词模型
KWS_MODEL_DIR="${MODELS_DIR}/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"
KWS_ENCODER="${KWS_MODEL_DIR}/encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx"
KWS_DECODER="${KWS_MODEL_DIR}/decoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx"
KWS_JOINER="${KWS_MODEL_DIR}/joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx"
KWS_TOKENS="${KWS_MODEL_DIR}/tokens.txt"
KWS_KEYWORDS="${KWS_MODEL_DIR}/keywords.txt"

# TTS 语音合成模型
TTS_MODEL_DIR="${MODELS_DIR}/vits-piper-zh_CN-xiao_ya-medium-fp16"
TTS_MODEL="${TTS_MODEL_DIR}/zh_CN-xiao_ya-medium.fp32.onnx"
TTS_TOKENS="${TTS_MODEL_DIR}/tokens.txt"
TTS_LEXICON="${TTS_MODEL_DIR}/lexicon.txt"
TTS_RULE_FSTS="${TTS_MODEL_DIR}/date.fst,${TTS_MODEL_DIR}/number.fst,${TTS_MODEL_DIR}/phone.fst"

# ── 参数 ────────────────────────────────────────────────────
KWS_THRESHOLD=0.10
KWS_SCORE=1.5
ASR_STABLE_THRESHOLD=10
ASR_MIN_QUERY_CHARS=2
ASR_COOLDOWN_MS=1200
TTS_SPEED=1.0
VOLUME_GAIN=3.0
SYSTEM_VOLUME=100

cd "${WORK_DIR}" || exit 1

# 系统音量
if [ -n "${PLAYBACK_DEVICE}" ]; then
    CARD_IDX=$(echo "${PLAYBACK_DEVICE}" | sed -n 's/.*:\([0-9]*\),.*/\1/p')
    if [ -n "${CARD_IDX}" ]; then
        amixer -c "${CARD_IDX}" cset numid=4 "${SYSTEM_VOLUME}" >/dev/null 2>&1
        echo "  系统音量: card ${CARD_IDX} PCM Playback = ${SYSTEM_VOLUME}/147"
    fi
fi

echo "=========================================="
echo "  HA 语音助手"
echo "=========================================="
echo "  HA:       ${HA_URL} (${HA_AGENT_ID})"
echo "  麦克风:   ${DEVICE_NAME}"
echo "  播放设备: ${PLAYBACK_DEVICE:-自动}"
echo "  唤醒阈值: ${KWS_THRESHOLD}"
echo "  ASR 稳定: ${ASR_STABLE_THRESHOLD} 次"
echo "  冷却期:   ${ASR_COOLDOWN_MS}ms"
echo "  语速:     ${TTS_SPEED}"
echo "=========================================="

exec python3 start.py \
    --kws-encoder    "${KWS_ENCODER}" \
    --kws-decoder    "${KWS_DECODER}" \
    --kws-joiner     "${KWS_JOINER}" \
    --kws-tokens     "${KWS_TOKENS}" \
    --kws-keywords   "${KWS_KEYWORDS}" \
    --kws-threshold  "${KWS_THRESHOLD}" \
    --kws-score      "${KWS_SCORE}" \
    --asr-stable-threshold "${ASR_STABLE_THRESHOLD}" \
    --asr-min-query-chars "${ASR_MIN_QUERY_CHARS}" \
    --asr-cooldown-ms "${ASR_COOLDOWN_MS}" \
    --tts-model      "${TTS_MODEL}" \
    --tts-tokens     "${TTS_TOKENS}" \
    --tts-lexicon    "${TTS_LEXICON}" \
    --tts-rule-fsts  "${TTS_RULE_FSTS}" \
    --tts-speed      "${TTS_SPEED}" \
    --volume-gain    "${VOLUME_GAIN}" \
    --ha-url         "${HA_URL}" \
    --ha-agent-id    "${HA_AGENT_ID}" \
    --device-name    "${DEVICE_NAME}" \
    --playback-device "${PLAYBACK_DEVICE}" \
    "$@"
