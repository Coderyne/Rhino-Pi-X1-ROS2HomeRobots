from dataclasses import dataclass
from pathlib import Path


@dataclass
class AssistantConfig:

    # ── KWS 关键词唤醒 ─────────────────────────────────────────────────
    kws_encoder: str = ""
    kws_decoder: str = ""
    kws_joiner: str = ""
    kws_tokens: str = ""
    kws_keywords_file: str = ""
    kws_num_threads: int = 1
    kws_keywords_threshold: float = 0.10
    kws_keywords_score: float = 1.5

    # ── ASR（固定 AidVoice NPU SenseVoice）────────────────────────────
    asr_stable_threshold: int = 10
    asr_min_query_chars: int = 2
    asr_cooldown_ms: int = 1200

    # ── TTS 语音合成 ───────────────────────────────────────────────────
    tts_model: str = ""
    tts_tokens: str = ""
    tts_lexicon: str = ""
    tts_rule_fsts: str = ""
    tts_num_threads: int = 1
    tts_speed: float = 1.0
    tts_sid: int = 0
    tts_volume_gain: float = 1.0
    tts_mode: str = "sentence"  # "sentence" 逐句 | "full" 整段

    # ── HA Conversation API ────────────────────────────────────────────
    ha_url: str = "http://localhost:8123"
    ha_token: str = ""
    ha_agent_id: str = "conversation.qwen3_4b_8550"
    ha_timeout: int = 30

    # ── 音频设备 ──────────────────────────────────────────────────────
    device_name: str = "plughw:2,0"
    playback_device: str = ""
    sample_rate: int = 16000

    def validate(self) -> None:
        required_files = {
            "kws_encoder": self.kws_encoder,
            "kws_decoder": self.kws_decoder,
            "kws_joiner": self.kws_joiner,
            "kws_tokens": self.kws_tokens,
            "kws_keywords_file": self.kws_keywords_file,
            "tts_model": self.tts_model,
            "tts_tokens": self.tts_tokens,
            "tts_lexicon": self.tts_lexicon,
        }
        for name, path in required_files.items():
            if not Path(path).is_file():
                raise FileNotFoundError(
                    f"[config] {name} 文件不存在: {path}"
                )
        if not self.ha_token:
            raise ValueError("[config] HA_TOKEN 未设置")
