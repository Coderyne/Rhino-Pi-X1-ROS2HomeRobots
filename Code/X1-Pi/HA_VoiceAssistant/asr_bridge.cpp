#include "asr_bridge.h"
#include "aidlux/aidvoice/aidvoice_speech.hpp"

#include <memory>
#include <cstring>

using namespace AidLux::AidVoice;

struct AsrBridgeCtx {
    std::shared_ptr<AidVoiceASR> asr;
    AsrResultCallback on_result = nullptr;
    AsrErrorCallback on_error = nullptr;
    void* userdata = nullptr;
};

class AsrBridgeCallbacks : public ASRCallbacks {
public:
    explicit AsrBridgeCallbacks(AsrBridgeCtx* ctx) : ctx_(ctx) {}

    void onResult(const AsrResult& result) override {
        if (ctx_->on_result) {
            ctx_->on_result(
                static_cast<int>(result.status),
                result.text.c_str(),
                result.id,
                ctx_->userdata
            );
        }
    }

    void onError(const AsrError& error) override {
        if (ctx_->on_error) {
            ctx_->on_error(
                error.error_code,
                error.message.c_str(),
                ctx_->userdata
            );
        }
    }

private:
    AsrBridgeCtx* ctx_;
};

extern "C" {

AsrHandle asr_bridge_create(int model_type, int feature_type) {
    auto* ctx = new AsrBridgeCtx();

    FeatureConfig cfg;
    cfg.feature_type = static_cast<FeatureType>(feature_type);
    cfg.model_type = static_cast<ModelType>(model_type);

    ctx->asr = create_asr(cfg);
    if (!ctx->asr) {
        delete ctx;
        return nullptr;
    }

    return static_cast<AsrHandle>(ctx);
}

int asr_bridge_init(AsrHandle h) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    return ctx->asr->init();
}

int asr_bridge_set_mode(AsrHandle h, int mode) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    ctx->asr->set_mode(static_cast<ASRMode>(mode));
    return 0;
}

int asr_bridge_set_callback(AsrHandle h, AsrResultCallback on_result, AsrErrorCallback on_error, void* userdata) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    ctx->on_result = on_result;
    ctx->on_error = on_error;
    ctx->userdata = userdata;
    auto* cb = new AsrBridgeCallbacks(ctx);
    ctx->asr->set_callback(cb);
    return 0;
}

int asr_bridge_set_echo_ms(AsrHandle h, int ms) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    ctx->asr->set_echo_ms(ms);
    return 0;
}

int asr_bridge_set_step_ms(AsrHandle h, int ms) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    ctx->asr->set_step_ms(ms);
    return 0;
}

int asr_bridge_write_float(AsrHandle h, const float* data, int len) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    std::vector<float> audio(data, data + len);
    return ctx->asr->write(audio);
}

int asr_bridge_audio_mic(AsrHandle h, int device_id) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    return ctx->asr->audio_mircophone(device_id);
}

int asr_bridge_stop(AsrHandle h) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    return ctx->asr->stop();
}

int asr_bridge_destroy(AsrHandle h) {
    if (!h) return -1;
    auto* ctx = static_cast<AsrBridgeCtx*>(h);
    ctx->asr->asr_destory();
    delete ctx;
    return 0;
}

} // extern "C"
