#ifndef ASR_BRIDGE_H
#define ASR_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void* AsrHandle;

typedef void (*AsrResultCallback)(int status, const char* text, int id, void* userdata);
typedef void (*AsrErrorCallback)(int error_code, const char* message, void* userdata);

AsrHandle asr_bridge_create(int model_type, int feature_type);
int asr_bridge_init(AsrHandle h);
int asr_bridge_set_mode(AsrHandle h, int streaming);
int asr_bridge_set_callback(AsrHandle h, AsrResultCallback on_result, AsrErrorCallback on_error, void* userdata);
int asr_bridge_set_echo_ms(AsrHandle h, int ms);
int asr_bridge_set_step_ms(AsrHandle h, int ms);
int asr_bridge_write_float(AsrHandle h, const float* data, int len);
int asr_bridge_audio_mic(AsrHandle h, int device_id);
int asr_bridge_stop(AsrHandle h);
int asr_bridge_destroy(AsrHandle h);

#ifdef __cplusplus
}
#endif

#endif
