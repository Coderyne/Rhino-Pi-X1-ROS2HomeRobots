# Qwen3-4B-Instruct 端侧部署指南 — AidGen C++ API + AidGenSE OpenAI API

## 概述

通过MMS拉取Qwen3-4B-Instruct模型，并将模型注册到 AidGenSE，提供标准 OpenAI API

## 环境

| 项目 | 规格 |
|------|------|
| 设备 | Rhino Pi-X1 |
| SoC | Qualcomm QCS8550 |
| 系统 | Ubuntu 22.04 |

## 通过MMS拉取Qwen3-4B-Instruct模型资源

```bash
cd ~
mkdir aidllm && cd aidllm
mkdir Qwen3-4B-Instruct && cd Qwen3-4B-Instruct
mms get -m Qwen3-4B-Instruct-2507 -p w4a16 -c qcs8550 -b qnn2.36 -d qwen3-4b-instruct
unzip qnn236_qcs8550_cl4096.zip
# 模型将被解压到 ~/aidllm/Qwen3-4B-Instruct/qnn236_qcs8550_cl4096/
```

## 导入Tokenizer
MMS的模型包中不包含 tokenizer，需复用 Qwen3-4B (cl2048) 的 `qwen3-4b-tokenizer.json`\
将qwen3-4b-tokenizer.json复制到当前模型目录即可

## 模型目录结构

```
/home/aidlux/aidllm/Qwen3-4B-Instruct/qnn236_qcs8550_cl4096/
├── qwen3-4b-instruct_qnn236_qcs8550_cl4096_1_of_3.aidem   # 模型分片 1/3
├── qwen3-4b-instruct_qnn236_qcs8550_cl4096_2_of_3.aidem   # 模型分片 2/3
├── qwen3-4b-instruct_qnn236_qcs8550_cl4096_3_of_3.aidem   # 模型分片 3/3
├── kv-cache.primary.qnn-htp                                 # KV Cache 配置
├── htp_backend_ext_config.json                              # HTP 后端扩展配置
├── qwen3-4b-tokenizer.json                                  # Tokenizer
└── chat.txt                                                 # 对话模板示例
```

---
将已部署的 cl4096 模型注册到 AidGenSE，提供标准 OpenAI API。
## 安装AidGenSE

```bash
sudo aid-pkg update
sudo aid-pkg -i aidgense
```

## 拷贝模型文件到 AidGenSE 目录

```bash
MODEL_DIR="/opt/aidlux/app/aid-openai-api/res/models/Qwen3-4B-Instruct-cl4096"
mkdir -p "$MODEL_DIR"

SRC="/home/aidlux/aidllm/Qwen3-4B-Instruct/qnn236_qcs8550_cl4096"
cp "$SRC"/qwen3-4b-instruct_qnn236_qcs8550_cl4096_*.aidem "$MODEL_DIR/"
cp "$SRC"/kv-cache.primary.qnn-htp "$MODEL_DIR/"
cp "$SRC"/qwen3-4b-tokenizer.json "$MODEL_DIR/"
cp "$SRC"/htp_backend_ext_config.json "$MODEL_DIR/"
```


### 创建模型配置 JSON

创建 `$MODEL_DIR/Qwen3-4B-Instruct-cl4096.json`：

```json
{
    "backend_type": "genie",
    "prefix_path": "/opt/aidlux/app/aid-openai-api/res/models/Qwen3-4B-Instruct-cl4096/kv-cache.primary.qnn-htp",
    "model": {
        "path": [
            "/opt/aidlux/app/aid-openai-api/res/models/Qwen3-4B-Instruct-cl4096/qwen3-4b-instruct_qnn236_qcs8550_cl4096_1_of_3.aidem",
            "/opt/aidlux/app/aid-openai-api/res/models/Qwen3-4B-Instruct-cl4096/qwen3-4b-instruct_qnn236_qcs8550_cl4096_2_of_3.aidem",
            "/opt/aidlux/app/aid-openai-api/res/models/Qwen3-4B-Instruct-cl4096/qwen3-4b-instruct_qnn236_qcs8550_cl4096_3_of_3.aidem"
        ]
    }
}
```

### 注册到 `api_cfg.json`

编辑 `/opt/aidlux/app/aid-openai-api/api_cfg.json`，在 `model_cfg_list` 中新增：

```json
{
    "model_id": "Qwen3-4B-Instruct-cl4096",
    "model_create": "1774523039284",
    "model_owner": "aplux",
    "cfg_path": "./models/Qwen3-4B-Instruct-cl4096/Qwen3-4B-Instruct-cl4096.json",
    "prompt_template_type": "qwen3"
}
```

`api_cfg.json` 中已有 `qwen3` 的 prompt template，无需重复添加。

### 验证注册

```bash
aidllm list api
```

输出中应包含 `Qwen3-4B-Instruct-cl4096`。

### 启动服务

```bash
aidllm start api -m Qwen3-4B-Instruct-cl4096
```

默认监听 `0.0.0.0:8888`。

### API 测试

**非流式请求**：

```bash
curl http://127.0.0.1:8888/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "Qwen3-4B-Instruct-cl4096",
    "messages": [{"role": "user", "content": "中国的首都是哪里？"}]
  }'
```

**流式请求**：

```bash
curl http://127.0.0.1:8888/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "Qwen3-4B-Instruct-cl4096",
    "messages": [{"role": "user", "content": "介绍一下LLM"}],
    "stream": true
  }'
```

### 服务管理

```bash
aidllm status api      # 查看运行状态
aidllm stop api        # 停止服务
aidllm restart api     # 重启服务
```

## 模型服务开机自启动
系统已配置 `/etc/rc.local` 在开机时自动遍历执行 `/etc/aidlux/` 下的所有可执行脚本。

### 创建启动脚本（`/etc/aidlux/qwen-llm.sh`）

```bash
sudo tee /etc/aidlux/qwen-llm.sh << 'EOF'
#!/bin/bash
nohup aidllm start api -m Qwen3-4B-Instruct-cl4096 > /var/log/aidllm.log 2>&1 &
EOF
```

### 赋予执行权限

```bash
sudo chmod 775 /etc/aidlux/qwen-llm.sh
```

### 重启验证

```bash
sudo reboot
```
重启后，使用 `aidllm status api` 验证服务是否已自动启动。

---

## 注意事项

| 问题 | 说明 |
|------|------|
| Tokenizer | 模型包中不包含 tokenizer，需复用 Qwen3-4B (cl2048) 的 `qwen3-4b-tokenizer.json`（Qwen3 系列通用） |
| 路径 | AidGenSE 的 config JSON 中需使用**绝对路径** |
| prompt 模板 | Qwen3 模板已内置于 `api_cfg.json` 的 `prompt_template_list` 中 |
| 推理参数 | 可通过 `aidllm start api` 的参数调整设备类型 (`--device`) 等 |

### 推理性能参考

| 指标 | 值 |
|------|:----:|
| Init 耗时 | ~7.85s |
| 首 Token 延迟 (TTFT) | ~118.8ms |
| Prefill 速率 | ~328.3 tok/s |
| Decode 速率 | ~18.6 tok/s |

---