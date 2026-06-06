# Home Assistant on Docker 部署教程

> Ubuntu 22.04 (ARM64) + Docker + Home Assistant + HACS

---

## 环境信息

- **系统**: Ubuntu 22.04.5 LTS (Jammy)
- **架构**: ARM64 (aarch64)
- **Home Assistant 版本**: 2026.5 (stable)
- **HACS 版本**: 2026.5.4

---

## 一、安装 Docker

### 1.1 安装前置依赖

```bash
sudo apt-get update -qq
sudo apt-get install -y ca-certificates curl gnupg
```

### 1.2 添加国内镜像源

```bash
# 创建密钥目录
sudo install -m 0755 -d /etc/apt/keyrings

# 下载并添加 Docker GPG 密钥（阿里云源）
curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg -o /tmp/docker.gpg
sudo gpg --dearmor --batch --yes -o /etc/apt/keyrings/docker.gpg /tmp/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 添加 Docker apt 仓库（阿里云源）
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://mirrors.aliyun.com/docker-ce/linux/ubuntu jammy stable" | sudo tee /etc/apt/sources.list.d/docker.list
```

### 1.3 安装 Docker

```bash
sudo apt-get update -qq
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 1.4 配置镜像加速器

```bash
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<'EOF'
{
  "registry-mirrors": [
    "https://docker.m.daocloud.io",
    "https://dockercf.jmirror.top",
    "https://dockerproxy.com"
  ]
}
EOF
sudo systemctl restart docker
```

### 1.5 验证

```bash
sudo docker run --rm hello-world
```

---

## 二、部署 Home Assistant

### 2.1 创建数据目录

```bash
mkdir -p ~/homeassistant
```

### 2.2 启动容器

```bash
sudo docker run -d \
  --name homeassistant \
  --restart unless-stopped \
  --privileged \
  --network host \
  -v ~/homeassistant:/config \
  -v /run/dbus:/run/dbus:ro \
  -e TZ=Asia/Shanghai \
  homeassistant/home-assistant:stable
```

**参数说明**:

| 参数 | 说明 |
|------|------|
| `-d` | 后台运行 |
| `--name homeassistant` | 容器名称 |
| `--restart unless-stopped` | 开机自启，异常自动重启 |
| `--privileged` | 授予特权，支持访问硬件 |
| `--network host` | 宿主机网络，支持设备发现 |
| `-v ~/homeassistant:/config` | 配置文件持久化 |
| `-v /run/dbus:/run/dbus:ro` | 蓝牙等设备通信 |
| `-e TZ=Asia/Shanghai` | 时区 |

### 2.3 访问

浏览器打开 `http://<本机IP>:8123`，按向导完成初始化。

---

## 三、安装 HACS

### 3.1 下载并安装

```bash
sudo docker exec homeassistant bash -c "wget -O - https://get.hacs.xyz | bash -"
```

### 3.2 重启容器

```bash
sudo docker restart homeassistant
```

### 3.3 激活 HACS

1. 等待 1-2 分钟，刷新 Home Assistant 页面
2. 进入 **设置 → 设备与服务 → 添加集成**
3. 搜索 `HACS` 并添加
4. 按提示授权 GitHub 账号
5. 刷新页面后，左侧菜单栏出现 **HACS**

---

## 四、日常管理

### 容器操作

```bash
# 启动
sudo docker start homeassistant

# 停止
sudo docker stop homeassistant

# 重启
sudo docker restart homeassistant

# 查看状态
sudo docker ps --filter name=homeassistant

# 查看日志（实时）
sudo docker logs -f homeassistant

# 进入容器 Shell
sudo docker exec -it homeassistant bash
```

### 升级 Home Assistant

```bash
# 拉取最新镜像
sudo docker pull homeassistant/home-assistant:stable

# 重建容器（保留 /config 数据）
sudo docker stop homeassistant
sudo docker rm homeassistant
sudo docker run -d \
  --name homeassistant \
  --restart unless-stopped \
  --privileged \
  --network host \
  -v ~/homeassistant:/config \
  -v /run/dbus:/run/dbus:ro \
  -e TZ=Asia/Shanghai \
  homeassistant/home-assistant:stable
```

> 配置文件保存在 `~/homeassistant` 目录，不会因重建容器而丢失。

### 配置文件位置

```
~/homeassistant/
├── configuration.yaml      # 主配置文件
├── automations.yaml        # 自动化
├── scripts.yaml            # 脚本
├── scenes.yaml             # 场景
├── custom_components/      # 自定义组件（含 HACS）
├── hacs                    # HACS 下载的集成/卡片
├── .storage/               # 内部状态存储
└── home-assistant.log      # 运行日志
```

---

## 五、与宿主机通信

Home Assistant 使用 `--network host` 模式，容器内可通过 `127.0.0.1` 直接访问宿主机上运行的任何服务。

例如宿主机上 8888 端口有服务，在 HA 配置中直接使用：

```yaml
rest_command:
  local_service:
    url: "http://127.0.0.1:8888/api"
    method: POST
```

---

## 六、备份与恢复

### 备份

```bash
# 停止容器
sudo docker stop homeassistant

# 打包配置文件
tar -czf ha-backup-$(date +%Y%m%d).tar.gz ~/homeassistant

# 重启容器
sudo docker start homeassistant
```

### 恢复

```bash
# 停止容器
sudo docker stop homeassistant

# 恢复配置
tar -xzf ha-backup-YYYYMMDD.tar.gz -C /

# 重启容器
sudo docker start homeassistant
```

---

## 七、常见问题

### 镜像拉取失败

更换 `daemon.json` 中的镜像加速器地址，然后重启 Docker：

```bash
sudo systemctl restart docker
```

### 容器异常退出

查看日志定位问题：

```bash
sudo docker logs --tail 100 homeassistant
```

常见原因：`configuration.yaml` 语法错误 → 检查缩进和格式。

### 端口被占用

检查 8123 端口：

```bash
sudo lsof -i :8123
```

### 无法访问宿主机服务

确认容器使用了 `--network host`：

```bash
sudo docker inspect homeassistant | grep NetworkMode
```

输出应为 `"NetworkMode": "host"`。

---

