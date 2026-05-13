## 文件结构

X1派的代码结构:

- [`ROS2_Packages`](ROS2_Packages)：ROS2底层功能包，完善中，包含机器人控制、传感器处理、导航等功能模块的ROS2包
- [`Vision`](Vision)：视觉控制部分的代码，用来实现体感交互的功能
- [`Voice_Assistant`](Voice_Assistant)：语音助手部分的代码，本地部署了ASR和TTS模型，用来实现语音交互的功能
- [`Aid_LLM`](Aid_LLM)：LLM部分，在NPU上部署Qwen3-4B实现本地推理

---

### 待补充...