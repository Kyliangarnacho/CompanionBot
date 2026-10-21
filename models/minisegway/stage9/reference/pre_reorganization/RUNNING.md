# Ubuntu / ROS 2 运行

主开发环境：WSL2 Ubuntu 24.04、系统 Python 3.12、ROS 2 Jazzy。Windows 的 `.venv` 不能复制到这里。当前环境已经建立；日常使用从下面开始。

## 日常启动

```bash
cd ~/projects/CompanionBot
source scripts/env.sh
python -c 'import sys; print(sys.executable); print(sys.version)'
python -m pip --version
python -m pip check
```

解释器应为本仓库 `.venv/bin/python`。`env.sh` 加载 Jazzy、项目 venv 和已构建 overlay，配置本地模型/绘图/日志缓存及可选音频库。默认 DDS discovery 为 localhost；跨机器发现应在下一阶段显式配置，不能据此假设两个机器的 monotonic clock 相同。

```bash
bash scripts/build_ros.sh
source scripts/env.sh  # 构建后重新加载 overlay
```

构建产物在 `ros2_ws/build`、`install`、`log`，实验/诊断数据在本阶段 `results`。

## 唯一正式交互入口

终端交互，无设备、无 API、执行 backend 明确为 Fake：

```bash
python scripts/demo_stage8_interaction.py \
  --no-camera --fake --headless --no-mic --no-tts --ros \
  --output-dir models/minisegway/stage9/results/interaction
```

输入“你好小柒”唤醒；输入 `/status`、`/cancel`、`/quit`。无相机时 Master 是 UNAVAILABLE，FOLLOW 会被原 Supervisor 拒绝。`--fake` 是离线语言模型选项；当前 RobotBackend / NavigationBackend 在所有模式中仍然是 Fake。

ROS launch 调用的也是这个文件、同一 Session / Supervisor / Camera owner：

```bash
ros2 launch companionbot_ros interaction.launch.py
```

默认 `fake/headless/no_camera/no_mic/no_tts:=true`，适合当前 WSL2。launch 的 stdin 不作为交互输入；使用原 GUI（用户人工操作）或有 source/freshness 契约的 `interaction/input` service。直接运行前一个 Python 命令可用终端输入。有限诊断可加 `exit_after:=10.0`。

回放仍走正式入口，保持真实 Camera / detector / tracker / ReID / depth worker 的代码路径；回放不是硬件验证：

```bash
python scripts/demo_stage8_interaction.py \
  --video /absolute/path/to/1280x720.avi --fake --headless --no-mic --no-tts --ros \
  --exit-after 20 --output-dir models/minisegway/stage9/results/replay
```

必须保留继承标定的 1280×720 profile，不自动缩放 K。正式入口不会自动选择 Master。真人选框由用户在 GUI 中进行；`smoke_stage9_perception_ros.py` 的 track 1 选择只用于明确标注的 fixture 验收。

## 新环境重建

先检查 `/etc/os-release`、`/opt/ros/jazzy/setup.bash`、`dpkg-query` 与 `command -v colcon rosdep cmake`。本机已有 Jazzy desktop/base 与开发工具，没有重复安装 ROS。若新系统缺少 ROS，按 [Jazzy 官方 Ubuntu deb 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)配置官方源；运行本项目需要 `ros-jazzy-ros-base`、`ros-dev-tools`、Python 3.12 venv、CMake/C++ 编译器和 rosidl generators。无需安装 Nav2/SLAM。

只在新环境没有 `.venv` 时创建：

```bash
cd ~/projects/CompanionBot
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
.venv/bin/python -c 'import sys; print(sys.executable); print(sys.version)'
.venv/bin/python -m pip --version
.venv/bin/python -m pip install torch==2.14.1+cpu torchvision==0.29.1+cpu \
  --index-url https://download.pytorch.org/whl/cpu
.venv/bin/python -m pip install -r requirements-linux.txt
.venv/bin/python -m pip install --no-build-isolation -e .
.venv/bin/python scripts/configure_linux_venv.py
source scripts/env.sh
python -m pip check
```

使用 Jazzy 二进制对应的系统 Python，见 [ROS 的 Python package / venv 说明](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html)。`--system-site-packages` 复用 apt 的 ROS/colcon，而项目数值、视觉与 Agent 包装在 `.venv`。setuptools 满足当前 PyTorch 的 `>=77.0.3` 要求；本机 colcon 对旧 `tests_require` 的推断不适用，`env.sh` 加载 `ros2_ws/colcon.defaults.yaml`，显式选择支持的 pytest runner。empy 为 3.3.4。`configure_linux_venv.py` 仅在 venv 写一行 namespace hook，防止 Ubuntu 旧 Matplotlib `.pth` 隐藏 pip 的 matching `mpl_toolkits`；不改系统 Python。

通常的 Linux 音频系统包是 `libportaudio2 libasound2-plugins espeak-ng pulseaudio-utils`。本机 sudo 需要密码，实际采用官方 Ubuntu deb 的项目内解包方案，未改系统包/设备设置：

```bash
bash scripts/install_linux_audio.sh  # 当前 Ubuntu amd64 的本地 fallback
source scripts/env.sh
```

WSLg 有 PulseServer 且没有硬件 PCM 节点时，项目 ALSA 配置使用 Pulse plugin；TTS 采用 eSpeak NG 合成 + paplay。普通 Linux 无 PulseServer 时使用 eSpeak 原生输出。Pi 应优先安装发行版对应架构的系统包，不能把本机 amd64 `.venv/native` 搬过去。

rosdep metadata 可在项目内初始化，无需 `sudo rosdep init`：

```bash
bash scripts/init_local_rosdep.sh
source scripts/env.sh
python /usr/bin/rosdep check --from-paths ros2_ws/src --ignore-src
```

本机已完成；缓存与官方 sources.list 位于 `.venv`。实际 pip 版本快照见 `results/python_freeze_delivery.txt`，系统与关键包版本见 `results/environment.json`；不是 Windows 依赖锁的等价副本。

## 既有模型资产

当前已恢复原来的模型，不引入新感知/ASR 模型。新环境可显式安装：

```bash
python scripts/install_stage7_5_osnet.py
python -c 'from perception.person_reid import download_default_osnet_weights; print(download_default_osnet_weights())'
python scripts/restore_stage9_assets.py
python -c 'from ultralytics import YOLO; YOLO("yolo26n.pt"); YOLO(".venv/models/yolo26n-depth.pt")'
```

OSNet source/weights 和 SenseVoice/tokens/Silero/test wav 按原 revision / SHA256 校验。恢复脚本依赖复制过来的 Stage 8 speech asset manifest；继承的 C920 标定 NPZ 和本地 CAD mesh 也需保留，不能凭空重新生成。下载只在显式安装时发生。模型、第三方 upstream 与本地 CAD 仍在 ignored 缓存，原 provenance/license 不变。

## 真实设备

本机无 `/dev/video*` 或 USB bus，当前不能复验 C920。若日后需要 WSL USB，先由用户按 [Microsoft USB/IP 说明](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)连接，再核对 Linux UVC 驱动、V4L2 节点、权限和真实 1280×720 帧；USB attach 不保证摄像头可用。本阶段没有执行 Windows 端设备绑定/重连或桌面操作。

普通 Linux / Pi 实机使用稳定的 `/dev/v4l/by-id/...` 路径，确认对应标定设备；正式入口例如：

```bash
python scripts/demo_stage8_interaction.py \
  --camera-device /dev/v4l/by-id/ACTUAL_CAMERA --camera-backend v4l2 \
  --audio-device ACTUAL_INPUT --output-dir models/minisegway/stage9/results/hardware
```

这是设备就绪后由用户运行的 GUI/真人验收命令，本阶段未运行。云端模型沿用原 `.env` / AgentConfig 配置，未读取密钥或发起真实 Qwen 请求；不加 `--fake` 仍不代表已有实体运动。

Linux 默认只选配置好的输入，不偷偷退回 C920 microphone 或 output monitor。设备索引/名称、采样率应实际检查：

```bash
python -c 'import sounddevice as sd; print(sd.query_devices()); print(sd.default.device)'
python scripts/demo_stage8_interaction.py --no-camera --diagnose-audio \
  --output-dir models/minisegway/stage9/results/audio
```

本机 WSLg `default`/ALSA 成功采集 16 kHz mono PCM；它是 RDPSource 音频桥，尚未确认物理麦克风身份。PCM 非零与 TTS 进程完成不是真人 ASR、可听起声、AEC 或可靠低延迟认证。

## 自动验证与仿真诊断

```bash
python -m pytest -q -p no:cacheprovider tests ros2_ws/src/companionbot_ros/test
python -m colcon --log-base ros2_ws/log test --build-base ros2_ws/build --install-base ros2_ws/install
python -m colcon --log-base ros2_ws/log test-result --test-result-base ros2_ws/build --verbose
python scripts/smoke_test_minisegway.py --output models/minisegway/stage9/results/plant_smoke.json
python scripts/verify_stage9_control.py
python scripts/verify_stage9_control.py --extended-baseline
python scripts/verify_stage9_dynamic.py
python scripts/verify_stage9_models.py
python scripts/smoke_stage9_perception_ros.py
python scripts/smoke_stage9_ros.py
python scripts/verify_stage9_audio.py
```

ROS smokes 分别使用 localhost domain 88/87；勿在同一 domain 启动其他 robot。控制诊断只重用冻结场景，最新动态验收同原 Windows 5/20 窗口语义比较。运行结果写 Stage 9；重新运行会覆盖同名本阶段结果，保留需要对比的旧批次后再运行。

单独查看冻结控制链的 ROS 遥测：

```bash
ros2 launch companionbot_ros simulation.launch.py duration_s:=8.0
ros2 topic echo /companionbot/sim/control
```

这是有限仿真诊断，不是第二套正式 Demo，不接收 ROS 轮端控制命令。`/clock` 表示本次 MuJoCo sim clock；camera receipt、host monotonic、ROS publication 时间仍分开。TF 默认不发布；只有显式 `provisional_camera_tf:=true` 才发布未实测的 `base_link → camera_optical_frame`，绝不发布 `map/odom` 定位。详细契约见 [INTERFACES.md](INTERFACES.md)。
