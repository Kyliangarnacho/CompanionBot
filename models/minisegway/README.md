# MiniSegway MuJoCo physics plant

`mini_segway.xml` 是 passive nominal plant；`mini_segway_moving_payload.xml`
增加了简化载物篮和自由刚体 payload。模型使用 CAD-derived visual mesh、
解析 collision proxy、显式 aggregate inertia 和两个自由 wheel hinge。
Actuator 是 `gear=1` 的直接 torque input，并保留 peak/stall hard limit；
模型本身不包含 controller，balance mode 也不使用 ball caster。

Artifact 按阶段分层：`stage1/results/` 保存 plant/PID/LQR 基线，`stage2/results/`
保存 full-state ID 与冻结 moving-payload acceptance，`stage3/config/` 和
`stage3/results/` 保存最终 commanded-motion/yaw/payload baseline。跨阶段仍直接使用的
MJCF、plant、sensor、estimator 与 Q 配置留在本目录，避免复制出多份真值。

## 原始传感器坐标约定

两份模型中的 `imu_site` 都刚性附着于 `chassis`，局部位置为
`[0, 0, 0.020] m`，局部姿态为单位四元数 `[1, 0, 0, 0]`，因此 IMU
坐标系与 chassis frame 完全对齐。该 site 只用于定义传感器坐标系和 viewer
中的小尺寸安装标记，不增加质量或惯量。

- chassis X：左右轮轴方向；pitch 绕 +X 旋转。
- chassis Y：前后方向；当前 longitudinal forward 是 -Y。
- chassis Z：竖直方向。
- `imu_accelerometer`：MuJoCo 原生三轴 accelerometer，输出包含重力的 local-frame
  proper acceleration，单位 m/s^2。
- `imu_gyro`：MuJoCo 原生三轴 gyro，输出 local-frame angular velocity，单位 rad/s。
- `left_wheel_angle` / `right_wheel_angle`：对应两侧 hinge 的原始 joint angle，
  单位 rad；当前不包含 CPR、计数量化或速度估计。

确定性 raw-sensor bring-up：

```powershell
.venv\Scripts\python.exe scripts\check_raw_sensors.py
```

## Virtual IMU hardware V1

`imu_hardware_config.json` 只定义一个 profile：RotorS `gazebo_imu_plugin.h` 的
ADIS16448 defaults。Sensor clock 与 physics tick 同为 1 kHz，packet 固定延迟 1 ms，
500 Hz estimator 每次只读取 `available_time <= current_time` 的最新 packet。RNG 明确
固定为 NumPy PCG64、默认 seed 为 `16448`。生产接口
`MiniSegwaySim.imu_measurement()` 返回 noisy raw accelerometer/gyro；
`ideal_imu_measurement()` 只供原始传感器 oracle 与诊断使用。每个 hardware tick 的
`imu_raw_log_fields()` 同时保留明确命名的 ideal/noisy 字段，estimator 只拿到 noisy tuple。

每轴使用独立标准正态抽样；sensor/system reset 时 drift 清零、turn-on bias 每轴只采样
一次。每个 sensor tick 生成并缓存一次 packet，重复读取不更新 bias/RNG。相同 seed 会
复现完整 reset/sample 序列；下式的 `dt` 是 1 ms sensor period，不是 2 ms estimator period：

```text
sigma_white = noise_density / sqrt(dt)
phi = exp(-dt/tau)
sigma_bias = sqrt(-random_walk^2*tau/2 * (exp(-2*dt/tau)-1))
bias[k+1] = phi*bias[k] + sigma_bias*N(0,1)
measurement = ideal + turn_on_bias + bias[k+1] + sigma_white*N(0,1)
```

Startup 使用 0.5 s、500 个 noisy raw gyro packet 的 held-stationary mean 做 zero-rate
bias calibration，不读取 hidden bias 或 MuJoCo truth；accelerometer bias 不校准。
Gyro 在 ±17.45329252 rad/s、accelerometer 在 ±176.52 m/s² 做 per-axis clipping 并记录
saturation flag。Packet stale threshold 是 5 ms；invalid/stale 时复用上一有效 measurement。
这是 virtual sensor/hardware layer，不属于 complementary filter。V1 仍不包含量化、
temperature、vibration、dropout、CRC/SPI failure 或 encoder fault injection。

最终 timestamp-aligned 链路的冻结 2 Hz single-Q moving-payload collision 结果保存在
`stage2/results/moving_payload_timestamp_aligned_collision_results.json`；详细历史逐点日志已在阶段收口时
删除，关键指标固化在根目录 `LEARNING_LOG.md` 与 `CURRENT_STATE.md`。

## Virtual quadrature encoder

`encoder_profile.json` 集中保存未来实体 encoder/gearbox 的 nominal 参数。当前 profile
采用 motor-shaft x4 decoded `48 counts/rev`、精确 gearbox ratio `46.8512`，因此
output-shaft resolution 为 `2248.8576 counts/rev`。左右轮使用同一 profile。

`MiniSegwaySim` 在 reset 时把原始 `jointpos` 读数捕获为零点，随后按下式输出累计
整数 count：

```text
scaled_count = (raw_joint_angle - reset_angle) * count_sign * 2248.8576 / (2*pi)
encoder_count = round_to_nearest_away_from_zero(scaled_count)
```

当前左右 `count_sign` 都是 `+1`：两轮正 count 对应绕 chassis `+X` 的正转，纯滚动
时对应 chassis 沿 `-Y` 前进。该符号是 CompanionBot 的软件归一化约定；未来实机若
因左右电机镜像安装或 A/B 接线导致 MCU 原始方向不同，应在 profile 中分别校正符号。
本层不模拟 A/B 电气方波、噪声、丢边、timer overflow，也不提供 wheel-speed 或状态估计。

确定性 virtual-encoder bring-up：

```powershell
.venv\Scripts\python.exe scripts\check_virtual_encoders.py
```

## Encoder displacement geometry

MuJoCo wheel hinge 的 `jointpos`、以及由它量化得到的 encoder count，测量的是 wheel
child body 相对 chassis parent body 的转角，不是轮子相对世界的总转角。对当前两侧
`count_sign = +1` 的 profile：

```text
delta_phi_i = 2*pi*delta_count_i / (count_sign_i*counts_per_output_rev)
delta_psi_world_i = delta_phi_i + delta_theta_chassis
delta_p_forward = wheel_radius * ((delta_phi_left + delta_phi_right)/2
                                  + delta_theta_chassis)
```

这里 `delta_theta_chassis` 是实际 chassis pitch 的变化，不是 `pitch_error` 或
`theta_eq`；`delta_p_forward` 为沿 `-Y` 的正向位移。加号来自 chassis pitch 与
relative hinge rotation 都绕同一个 chassis/world `+X` 轴：wheel world rotation
等于 parent rotation 加 child-relative rotation。

`scripts/check_encoder_displacement_geometry.py` 的确定性结果如下，误差均为
`estimate - GT`：

| Case | GT displacement | Naive error `r*mean(delta_phi)` | Corrected error |
|---|---:|---:|---:|
| 固定 axle、chassis `+5 deg`、wheel relative `-5 deg` | 0 mm | -3.637717 mm | +0.027475 mm |
| 固定 pitch、纯滚动前进 100 mm | 100 mm | -0.021458 mm | -0.021458 mm |
| Frozen LQR、初始 `+5 deg` error、1 s | +8.943057 mm | +10.477657 mm | +6.689588 mm |

前两个受控运动学 case 的 corrected 误差都在半个 encoder count 对应的
`0.058673 mm` 内。LQR case 中 raw-angle oracle 的 corrected error 仍有
`+6.682021 mm`，说明其主要残差来自该动态接触过程中的 wheel-ground slip，而不是
count quantization；pitch correction 仍把 naive error 降低了约 36%。本结论只定义
pure-rolling measurement geometry，不提供速度估计、滤波、状态积分或 controller 输入。

```powershell
.venv\Scripts\python.exe scripts\check_encoder_displacement_geometry.py
```

## Passive longitudinal estimator baseline

`longitudinal_estimator_config.json` 集中定义 2 ms sample period、IMU complementary
filter 的 `tau = 0.5 s`（`fc = 1/(2*pi*tau) = 0.31831 Hz`）和 encoder PLL bandwidth。
每次 reset 先用
`theta_acc = atan2(accel_y, accel_z)` 初始化物理 chassis pitch，随后：

```text
alpha = tau / (tau + dt)
theta_gyro = theta_hat_previous + gyro_x*dt
theta_hat = theta_gyro + (1-alpha)*wrap(theta_acc - theta_gyro)
theta_dot_hat = gyro_x
```

`theta_hat` 不减 `theta_eq`；后者仍只属于 controller reference。里程计只读 integer
counts 与 IMU estimate：

```text
phi_hat_i = 2*pi*c_hat_i / (count_sign_i*2248.8576)
omega_i_pll = 2*pi*c_dot_hat_i / (count_sign_i*2248.8576)
p_hat += wheel_radius*(mean(delta_phi_hat_left, delta_phi_hat_right)
                      + delta_theta_hat)
v_hat = wheel_radius*(mean(omega_left_pll, omega_right_pll) + theta_dot_hat)
```

每个轮子独立运行一个 linear incremental-count tracking PLL。核心结构借鉴 ODrive 官方
`Encoder::update_pll_gains()` 与 `Encoder::update()`：先用上一周期速度预测 count position，
再用 `integer_count-floor(predicted_position)` 作为离散 phase error 校正 position/velocity：

```text
pll_kp = 2*bandwidth
pll_ki = 0.25*pll_kp^2
position += dt*velocity
error = integer_count - floor(position)
position += dt*pll_kp*error
velocity += dt*pll_ki*error
```

候选 40/80/120 rad/s 均满足 `dt*pll_kp < 1`；最终选择 80 rad/s（`kp=160 s^-1`、
`ki=6400 s^-2`、`dt*kp=0.32`）。沿用 ODrive near-zero snap，阈值为
`0.5*dt*ki = 6.4 counts/s`。只借鉴 predictor/corrector、gain structure、稳定性检查和
snap；没有移植 HAL、SPI、index、phase interpolation 或 commutation。上游：
[ODrive encoder.cpp](https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/encoder.cpp)。

80 rad/s 是已完成 bring-up 后冻结的 error/lag 折中。Encoder 主路径统一为：integer
count 只作为 PLL measurement；PLL 连续 `c_hat/c_dot_hat` 分别转换为
`phi_hat/omega_hat`，再生成 `p_hat/v_hat`。Reset 时 `p_hat=0`，后续累计左右
`delta_phi_hat` 均值与 `delta_theta_hat`；raw-count position 不再直接进入主路径。

## Full-sensorized frozen ID-LQR takeover

阶段验收曾保持 frozen nominal `K_id`、`theta_eq`、500 Hz controller、1 kHz physics、
每轮 ±0.63 N·m torque limit 和 plant 全部不变，对比：

```text
A controller state = [p_GT, v_GT, theta_GT-theta_eq, theta_dot_GT]
B controller state = [p_hat, v_hat, wrap(theta_hat-theta_eq), theta_dot_hat]
```

B 的 state assembly 只接受 `LongitudinalEstimate`，不读取 MuJoCo `qpos/qvel` 或
`longitudinal_state()`。GT state 在 controller command 组装完成后才进入 evaluator/logger。
Reset 时假定机器人在释放前静止持有：encoder odometry 从 `p_hat=0` 开始，IMU 用
`atan2(accel_y,accel_z)` 初始化实际 physical pitch；左右 PLL position 直接初始化为当前
integer count、velocity 初始化为 0，从下一 2 ms tick 起更新。

| Initial error | GT settle / RMS | PLL settle / RMS | PLL drift | PLL peak torque / saturation |
|---:|---:|---:|---:|---:|
| -5° | 0.154 s / 0.258° | 0.277 s / 0.291° | +6.32 mm | 0.378 N·m / 0% |
| -2° | 0.127 s / 0.104° | 0.289 s / 0.149° | +2.50 mm | 0.151 N·m / 0% |
| +2° | 0.127 s / 0.104° | 0.255 s / 0.118° | -2.73 mm | 0.151 N·m / 0% |
| +5° | 0.154 s / 0.258° | 0.289 s / 0.302° | -6.48 mm | 0.378 N·m / 0% |

四个 PLL sensorized case 均未翻倒、通过 settling gate，且确定性重复完全一致。相对 GT
的 PLL velocity error RMS 为 2.82–7.28 mm/s、peak 为 0.0316–0.0795 m/s。±5° 的旧
velocity path saturation 消失，峰值
回到与 GT 相同的 0.378 N·m。累计 `p_hat` error peak 仍为 2.44–6.35 mm，且真实最终
position drift 仍为 2.50–6.48 mm；velocity 动态改善后该残差继续支持 wheel-ground slip
是独立 odometry limitation。本轮没有修改 controller、plant 或 attitude estimator。

本节表格与误差数值记录的是统一 PLL position 主路径切换前的实验；该结构收口按要求没有
重跑 recovery/regression，因此这些数值尚未作为新 position 主路径的验收结果。

## Sensorized moving-payload collision chain

`scripts/run_moving_payload_collision_acceptance.py` 的当前在线数据流为：

```text
native IMU + integer encoder count
    -> longitudinal estimator x_hat[k]
    -> fixed nominal ID-LQR
    -> nominal prediction A*x_hat[k] + B*u[k]
    -> innovation x_hat[k+1] - prediction
    -> matched projection/bound -> one Q-filter -> bounded u_dr
```

LQR、matched-disturbance observation 和相关在线 regressor 只接收同一 estimator timeline 的
`x_hat`。MuJoCo longitudinal GT、payload pose/contact 和事后 instantaneous equilibrium
只用于 command 组装完成后的 evaluator/logger。旧 slow/fast、Q sweep 和未对齐状态的
逐点结果已经删除；关键比较表保存在 `LEARNING_LOG.md`。最终结果只保留
`stage2/results/moving_payload_timestamp_aligned_collision_results.json`。

第三方 CAD 派生 mesh 已被 Git 忽略。若本地仍保留 upstream STEP，可使用独立
CAD 环境重新生成 mesh、MJCF 和质量属性报告：

```powershell
D:\project\MiniSegway_assembly_work\.venv\Scripts\python.exe tools\build_minisegway_plant.py
```

加载/contact smoke test：

```powershell
.venv\Scripts\python.exe scripts\smoke_test_minisegway.py
```

厂家值、CAD 计算值和 provisional 值的 provenance 记录在 `plant_report.json`；
未来必须实测的假设集中在 `plant_parameters.json`。项目总览与控制入口见根目录
`README.md`，冻结指标与边界见 `CURRENT_STATE.md`。
