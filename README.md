# fish_ws — ROS 1 Noetic 四电机扑动控制

目标运行环境：**Jetson Orin Nano + Ubuntu 20.04 + ROS 1 Noetic**。
无电机 SDK 时可用 `backend=mock` 完整跑通波形、单键联动与录包；实机后端直接编译仓库内
DynamixelSDK 的 C++ 子集。

## 包

| 包 | 说明 |
|---|---|
| `fish_interfaces` | `FlappingCommand` / `MotorTelemetry` / `GaitState` / `ExperimentState`，以及 `SetGait` 服务 |
| `fish_control` | Fluent 非对称相位、参数整定、安全状态机、mock/实机总线、controller 节点 |
| `fish_tools` | `experiment_manager.py`（先录后动/停稳保持后关包）、`keyboard_console.py`（空格）、`bag_audit.py` |
| `fish_bringup` | launch 与 mock/hardware/录包配置 |

## 构建与测试

```bash
cd 03_hardware/fish_ws
catkin_make
source devel/setup.bash
```

- Python 测试：`catkin_make run_tests`
- C++ 纯逻辑测试：`cd build/fish_control && ctest --output-on-failure`

当前实现未在装好 Noetic 的机器上执行过上述命令，请以实际构建输出为准。

## 运行

mock（默认）：

```bash
cd 03_hardware/fish_ws
source devel/setup.bash
roslaunch fish_bringup fish.launch
```

实机（先完成标定并确认 `calibration/confirmed: true`）：

```bash
roslaunch fish_bringup fish.launch \
  controller_config:=$(rospack find fish_bringup)/config/hardware.yaml
```

实机后端面向 **DYNAMIXEL XW430-T200**（IP68、RS485、Protocol 2.0，位置控制模式），
使用同步读 `126..146` / `64..70` 与同步写 `116`。未找到 SDK 时该后端会显式报错，不会回退 mock。
构建时可用 `-DFISH_DYNAMIXEL_SDK_DIR=<其它路径>` 指向外部 SDK。
仓库只跟踪 DynamixelSDK 的 `c++/include`、`c++/src`、`LICENSE` 与 `SDK_REVISION`
（Apache-2.0）；`COLCON_IGNORE`/`CATKIN_IGNORE` 让构建系统跳过 SDK 内的 ROS 包，
其它语言绑定、示例与构建目录不纳入 Git。
串口设备名与权限只在 SoC 上配置，例如 U2D2 常见 `/dev/ttyUSB0`，CDC-ACM 适配器可能是 `/dev/ttyACM0`。

实机前提：

- 舵机设为 Position Control Mode，ID 与 `motors/ids` 一致，`bus/baudrate` 与 EEPROM 设置一致（模板用 1 Mbps）。
- 填写 `zero_raw`、`direction`、`external_ratio`、`mapping/*` 与 `limits/*` 后再把 `calibration/confirmed` 设为 `true`；否则 arm 会被拒绝。
- 正常停止只保持位置、不卸载扭矩；关闭 controller 进程也不会自动卸扭矩，断电或显式 disarm 才会释放。

另开一个终端运行键盘（需要交互式 TTY）：

```bash
cd 03_hardware/fish_ws
source devel/setup.bash
rosrun fish_tools keyboard_console.py
```

- 空格：`IDLE/HOLDING` 开始（先启动录包，再 arm/start）；`RUNNING` 停止（先 `BRAKING` 减速、确认 `HOLDING`、录尾部数据，再关包）。
- `e`：软件急停；`q`/EOF/Ctrl+C：正常停止后退出键盘，controller 继续保持位置与扭矩。

运行时调整动作参数（仅 `DISARMED`/`HOLDING`，整组原子校验，下一次 start 生效）：

```bash
rosservice call /fish/controller/set_gait \
  "{amplitude_model: 0.08, frequency_hz: 1.0, tf_coeff: 0.65, body_coeff: 0.5}"
```

## 录包审计

每次启动生成 `bags/<run_id>/{session.json,bag.bag,recorder.log}`：

```bash
rosrun fish_tools bag_audit.py bags/<run_id>
```

返回码 0 表示 metadata 可读、六个 topic 齐全、四电机均有有效角度/电流样本且 `stop_hold_confirmed`。

## 波形参数语义

- `gait/tf_coeff` 是**下扑（top→bottom）占周期比例**，上抬占 `1 - tf_coeff`。
- `mock.yaml` 默认 `0.65`：下扑做功慢、上抬回位快。
- Fluent 原始默认 `0.35`（`test_plot/output/data1_baseline.svg`）：下扑快、上抬慢，仅作对照。

## 波形模式

| `gait/waveform` | 行为 |
|---|---|
| `fluent_asymmetric_delayed`（默认） | 空间相位以时间延迟实现；快慢段切换点落在**每个电机自己的波峰/波谷**，每台电机下扑占比都等于 `tf_coeff` |
| `fluent_asymmetric` | Fluent UDF 原始全局相位，可复现仿真；`x≠0` 的安装点换向点会落在行程中段 |

两种模式都在 `GaitState`、`FlappingCommand` 和录包中记录实际生效的 `tf_coeff` 与 `waveform`。

## 当前边界

- mock 角度/电流是模拟值，`backend=mock`；不能当作实机实测数据。
- Fluent 时间相位与空间模型波形可独立核对；几何到关节角的实机映射未标定，`mapping/kind` 仅支持 `mock_linear`（mock）或 `linear_calibrated`（实机）。
- 实机后端已接入 DynamixelSDK，但四电机 ID/方位、机械零位、限位与安全阈值仍需按实机标定；未标定时 `calibration/confirmed` 保持 `false`。
