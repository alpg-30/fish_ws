# fish_ws — mock-first ROS 2 四电机扑动控制

Task003 实施工程（无电机 SDK、无实机）。当前只支持 `backend=mock`；请求 `dynamixel` 会明确失败，不会回退 mock。

## 包

| 包 | 说明 |
|---|---|
| `fish_interfaces` | `FlappingCommand` / `MotorTelemetry` / `GaitState` / `ExperimentState` |
| `fish_control` | Fluent 非对称相位、参数事务、安全状态机、mock 总线、controller 节点 |
| `fish_tools` | `experiment_manager`（先录后动/停稳保持后关包）、`keyboard_console`（空格）、`bag_audit` |
| `fish_bringup` | launch 与 mock/录包/QoS 配置 |

## 构建与测试

```bash
cd 03_hardware/fish_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select fish_control
colcon test-result --verbose
```

当前实现未在含 ROS 2 Jazzy 的机器上执行过上述命令，请以实际构建输出为准。

## 运行

```bash
cd 03_hardware/fish_ws
source install/setup.bash
ros2 launch fish_bringup fish.launch.py
```

另开一个终端运行键盘（需要交互式 TTY）：

```bash
cd 03_hardware/fish_ws
source install/setup.bash
ros2 run fish_tools keyboard_console
```

- 空格：`IDLE/HOLDING` 开始（先启动录包，再 arm/start）；`RUNNING` 停止（先 `BRAKING` 减速、确认 `HOLDING`、录尾部数据，再关包）。
- `e`：软件急停；`q`/EOF/Ctrl+C：正常停止后退出键盘，controller 继续保持位置与扭矩。

## 录包审计

每次启动生成 `bags/<run_id>/{session.json,bag/,recorder.log}`：

```bash
ros2 run fish_tools bag_audit bags/<run_id>
```

返回码 0 表示 metadata 可读、七个 topic 齐全、四电机均有有效角度/电流样本且 `stop_hold_confirmed`。

## 波形参数语义

- `gait.tf_coeff` 是**下扑（top→bottom）占周期比例**，上抬占 `1 - tf_coeff`。
- `mock.yaml` 默认 `0.65`：下扑做功慢、上抬回位快。
- Fluent 原始默认 `0.35`（`test_plot/output/data1_baseline.svg`）：下扑快、上抬慢，仅作对照。

## 波形模式

| `gait.waveform` | 行为 |
|---|---|
| `fluent_asymmetric_delayed`（默认） | 空间相位以时间延迟实现；快慢段切换点落在**每个电机自己的波峰/波谷**，每台电机下扑占比都等于 `tf_coeff` |
| `fluent_asymmetric` | Fluent UDF 原始全局相位，可复现仿真；`x≠0` 的安装点换向点会落在行程中段 |

两种模式都在 `GaitState`、`FlappingCommand` 和 bag 中记录实际生效的 `tf_coeff` 与 `waveform`。

## 当前边界

- mock 角度/电流是模拟值，`backend=mock`；不能当作实机实测数据。
- Fluent 时间相位与空间模型波形可独立核对；几何到关节角的实机映射未标定，`mapping.kind` 仅支持 `mock_linear`。
- SDK、总线参数、四电机 ID/方位、机械零位与安全阈值待用户提供后再接实机。
