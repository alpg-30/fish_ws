#!/usr/bin/env bash
# 实机单终端运行脚本（Jetson Nano / ROS Melodic）
#
# 功能：
#   1. 自动拉起 roscore（已有 master 则复用）
#   2. roslaunch 启动 controller + experiment_manager
#   3. roslaunch 启动前开始录包，包含：
#        /fish/flapping_command  舵机摆角指令（ideal/applied/goal raw）
#        /fish/motor_telemetry   真实位置反馈 + 实时电流
#        /fish/joint_states      关节角信号
#        /fish/gait_state        当前配置
#        /fish/experiment_state  实验状态
#        /fish/diagnostics       诊断
#        /imu                    IMU（N100WP，100 Hz，带 header.stamp）
#   4. 前台 keyboard_console：空格 start/stop，e 急停，q 退出
#
# 用法：
#   ./src/fish_bringup/scripts/run_real.sh
#   CONFIG=/path/to/other.yaml ./src/fish_bringup/scripts/run_real.sh
#
# 输出：
#   bags/launch_<时间戳>/bag.bag
#   bags/launch_<时间戳>/{roscore,rosbag,roslaunch}.log

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
CONFIG="${CONFIG:-$WS/src/fish_bringup/config/hardware.yaml}"
BAG_ROOT="${BAG_ROOT:-$WS/bags}"
STAMP="$(date +%Y%m%dT%H%M%S)"
RUN_DIR="$BAG_ROOT/launch_$STAMP"
BAG="$RUN_DIR/bag.bag"

if [[ ! -f /opt/ros/melodic/setup.bash ]]; then
  echo "[run_real] 未找到 /opt/ros/melodic/setup.bash" >&2
  exit 1
fi
if [[ ! -f "$WS/devel/setup.bash" ]]; then
  echo "[run_real] 未找到 $WS/devel/setup.bash，请先 catkin_make" >&2
  exit 1
fi

set +u
source /opt/ros/melodic/setup.bash
source "$WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

CONFIG="$(readlink -f "$CONFIG")"
if [[ ! -f "$CONFIG" ]]; then
  echo "[run_real] 配置文件不存在: $CONFIG" >&2
  exit 1
fi

mkdir -p "$RUN_DIR"

ROSCORE_PID=""
BAG_PID=""
LAUNCH_PID=""
OWN_MASTER=0

cleanup() {
  trap - EXIT INT TERM
  # 先尽力停到 HOLDING 再关闭扭矩，避免退出后舵机仍保持使能。
  if rosservice list 2>/dev/null | grep -q "/fish/controller/stop_hold"; then
    timeout 5 rosservice call /fish/controller/stop_hold >/dev/null 2>&1 || true
    sleep 2
    timeout 5 rosservice call /fish/controller/disarm >/dev/null 2>&1 || true
    sleep 1
  fi
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "[run_real] 停止 rosbag ..."
    kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[run_real] 停止 roslaunch ..."
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ "$OWN_MASTER" == "1" && -n "$ROSCORE_PID" ]]; then
    echo "[run_real] 停止 roscore ..."
    kill -INT -- "-$ROSCORE_PID" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-$ROSCORE_PID" 2>/dev/null || true
  fi
  echo "[run_real] 录包结束: $BAG"
}
trap cleanup EXIT INT TERM

if rosservice list >/dev/null 2>&1; then
  echo "[run_real] 检测到已有 ROS master，直接复用"
else
  echo "[run_real] 启动 roscore ..."
  setsid roscore >"$RUN_DIR/roscore.log" 2>&1 &
  ROSCORE_PID=$!
  OWN_MASTER=1
  for _ in $(seq 1 100); do
    rosservice list >/dev/null 2>&1 && break
    sleep 0.1
  done
  if ! rosservice list >/dev/null 2>&1; then
    echo "[run_real] roscore 启动失败，见 $RUN_DIR/roscore.log" >&2
    exit 1
  fi
fi

echo "[run_real] 开始录包: $BAG"
rosbag record -O "$BAG" \
  /fish/flapping_command \
  /fish/motor_telemetry \
  /fish/joint_states \
  /fish/gait_state \
  /fish/experiment_state \
  /fish/diagnostics \
  /imu \
  >"$RUN_DIR/rosbag.log" 2>&1 &
BAG_PID=$!
sleep 0.5
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "[run_real] rosbag 启动失败，见 $RUN_DIR/rosbag.log" >&2
  exit 1
fi

echo "[run_real] 启动 controller + experiment_manager ..."
roslaunch fish_bringup fish.launch \
  controller_config:="$CONFIG" \
  bag_root:="$BAG_ROOT" \
  imu:=true \
  >"$RUN_DIR/roslaunch.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 150); do
  if rosservice list 2>/dev/null | grep -q "/fish/controller/arm" \
     && rosservice list 2>/dev/null | grep -q "/fish/experiment/set_running"; then
    break
  fi
  sleep 0.2
done
if ! rosservice list 2>/dev/null | grep -q "/fish/controller/arm"; then
  echo "[run_real] controller 未就绪，见 $RUN_DIR/roslaunch.log" >&2
  exit 1
fi

echo "[run_real] 就绪：空格 start/stop，e 急停，q 退出"
echo "[run_real] 配置: $CONFIG"
echo "[run_real] 日志: $RUN_DIR"
rosrun fish_tools keyboard_console.py
