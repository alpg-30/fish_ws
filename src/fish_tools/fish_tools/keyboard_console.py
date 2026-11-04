"""TTY-only single-key console; pure key decisions remain ROS-independent."""

import os
import select
import sys
import termios
import time
import tty


START_STATES = {"IDLE", "HOLDING"}
STOP_STATES = {"RUNNING"}
BUSY_STATES = {"STARTING", "STOPPING"}


def target_for_state(state):
    """Derive a request only from the authoritative ExperimentState."""
    if state in START_STATES:
        return True
    if state in STOP_STATES:
        return False
    return None


class SpaceDebouncer:
    def __init__(self, debounce_s=0.35):
        self.debounce_s = float(debounce_s)
        self.last_accepted = None

    def accept(self, state, transition_in_progress, now):
        if transition_in_progress or state in BUSY_STATES:
            return None
        target = target_for_state(state)
        if target is None:
            return None
        if self.last_accepted is not None and now - self.last_accepted < self.debounce_s:
            return None
        self.last_accepted = now
        return target


def stop_wait_complete(state, transition_in_progress):
    return not transition_in_progress and state in {"IDLE", "HOLDING", "ERROR"}


class KeyboardNode:
    def __init__(self):
        import rclpy
        from fish_interfaces.msg import ExperimentState
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_srvs.srv import SetBool, Trigger

        self._rclpy = rclpy
        self._set_bool_type = SetBool
        self._trigger_type = Trigger
        self.node = rclpy.create_node("keyboard_console", namespace="fish")
        self.latest = None
        self._last_printed = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.node.create_subscription(
            ExperimentState, "/fish/experiment_state", self._on_state, qos
        )
        self.set_running_client = self.node.create_client(
            SetBool, "/fish/experiment/set_running"
        )
        self.estop_client = self.node.create_client(Trigger, "/fish/controller/estop")

    def _on_state(self, message):
        self.latest = message
        summary = (
            message.state,
            message.run_id,
            message.controller_state,
            message.recorder_ready,
            message.last_error,
        )
        if summary != self._last_printed:
            self._last_printed = summary
            print(
                "state={0} controller={1} recorder_ready={2} run={3} {4}".format(
                    message.state,
                    message.controller_state or "UNKNOWN",
                    message.recorder_ready,
                    message.run_id or "-",
                    message.last_error,
                ),
                flush=True,
            )

    def spin_for_initial_state(self, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while self.latest is None and time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.latest is not None

    def _call(self, client, request, timeout_s):
        if not client.wait_for_service(timeout_sec=1.0):
            print("service unavailable: {0}".format(client.srv_name), flush=True)
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        if not future.done():
            future.cancel()
            print("service timeout: {0}".format(client.srv_name), flush=True)
            return None
        return future.result()

    def request_running(self, target):
        request = self._set_bool_type.Request()
        request.data = target
        response = self._call(self.set_running_client, request, 5.0)
        if response is not None:
            print("set_running({0}): {1} {2}".format(target, response.success, response.message), flush=True)
        return response

    def request_estop(self):
        response = self._call(self.estop_client, self._trigger_type.Request(), 3.0)
        if response is not None:
            print("estop: {0} {1}".format(response.success, response.message), flush=True)
        return response

    def normal_stop_before_exit(self, timeout_s=30.0):
        if self.latest is None or self.latest.state != "RUNNING":
            return True
        response = self.request_running(False)
        if response is None or not response.success:
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest is not None and stop_wait_complete(
                self.latest.state, self.latest.transition_in_progress
            ):
                return self.latest.state in {"HOLDING", "IDLE"}
        print("bounded stop wait expired; controller is left running", flush=True)
        return False

    def close(self):
        self.node.destroy_node()


def run_console():
    if not sys.stdin.isatty():
        print("keyboard_console requires an interactive TTY", file=sys.stderr)
        return 2

    import rclpy

    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    keyboard = None
    ros_initialized = False
    exit_code = 0
    try:
        rclpy.init()
        ros_initialized = True
        keyboard = KeyboardNode()
        tty.setcbreak(fd)
        print("space=start/stop  e=estop  q=quit (short presses only)", flush=True)
        keyboard.spin_for_initial_state()
        gate = SpaceDebouncer()
        quitting = False
        last_busy_notice = 0.0
        while not quitting:
            ready, _, _ = select.select([fd], [], [], 0.05)
            rclpy.spin_once(keyboard.node, timeout_sec=0.0)
            if not ready:
                continue
            data = os.read(fd, 32)
            if not data:
                quitting = True
                break
            for value in data:
                char = chr(value)
                if char in ("q", "\x04", "\x03"):
                    quitting = True
                    break
                if char == "e":
                    keyboard.request_estop()
                    continue
                if char != " ":
                    continue
                state = keyboard.latest
                if state is None:
                    print("waiting for ExperimentState; key ignored", flush=True)
                    continue
                target = gate.accept(
                    state.state, state.transition_in_progress, time.monotonic()
                )
                if target is None:
                    now = time.monotonic()
                    if now - last_busy_notice >= 0.5:
                        print("key ignored (busy, debounce, or non-startable state)", flush=True)
                        last_busy_notice = now
                    continue
                keyboard.request_running(target)
        if not keyboard.normal_stop_before_exit():
            exit_code = 1
    except KeyboardInterrupt:
        if keyboard is not None and not keyboard.normal_stop_before_exit():
            exit_code = 1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        if keyboard is not None:
            keyboard.close()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()
    return exit_code


def main():
    return run_console()


if __name__ == "__main__":
    sys.exit(main())
