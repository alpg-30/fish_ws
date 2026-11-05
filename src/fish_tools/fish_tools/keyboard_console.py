"""TTY-only single-key console for ROS 1; pure key decisions stay ROS-free."""

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
        import rospy
        from fish_interfaces.msg import ExperimentState
        from std_srvs.srv import SetBool, Trigger

        self._rospy = rospy
        self._set_bool_type = SetBool
        self._trigger_type = Trigger
        self.latest = None
        self._last_printed = None
        self.subscription = rospy.Subscriber(
            "/fish/experiment_state", ExperimentState, self._on_state, queue_size=1)
        self.set_running_client = rospy.ServiceProxy(
            "/fish/experiment/set_running", SetBool)
        self.estop_client = rospy.ServiceProxy("/fish/controller/estop", Trigger)

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
            time.sleep(0.05)
        return self.latest is not None

    def _call(self, client, request, timeout_s, label):
        try:
            if not client.wait_for_service(timeout=min(1.0, timeout_s)):
                print("service unavailable: {0}".format(label), flush=True)
                return None
            return client(request)
        except self._rospy.ServiceException as exc:
            print("service call failed: {0}".format(exc), flush=True)
            return None

    def request_running(self, target):
        request = self._set_bool_type.Request()
        request.data = target
        response = self._call(
            self.set_running_client, request, 5.0, "/fish/experiment/set_running")
        if response is not None:
            print("set_running({0}): {1} {2}".format(
                target, response.success, response.message), flush=True)
        return response

    def request_estop(self):
        response = self._call(
            self.estop_client, self._trigger_type.Request(), 3.0,
            "/fish/controller/estop")
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
            time.sleep(0.05)
            if self.latest is not None and stop_wait_complete(
                self.latest.state, self.latest.transition_in_progress
            ):
                return self.latest.state in {"HOLDING", "IDLE"}
        print("bounded stop wait expired; controller is left running", flush=True)
        return False

    def close(self):
        try:
            self.subscription.unregister()
        except Exception:
            pass


def run_console():
    if not sys.stdin.isatty():
        print("keyboard_console requires an interactive TTY", file=sys.stderr)
        return 2

    import rospy

    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    keyboard = None
    exit_code = 0
    try:
        rospy.init_node("keyboard_console")
        keyboard = KeyboardNode()
        tty.setcbreak(fd)
        print("space=start/stop  e=estop  q=quit (short presses only)", flush=True)
        keyboard.spin_for_initial_state()
        gate = SpaceDebouncer()
        quitting = False
        last_busy_notice = 0.0
        while not quitting:
            ready, _, _ = select.select([fd], [], [], 0.05)
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
                    state.state, state.transition_in_progress, time.monotonic())
                if target is None:
                    now = time.monotonic()
                    if now - last_busy_notice >= 0.5:
                        print("key ignored (busy, debounce, or non-startable state)",
                              flush=True)
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
        try:
            rospy.signal_shutdown("keyboard console exit")
        except Exception:
            pass
    return exit_code


def main():
    return run_console()


if __name__ == "__main__":
    sys.exit(main())
