"""Serial experiment coordinator with record-before-motion and hold-before-close."""

import json
import os
import queue
import shutil
import signal
import subprocess
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from .bag_audit import REQUIRED_TOPICS


STATES = {"IDLE", "STARTING", "RUNNING", "STOPPING", "HOLDING", "ERROR"}
FAULT_STATES = {"FAULT", "ESTOP_LATCHED"}
QOS_CONTRACT = {
    "/fish/flapping_command": "reliable/volatile/depth50",
    "/fish/motor_telemetry": "reliable/volatile/depth50",
    "/fish/joint_states": "reliable/volatile/depth10",
    "/fish/gait_state": "reliable/transient_local/depth1",
    "/fish/experiment_state": "reliable/transient_local/depth1",
    "/fish/diagnostics": "reliable/volatile/depth10",
    "/parameter_events": "reliable/volatile/depth1000",
}


def required_topic_entries():
    return [
        {"name": name, "type": type_name, "qos_profile": QOS_CONTRACT[name]}
        for name, type_name in REQUIRED_TOPICS.items()
    ]


def decide_set_running(state, transition_in_progress, target):
    """Return (accepted, duplicate, message) without mutating local state."""
    if transition_in_progress or state in {"STARTING", "STOPPING"}:
        if target and state == "STARTING":
            return True, True, "start already in progress"
        if not target and state == "STOPPING":
            return True, True, "stop already in progress"
        return False, False, "opposite request rejected during transition"
    if target:
        if state == "RUNNING":
            return True, True, "already running"
        if state in {"IDLE", "HOLDING", "ERROR"}:
            return True, False, "start queued"
    else:
        if state == "RUNNING":
            return True, False, "stop queued"
        if state in {"IDLE", "HOLDING", "ERROR"}:
            return True, True, "already stopped"
    return False, False, "request incompatible with state {0}".format(state)


def final_experiment_state(stop_hold_confirmed, bag_valid, forced_error):
    if forced_error or not stop_hold_confirmed:
        return "ERROR"
    return "HOLDING"


def make_run_id(now=None):
    now = now or datetime.now(timezone.utc)
    return now.strftime("%Y%m%dT%H%M%S.%fZ") + "-" + uuid.uuid4().hex[:10]


def make_recorder_argv(qos_override_file, bag_path, ros2_executable="ros2"):
    return [
        ros2_executable,
        "bag",
        "record",
        "-s",
        "sqlite3",
        "--disable-keyboard-controls",
        "--qos-profile-overrides-path",
        str(qos_override_file),
        "-o",
        str(bag_path),
        *REQUIRED_TOPICS.keys(),
    ]


def atomic_write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp-" + uuid.uuid4().hex)
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def new_session(run_id, backend, parameters, bag_relative_path, started_ros_time_ns,
                initial_active_config_revision):
    return {
        "schema_version": 1,
        "run_id": run_id,
        "backend": backend,
        "storage_id": "sqlite3",
        "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
        "software_revision": parameters["software_revision"],
        "fluent_reference_revision_or_digest": parameters["fluent_reference"],
        "started_ros_time_ns": int(started_ros_time_ns),
        "stopped_ros_time_ns": None,
        "status": "STARTING",
        "controller_state_at_end": "UNKNOWN",
        "stop_hold_confirmed": False,
        "accepted_parameters": {},
        "initial_active_config_revision": int(initial_active_config_revision),
        "required_topics": required_topic_entries(),
        "bag_relative_path": bag_relative_path,
        "recorder_exit_code": None,
        "validation": {
            "metadata_readable": False,
            "required_topics_present": False,
            "per_motor_valid_angle_samples": [0, 0, 0, 0],
            "per_motor_valid_current_samples": [0, 0, 0, 0],
            "sequence_gap_count": 0,
            "failures": [],
        },
    }


class ExperimentManager:
    def __init__(self):
        import rclpy
        from fish_interfaces.msg import ExperimentState, FlappingCommand, GaitState, MotorTelemetry
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_srvs.srv import SetBool, Trigger

        self._rclpy = rclpy
        self._messages = {
            "experiment": ExperimentState,
            "gait": GaitState,
            "command": FlappingCommand,
            "telemetry": MotorTelemetry,
        }
        self._trigger_type = Trigger
        self._get_parameters_type = GetParameters
        self._list_parameters_type = ListParameters
        self.node = rclpy.create_node("experiment_manager", namespace="fish")
        self.parameters = self._declare_parameters()
        self._lock = threading.RLock()
        self._queue = queue.Queue(maxsize=8)
        self._stop_event = threading.Event()
        self._shutdown_done = threading.Event()
        self._accepting = True
        self._failure_enqueued = False
        self._queued_target = None
        self._active = False
        self._session = None
        self._session_path = None
        self._run_dir = None
        self._recorder = None
        self._latest = {"gait": None, "command": None, "telemetry": None}
        self._received_at = {"gait": 0.0, "command": 0.0, "telemetry": 0.0}
        self._state = {
            "run_id": "",
            "state": "IDLE",
            "transition_in_progress": False,
            "recorder_ready": False,
            "bag_relative_path": "",
            "last_error": "",
        }

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        stream = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.node.create_publisher(
            ExperimentState, "/fish/experiment_state", latched
        )
        self.subscriptions = [
            self.node.create_subscription(GaitState, "/fish/gait_state", self._on_gait, latched),
            self.node.create_subscription(
                FlappingCommand, "/fish/flapping_command", self._on_command, stream
            ),
            self.node.create_subscription(
                MotorTelemetry, "/fish/motor_telemetry", self._on_telemetry, stream
            ),
        ]
        self.service = self.node.create_service(
            SetBool, "/fish/experiment/set_running", self._on_set_running
        )
        self.trigger_clients = {
            name: self.node.create_client(Trigger, "/fish/controller/" + name)
            for name in ("arm", "start", "stop_hold", "estop")
        }
        self.list_parameters_client = self.node.create_client(
            ListParameters, "/fish/controller/list_parameters"
        )
        self.get_parameters_client = self.node.create_client(
            GetParameters, "/fish/controller/get_parameters"
        )
        self.timer = self.node.create_timer(0.5, self._heartbeat)
        self._publish_state()
        self.worker = threading.Thread(target=self._worker_loop, name="experiment-worker", daemon=True)
        self.worker.start()

    def _declare_parameters(self):
        defaults = {
            "bag_root": "bags",
            "qos_override_file": "",
            "pre_roll_s": 0.5,
            "tail_s": 0.5,
            "recorder_start_timeout_s": 8.0,
            "stop_timeout_s": 10.0,
            "finalize_timeout_s": 15.0,
            "min_free_disk_mb": 100,
            "software_revision": "task003-mock-first",
            "fluent_reference": "way_2.c:phase_from_time+bodong1",
        }
        for name, value in defaults.items():
            self.node.declare_parameter(name, value)
        values = {name: self.node.get_parameter(name).value for name in defaults}
        for name in ("pre_roll_s", "tail_s", "recorder_start_timeout_s", "stop_timeout_s", "finalize_timeout_s"):
            if float(values[name]) <= 0.0:
                raise ValueError("{0} must be positive".format(name))
        if int(values["min_free_disk_mb"]) < 0:
            raise ValueError("min_free_disk_mb must be non-negative")
        return values

    def _now_ns(self):
        return self.node.get_clock().now().nanoseconds

    def _on_gait(self, message):
        with self._lock:
            self._latest["gait"] = message
            self._received_at["gait"] = time.monotonic()

    def _on_command(self, message):
        with self._lock:
            self._latest["command"] = message
            self._received_at["command"] = time.monotonic()

    def _on_telemetry(self, message):
        with self._lock:
            self._latest["telemetry"] = message
            self._received_at["telemetry"] = time.monotonic()

    def _latest_gait(self):
        with self._lock:
            return self._latest["gait"], self._received_at["gait"]

    def _set_state(self, state, transition, recorder_ready=None, last_error=None):
        if state not in STATES:
            raise ValueError("invalid experiment state")
        with self._lock:
            self._state["state"] = state
            self._state["transition_in_progress"] = transition
            if recorder_ready is not None:
                self._state["recorder_ready"] = recorder_ready
            if last_error is not None:
                self._state["last_error"] = last_error
        self._publish_state()

    def _publish_state(self):
        with self._lock:
            values = dict(self._state)
            gait = self._latest["gait"]
            message = self._messages["experiment"]()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.run_id = values["run_id"]
            message.state = values["state"]
            message.transition_in_progress = values["transition_in_progress"]
            message.recorder_ready = values["recorder_ready"]
            message.controller_state = gait.controller_state if gait is not None else "UNKNOWN"
            message.bag_relative_path = values["bag_relative_path"]
            message.last_error = values["last_error"]
            self.publisher.publish(message)

    def _on_set_running(self, request, response):
        target = bool(request.data)
        with self._lock:
            if not self._accepting:
                response.success = False
                response.message = "manager is shutting down"
                return response
            if self._queued_target is not None:
                response.success = self._queued_target == target
                response.message = (
                    "identical request already queued"
                    if response.success else "opposite request rejected while one is queued"
                )
                return response
            state = self._state["state"]
            transition = self._state["transition_in_progress"]
            accepted, duplicate, message = decide_set_running(state, transition, target)
            response.success = accepted
            response.message = message
            if accepted and not duplicate:
                try:
                    self._queue.put_nowait(("start" if target else "stop", None))
                    self._queued_target = target
                except queue.Full:
                    response.success = False
                    response.message = "experiment request queue is full"
        return response

    def _worker_loop(self):
        while not self._stop_event.is_set():
            try:
                operation, detail = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if operation == "start":
                    self._start_run()
                elif operation == "stop":
                    self._stop_active()
                elif operation == "abort":
                    self._stop_active(forced_error=detail)
                elif operation == "shutdown":
                    if self._active:
                        self._stop_active(forced_error=detail or "manager shutdown")
                    self._shutdown_done.set()
            except Exception as exc:
                self.node.get_logger().error("experiment operation failed: {0}".format(exc))
                try:
                    if self._active:
                        self._stop_active(forced_error=str(exc))
                    else:
                        self._set_state("ERROR", False, False, str(exc))
                except Exception as cleanup_exc:
                    self.node.get_logger().error("experiment cleanup failed: {0}".format(cleanup_exc))
                    self._set_state("ERROR", False, False, str(cleanup_exc))
            finally:
                if operation in {"start", "stop", "abort"}:
                    with self._lock:
                        self._failure_enqueued = False
                        if operation in {"start", "stop"}:
                            self._queued_target = None
                self._queue.task_done()

    def _write_session(self):
        if self._session_path is not None and self._session is not None:
            atomic_write_json(self._session_path, self._session)

    def _relative_bag_path(self, bag_path):
        return Path(os.path.relpath(str(bag_path), str(Path.cwd()))).as_posix()

    def _preflight(self):
        gait, received = self._latest_gait()
        if gait is None or time.monotonic() - received > 2.0:
            raise RuntimeError("fresh GaitState unavailable")
        if gait.controller_state not in {"DISARMED", "HOLDING"}:
            raise RuntimeError("controller must be DISARMED or HOLDING, got " + gait.controller_state)
        if not gait.configuration_valid:
            raise RuntimeError("controller configuration invalid: " + gait.configuration_error)
        if gait.backend not in {"mock", "dynamixel"}:
            raise RuntimeError("unsupported controller backend: " + gait.backend)
        qos_path = Path(self.parameters["qos_override_file"]).expanduser()
        if not qos_path.is_file() or not os.access(str(qos_path), os.R_OK):
            raise RuntimeError("qos_override_file is not a readable file")
        bag_root = Path(self.parameters["bag_root"]).expanduser().resolve()
        bag_root.mkdir(parents=True, exist_ok=True)
        if not bag_root.is_dir() or not os.access(str(bag_root), os.W_OK):
            raise RuntimeError("bag_root is not writable")
        free_mb = shutil.disk_usage(str(bag_root)).free / (1024 * 1024)
        if free_mb < int(self.parameters["min_free_disk_mb"]):
            raise RuntimeError("insufficient free disk: {0:.1f} MiB".format(free_mb))
        self._check_publishers()
        for key in ("command", "telemetry"):
            with self._lock:
                received = self._received_at[key]
            if received == 0.0 or time.monotonic() - received > 2.0:
                raise RuntimeError("fresh {0} stream unavailable".format(key))
        return gait, bag_root, qos_path.resolve()

    def _check_publishers(self):
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

        expected_durability = {
            name: DurabilityPolicy.TRANSIENT_LOCAL if "transient_local" in QOS_CONTRACT[name]
            else DurabilityPolicy.VOLATILE
            for name in REQUIRED_TOPICS
        }
        for name, type_name in REQUIRED_TOPICS.items():
            endpoints = self.node.get_publishers_info_by_topic(name)
            compatible = any(
                endpoint.topic_type == type_name
                and endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                and endpoint.qos_profile.durability == expected_durability[name]
                for endpoint in endpoints
            )
            if not compatible:
                raise RuntimeError("missing compatible publisher/type/QoS on " + name)

    def _start_run(self):
        # 先分配本次 run_id 并清空上一会话记录，避免 latched 的 STARTING 消息携带旧 run_id。
        run_id = make_run_id()
        with self._lock:
            self._state["run_id"] = run_id
            self._state["bag_relative_path"] = ""
        self._set_state("STARTING", True, False, "")
        gait, bag_root, qos_path = self._preflight()
        run_dir = bag_root / run_id
        run_dir.mkdir(parents=False, exist_ok=False)
        bag_path = run_dir / "bag"
        if bag_path.exists():
            raise RuntimeError("bag output already exists")
        relative = self._relative_bag_path(bag_path)
        with self._lock:
            self._state["run_id"] = run_id
            self._state["bag_relative_path"] = relative
        self._session = new_session(
            run_id, gait.backend, self.parameters, relative, self._now_ns(),
            gait.active_config_revision,
        )
        self._session_path = run_dir / "session.json"
        self._run_dir = run_dir
        self._active = True
        self._write_session()
        self._publish_state()

        baseline = {name: self.node.count_subscribers(name) for name in REQUIRED_TOPICS}
        recorder_log = (run_dir / "recorder.log").open("wb")
        try:
            self._recorder = subprocess.Popen(
                make_recorder_argv(qos_path, bag_path),
                stdin=subprocess.DEVNULL,
                stdout=recorder_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        finally:
            recorder_log.close()
        started = time.monotonic()
        self._wait_recorder_ready(baseline, started)
        accepted = self._get_complete_parameter_snapshot()
        self._session["accepted_parameters"] = accepted
        if accepted.get("backend") != gait.backend:
            raise RuntimeError("parameter snapshot backend does not match GaitState")
        self._write_session()
        self._recording_delay(float(self.parameters["pre_roll_s"]))
        self._session["status"] = "RECORDING"
        self._write_session()
        self._set_state("STARTING", True, True)
        self._recording_delay(0.2)

        gait, _ = self._latest_gait()
        if gait is None or gait.controller_state in FAULT_STATES:
            raise RuntimeError("controller fault before arm")
        if gait.controller_state == "DISARMED":
            self._call_trigger("arm")
            self._wait_controller({"HOLDING"}, time.monotonic(), self.parameters["stop_timeout_s"])
        elif gait.controller_state != "HOLDING":
            raise RuntimeError("controller left safe start state before arm")
        self._recording_delay(0.0)
        self._call_trigger("start")
        self._wait_controller({"RUNNING"}, time.monotonic(), self.parameters["stop_timeout_s"])
        self._set_state("RUNNING", False, True)

    def _wait_recorder_ready(self, baseline, started):
        timeout = float(self.parameters["recorder_start_timeout_s"])
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_recorder()
            subscriptions_ready = all(
                self.node.count_subscribers(name) > baseline[name] for name in REQUIRED_TOPICS
            )
            with self._lock:
                received = dict(self._received_at)
            fresh = all(
                received[key] >= started and time.monotonic() - received[key] < 1.0
                for key in ("gait", "command", "telemetry")
            )
            if subscriptions_ready and fresh:
                return
            time.sleep(0.05)
        raise RuntimeError("recorder subscriptions or fresh streams not ready within timeout")

    def _check_recorder(self):
        if self._recorder is None:
            raise RuntimeError("recorder was not started")
        code = self._recorder.poll()
        if code is not None:
            raise RuntimeError("recorder exited prematurely with code {0}".format(code))
        gait, _ = self._latest_gait()
        if gait is not None and gait.controller_state in FAULT_STATES:
            raise RuntimeError("controller entered " + gait.controller_state)

    def _recording_delay(self, duration_s):
        deadline = time.monotonic() + max(0.0, duration_s)
        while True:
            if self._recorder is not None and self._recorder.poll() is None:
                self._check_recorder()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(0.05, remaining))

    def _call_client(self, client, request, timeout_s):
        timeout_s = float(timeout_s)
        if not client.wait_for_service(timeout_sec=min(1.0, timeout_s)):
            raise RuntimeError("service unavailable: " + client.srv_name)
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout_s):
            future.cancel()
            raise RuntimeError("service timeout: " + client.srv_name)
        result = future.result()
        if result is None:
            raise RuntimeError("service failed: " + client.srv_name)
        return result

    def _call_trigger(self, name):
        response = self._call_client(
            self.trigger_clients[name], self._trigger_type.Request(),
            self.parameters["stop_timeout_s"],
        )
        if not response.success:
            raise RuntimeError("controller {0} rejected: {1}".format(name, response.message))
        return response

    def _wait_controller(self, expected, after, timeout_s):
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            gait, received = self._latest_gait()
            if gait is not None and received >= after:
                if gait.controller_state in expected:
                    return gait
                if gait.controller_state in FAULT_STATES:
                    raise RuntimeError("controller entered " + gait.controller_state)
            time.sleep(0.05)
        raise RuntimeError("timed out waiting for controller " + "/".join(sorted(expected)))

    def _get_complete_parameter_snapshot(self):
        list_request = self._list_parameters_type.Request()
        list_request.depth = self._list_parameters_type.Request.DEPTH_RECURSIVE
        list_request.prefixes = []
        listed = self._call_client(
            self.list_parameters_client, list_request, self.parameters["stop_timeout_s"]
        )
        names = sorted(listed.result.names)
        request = self._get_parameters_type.Request()
        request.names = names
        response = self._call_client(
            self.get_parameters_client, request, self.parameters["stop_timeout_s"]
        )
        if len(response.values) != len(names):
            raise RuntimeError("incomplete controller parameter snapshot")
        from rclpy.parameter import parameter_value_to_python

        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)
        }

    def _stop_active(self, forced_error=None):
        if not self._active or self._session is None:
            if forced_error:
                self._set_state("ERROR", False, False, forced_error)
            return
        if forced_error is None and self._state["state"] == "RUNNING":
            if self._recorder is None:
                forced_error = "recorder unavailable during RUNNING"
            elif self._recorder.poll() is not None:
                forced_error = "recorder exited during RUNNING with code {0}".format(
                    self._recorder.poll()
                )
        errors = [forced_error] if forced_error else []
        self._session["status"] = "ERROR" if forced_error else "RECORDING"
        self._write_session()
        recorder_alive = self._recorder is not None and self._recorder.poll() is None
        self._set_state("STOPPING", True, recorder_alive, forced_error or "")
        gait, _ = self._latest_gait()
        controller_fault = gait is not None and gait.controller_state in FAULT_STATES
        hold_confirmed = False
        if gait is not None and gait.controller_state != "DISARMED" and not controller_fault:
            requested_at = time.monotonic()
            try:
                self._call_trigger("stop_hold")
                self._wait_controller(
                    {"HOLDING"}, requested_at, self.parameters["stop_timeout_s"]
                )
                hold_confirmed = True
            except Exception as exc:
                errors.append("stop_hold confirmation failed: {0}".format(exc))
        elif controller_fault:
            errors.append("controller is " + gait.controller_state)
        else:
            errors.append("controller is not available for stop_hold")
        self._session["stop_hold_confirmed"] = hold_confirmed
        self._write_session()

        try:
            self._recording_delay(float(self.parameters["tail_s"]))
        except Exception as exc:
            errors.append("tail recording failed: {0}".format(exc))

        exit_code, termination_signals, finalize_errors = self._finalize_recorder()
        errors.extend(finalize_errors)
        self._session["recorder_exit_code"] = exit_code
        self._session["termination_signals"] = termination_signals
        if exit_code != 0:
            errors.append("recorder exit code is {0}".format(exit_code))

        from .bag_audit import audit_run

        try:
            audit = audit_run(self._run_dir, info_timeout_s=float(self.parameters["finalize_timeout_s"]))
            validation = audit.get("validation") or {}
            bag_valid = audit.get("valid") is True
            failures = list(validation.get("failures", []))
        except Exception as exc:
            bag_valid = False
            validation = {}
            failures = ["audit aborted: {0}".format(exc)]
        validation.setdefault("metadata_readable", False)
        validation.setdefault("required_topics_present", False)
        validation.setdefault("per_motor_valid_angle_samples", [0, 0, 0, 0])
        validation.setdefault("per_motor_valid_current_samples", [0, 0, 0, 0])
        validation.setdefault("sequence_gap_count", 0)
        validation["failures"] = errors + [item for item in failures if item not in errors]
        self._session["validation"] = validation
        bag_valid = bag_valid and not errors

        gait, _ = self._latest_gait()
        end_state = gait.controller_state if gait is not None else "UNKNOWN"
        self._session["controller_state_at_end"] = (
            end_state if end_state in {"HOLDING", "FAULT", "ESTOP_LATCHED"} else "UNKNOWN"
        )
        self._session["stopped_ros_time_ns"] = self._now_ns()
        self._session["status"] = "CLOSED_VALID" if bag_valid and hold_confirmed else "CLOSED_INVALID"
        final_state = final_experiment_state(hold_confirmed, bag_valid, bool(forced_error))
        if not errors and not bag_valid:
            errors.append("bag validation failed")
        final_error = "; ".join(errors + validation["failures"][len(errors):]) if not bag_valid or errors else ""
        self._write_session()
        self._active = False
        self._recorder = None
        self._set_state(final_state, False, False, final_error)

    def _finalize_recorder(self):
        process = self._recorder
        if process is None:
            return None, [], ["recorder process unavailable"]
        signals = []
        errors = []
        timeout = float(self.parameters["finalize_timeout_s"])
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                signals.append("SIGINT")
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                errors.append("recorder SIGINT finalize timeout")
            except ProcessLookupError:
                pass
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                signals.append("SIGTERM")
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                errors.append("recorder SIGTERM timeout")
            except ProcessLookupError:
                pass
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
                signals.append("SIGKILL")
                process.wait(timeout=2.0)
                errors.append("recorder required explicit SIGKILL after finalize timeouts")
            except (subprocess.TimeoutExpired, ProcessLookupError) as exc:
                errors.append("recorder could not be reaped: {0}".format(exc))
        code = process.poll()
        return code if code is not None else "NOT_FINALIZED", signals, errors

    def _heartbeat(self):
        with self._lock:
            state = self._state["state"]
            process = self._recorder if self._active else None
            failure_enqueued = self._failure_enqueued
        gait, _ = self._latest_gait()
        reason = None
        if state == "RUNNING" and process is None:
            reason = "recorder unavailable during RUNNING"
        elif state == "RUNNING" and process.poll() is not None:
            reason = "recorder exited during RUNNING with code {0}".format(process.poll())
        if state == "RUNNING" and gait is not None and gait.controller_state in FAULT_STATES:
            reason = "controller entered " + gait.controller_state
        if reason and not failure_enqueued:
            with self._lock:
                self._failure_enqueued = True
            recorder_alive = process is not None and process.poll() is None
            self._set_state("STOPPING", True, recorder_alive, reason)
            try:
                self._queue.put_nowait(("abort", reason))
            except queue.Full:
                self._set_state("ERROR", False, False, reason + "; abort queue full")
        elif gait is not None and gait.controller_state in FAULT_STATES and state in {"IDLE", "HOLDING"}:
            self._set_state("ERROR", False, False, "controller entered " + gait.controller_state)
        self._publish_state()

    def initiate_shutdown(self):
        with self._lock:
            if not self._accepting:
                return
            self._accepting = False
            needs_cleanup = self._active or self._state["state"] in {"STARTING", "RUNNING", "STOPPING"}
        if needs_cleanup:
            try:
                self._queue.put(("shutdown", "manager shutdown"), timeout=0.5)
            except queue.Full:
                self.node.get_logger().error("shutdown cleanup could not be queued")
                self._shutdown_done.set()
        else:
            self._shutdown_done.set()

    def finish_shutdown(self):
        self._stop_event.set()
        self.worker.join(timeout=2.0)
        if self.worker.is_alive():
            self.node.get_logger().error("experiment worker did not stop within its bound")


def main():
    import rclpy

    rclpy.init()
    manager = None
    try:
        manager = ExperimentManager()
        rclpy.spin(manager.node)
    except KeyboardInterrupt:
        pass
    finally:
        if manager is not None:
            manager.initiate_shutdown()
            timeout = (
                float(manager.parameters["stop_timeout_s"])
                + float(manager.parameters["finalize_timeout_s"])
                + float(manager.parameters["tail_s"])
                + 8.0
            )
            deadline = time.monotonic() + timeout
            while not manager._shutdown_done.is_set() and time.monotonic() < deadline:
                try:
                    rclpy.spin_once(manager.node, timeout_sec=0.1)
                except Exception:
                    break
            manager.finish_shutdown()
            manager.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
