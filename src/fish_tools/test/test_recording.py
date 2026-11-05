import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from datetime import datetime, timezone

from fish_tools.bag_audit import REQUIRED_TOPICS, session_close_valid
from fish_tools.experiment_manager import (
    atomic_write_json,
    final_experiment_state,
    make_recorder_argv,
    make_run_id,
    new_session,
)


PARAMETERS = {
    "software_revision": "test-revision",
    "fluent_reference": "test-fluent-digest",
}


class RecordingContractTest(unittest.TestCase):
    def test_recorder_argv_is_explicit_and_scoped(self):
        argv = make_recorder_argv("/tmp/run/bag.bag")
        self.assertEqual(argv[:4], ["rosbag", "record", "-O", "/tmp/run/bag.bag"])
        self.assertEqual(set(argv[4:]), set(REQUIRED_TOPICS))
        self.assertNotIn("--all", argv)
        self.assertNotIn("-a", argv)

    def test_run_id_does_not_repeat_within_same_timestamp(self):
        now = datetime(2026, 11, 4, tzinfo=timezone.utc)
        self.assertNotEqual(make_run_id(now), make_run_id(now))

    def test_session_fields_and_completion_are_separate(self):
        session = new_session("run", "mock", PARAMETERS, "bags/run/bag.bag", 123, 7)
        required = {
            "schema_version", "run_id", "backend", "ros_distro", "software_revision",
            "fluent_reference_revision_or_digest", "started_ros_time_ns",
            "stopped_ros_time_ns", "status", "controller_state_at_end",
            "stop_hold_confirmed", "accepted_parameters", "initial_active_config_revision",
            "required_topics", "bag_relative_path", "recorder_exit_code", "validation",
        }
        self.assertTrue(required.issubset(session))
        self.assertEqual(session["ros_distro"], "noetic")
        self.assertFalse(session_close_valid(session))
        session.update(status="CLOSED_VALID", stop_hold_confirmed=True, recorder_exit_code=0)
        session["validation"].update(
            metadata_readable=True,
            required_topics_present=True,
            per_motor_valid_angle_samples=[1, 1, 1, 1],
            per_motor_valid_current_samples=[1, 1, 1, 1],
        )
        self.assertTrue(session_close_valid(session))
        session["stop_hold_confirmed"] = False
        self.assertFalse(session_close_valid(session))

    def test_hold_and_bag_failure_are_not_collapsed(self):
        self.assertEqual(final_experiment_state(True, False, False), "HOLDING")
        self.assertEqual(final_experiment_state(False, True, False), "ERROR")
        self.assertEqual(final_experiment_state(True, True, True), "ERROR")
        self.assertEqual(final_experiment_state(True, True, False), "HOLDING")

    def test_atomic_session_update(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.json"
            atomic_write_json(path, {"status": "STARTING"})
            atomic_write_json(path, {"status": "CLOSED_INVALID"})
            self.assertEqual(json.loads(path.read_text())["status"], "CLOSED_INVALID")
            self.assertEqual(list(Path(directory).glob("*.tmp-*")), [])


@unittest.skipUnless(
    os.environ.get("FISH_RUN_ROS_TESTS") == "1",
    "actual 10s ROS bag integration requires FISH_RUN_ROS_TESTS=1 and a built catkin workspace",
)
class ActualRosBagIntegrationTest(unittest.TestCase):
    def test_ten_second_mock_run_and_audit(self):
        import rospy
        from fish_interfaces.msg import ExperimentState
        from std_srvs.srv import SetBool

        from fish_tools.bag_audit import audit_run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            launch_log = (root / "launch.log").open("wb")
            launch = subprocess.Popen(
                ["roslaunch", "fish_bringup", "fish.launch", "bag_root:=" + str(root)],
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            latest = []
            client = None
            try:
                rospy.init_node("fish_bag_integration_test", anonymous=True)
                rospy.Subscriber(
                    "/fish/experiment_state", ExperimentState, latest.append, queue_size=1)
                client = rospy.ServiceProxy("/fish/experiment/set_running", SetBool)

                def spin_until(predicate, timeout):
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        if launch.poll() is not None:
                            self.fail("launch exited early; see " + str(root / "launch.log"))
                        if predicate():
                            return True
                        time.sleep(0.05)
                    return False

                def request(target):
                    message = SetBool.Request()
                    message.data = target
                    response = client(message)
                    self.assertIsNotNone(response)
                    self.assertTrue(response.success, response.message)

                self.assertTrue(spin_until(lambda: client.wait_for_service(timeout=0.5), 20.0),
                                "manager unavailable")
                self.assertTrue(
                    spin_until(
                        lambda: latest and latest[-1].state in {"IDLE", "HOLDING"},
                        20.0),
                    "manager did not become startable")
                request(True)
                self.assertTrue(
                    spin_until(lambda: latest and latest[-1].state == "RUNNING", 40.0),
                    "run did not reach RUNNING: " +
                    (latest[-1].last_error if latest else "no state"))
                spin_until(lambda: latest and latest[-1].state == "ERROR", 10.0)
                self.assertEqual(latest[-1].state, "RUNNING", latest[-1].last_error)
                request(False)
                self.assertTrue(
                    spin_until(
                        lambda: latest and not latest[-1].transition_in_progress
                        and latest[-1].state in {"HOLDING", "ERROR"},
                        40.0),
                    "run did not finish stopping")
                self.assertEqual(latest[-1].state, "HOLDING", latest[-1].last_error)
                self.assertEqual(latest[-1].last_error, "")
                sessions = [path for path in root.glob("*/session.json")]
                self.assertEqual(len(sessions), 1)
                report = audit_run(sessions[0])
                self.assertTrue(report["valid"], json.dumps(report, indent=2))
                session = json.loads(sessions[0].read_text())
                self.assertTrue(session_close_valid(session), json.dumps(session, indent=2))
                self.assertEqual(report["evidence_class"], "mock")
            finally:
                if client is not None and latest and latest[-1].state == "RUNNING":
                    try:
                        message = SetBool.Request()
                        message.data = False
                        client(message)
                        deadline = time.monotonic() + 40.0
                        while time.monotonic() < deadline:
                            if (latest and not latest[-1].transition_in_progress
                                    and latest[-1].state in {"HOLDING", "ERROR"}):
                                break
                            time.sleep(0.05)
                    except Exception:
                        pass
                try:
                    rospy.signal_shutdown("integration test done")
                except Exception:
                    pass
                if launch.poll() is None:
                    os.killpg(launch.pid, signal.SIGINT)
                    try:
                        launch.wait(timeout=15.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(launch.pid, signal.SIGTERM)
                        launch.wait(timeout=5.0)
                launch_log.close()


if __name__ == "__main__":
    unittest.main()
