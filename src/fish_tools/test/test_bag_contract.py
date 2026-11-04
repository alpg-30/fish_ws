import math
import tempfile
import unittest

from fish_tools.bag_audit import (
    REQUIRED_TOPICS,
    audit_run,
    count_valid_motor_samples,
    evidence_class,
    sequence_statistics,
    topic_contract_failures,
)
from fish_tools.experiment_manager import required_topic_entries


class TopicContractTest(unittest.TestCase):
    def test_exact_required_topic_types(self):
        self.assertEqual(len(REQUIRED_TOPICS), 7)
        self.assertEqual(REQUIRED_TOPICS["/fish/flapping_command"], "fish_interfaces/msg/FlappingCommand")
        self.assertEqual(REQUIRED_TOPICS["/fish/motor_telemetry"], "fish_interfaces/msg/MotorTelemetry")
        self.assertEqual(REQUIRED_TOPICS["/fish/experiment_state"], "fish_interfaces/msg/ExperimentState")
        self.assertIn("/parameter_events", REQUIRED_TOPICS)

    def test_session_and_recorded_topic_contract(self):
        entries = required_topic_entries()
        self.assertEqual(topic_contract_failures(entries, REQUIRED_TOPICS), [])
        failures = topic_contract_failures(entries, dict(REQUIRED_TOPICS))
        self.assertEqual(failures, [])
        missing = dict(REQUIRED_TOPICS)
        del missing["/fish/gait_state"]
        self.assertTrue(any("gait_state" in failure for failure in topic_contract_failures(entries, missing)))

    def test_missing_session_file_is_a_real_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            report = audit_run(directory)
        self.assertFalse(report["valid"])
        self.assertTrue(report["validation"]["failures"])


class SequenceContractTest(unittest.TestCase):
    def test_command_and_telemetry_statistics_are_independent(self):
        command = sequence_statistics([0, 1, 4, 5])
        telemetry = sequence_statistics([10, 11, 12])
        self.assertEqual(command["gap_count"], 2)
        self.assertEqual(command["gap_event_count"], 1)
        self.assertEqual(command["missing_sample_count"], 2)
        self.assertEqual(telemetry["gap_count"], 0)
        total = command["gap_count"] + telemetry["gap_count"]
        self.assertEqual(total, 2)

    def test_duplicates_and_reordering_are_not_counted_as_gaps(self):
        report = sequence_statistics([1, 1, 0])
        self.assertEqual(report["gap_count"], 0)
        self.assertEqual(report["duplicate_count"], 1)
        self.assertEqual(report["out_of_order_count"], 1)


class TelemetryContractTest(unittest.TestCase):
    def test_four_motor_valid_finite_samples(self):
        records = [
            {
                "valid": [True, True, True, True],
                "position_rad": [0.0, 0.1, -0.1, 0.2],
                "current_a": [0.0, -0.00269, 0.2, -0.3],
            },
            {
                "valid": [False, True, True, True],
                "position_rad": [99.0, math.nan, 0.3, 0.4],
                "current_a": [99.0, 0.1, math.inf, -0.1],
            },
        ]
        result = count_valid_motor_samples(records)
        self.assertEqual(result["per_motor_valid_angle_samples"], [1, 1, 2, 2])
        self.assertEqual(result["per_motor_valid_current_samples"], [1, 2, 1, 2])

    def test_mock_evidence_cannot_be_labelled_hardware(self):
        self.assertEqual(evidence_class("mock"), "mock")
        self.assertEqual(evidence_class("dynamixel"), "hardware")
        self.assertEqual(evidence_class("unknown"), "unknown")


if __name__ == "__main__":
    unittest.main()
