"""Offline ROS 1 rosbag auditing without import-time ROS dependencies."""

import argparse
import json
import os
import sys

from ._compat import CommandTimeout, isfinite, open_text, run_command, text_type


REQUIRED_TOPICS = {
    "/fish/flapping_command": "fish_interfaces/FlappingCommand",
    "/fish/motor_telemetry": "fish_interfaces/MotorTelemetry",
    "/fish/joint_states": "sensor_msgs/JointState",
    "/fish/gait_state": "fish_interfaces/GaitState",
    "/fish/experiment_state": "fish_interfaces/ExperimentState",
    "/fish/diagnostics": "diagnostic_msgs/DiagnosticArray",
}
TOPIC_TRANSPORT = {
    "/fish/flapping_command": "stream",
    "/fish/motor_telemetry": "stream",
    "/fish/joint_states": "stream",
    "/fish/gait_state": "latched",
    "/fish/experiment_state": "latched",
    "/fish/diagnostics": "stream",
}
REQUIRED_EXPERIMENT_STATES = {"STARTING", "RUNNING", "STOPPING"}


def required_topic_entries():
    return [
        {"name": name, "type": type_name, "transport": TOPIC_TRANSPORT[name]}
        for name, type_name in REQUIRED_TOPICS.items()
    ]


def empty_validation():
    return {
        "metadata_readable": False,
        "required_topics_present": False,
        "per_motor_valid_angle_samples": [0, 0, 0, 0],
        "per_motor_valid_current_samples": [0, 0, 0, 0],
        "sequence_gap_count": 0,
        "command_sequence_gap_count": 0,
        "telemetry_sequence_gap_count": 0,
        "sequence_statistics": {},
        "failures": [],
    }


def sequence_statistics(sequences):
    """Count missing, duplicate, and non-monotonic samples in arrival order."""
    report = {
        "sample_count": len(sequences),
        "gap_count": 0,
        "gap_event_count": 0,
        "missing_sample_count": 0,
        "duplicate_count": 0,
        "out_of_order_count": 0,
    }
    for previous, current in zip(sequences, sequences[1:]):
        if current > previous + 1:
            missing = current - previous - 1
            report["gap_event_count"] += 1
            report["gap_count"] += missing
            report["missing_sample_count"] += missing
        elif current == previous:
            report["duplicate_count"] += 1
        elif current < previous:
            report["out_of_order_count"] += 1
    return report


def count_valid_motor_samples(records):
    """Return per-index finite, valid angle/current sample counts."""
    angles = [0, 0, 0, 0]
    currents = [0, 0, 0, 0]
    for record in records:
        valid = record.get("valid", ())
        positions = record.get("position_rad", ())
        current_values = record.get("current_a", ())
        if not (len(valid) == len(positions) == len(current_values) == 4):
            continue
        for index in range(4):
            if not valid[index]:
                continue
            if isfinite(float(positions[index])):
                angles[index] += 1
            if isfinite(float(current_values[index])):
                currents[index] += 1
    return {
        "per_motor_valid_angle_samples": angles,
        "per_motor_valid_current_samples": currents,
    }


def topic_contract_failures(session_topics, recorded_types):
    """Check both the session declaration and actual rosbag topics."""
    failures = []
    declared = {}
    for entry in session_topics or []:
        name = entry.get("name")
        declared[name] = entry.get("type")
        expected = REQUIRED_TOPICS.get(name)
        if expected is None:
            failures.append("unexpected session topic: {0}".format(name))
        elif entry.get("type") != expected:
            failures.append("session type mismatch for {0}".format(name))
        if not entry.get("transport"):
            failures.append("missing session transport info for {0}".format(name))
    missing_declarations = set(REQUIRED_TOPICS) - set(declared)
    if missing_declarations:
        failures.append("session topics missing: {0}".format(
            ", ".join(sorted(missing_declarations))))
    for name, expected in REQUIRED_TOPICS.items():
        actual = recorded_types.get(name)
        if actual is None:
            failures.append("recorded topic missing: {0}".format(name))
        elif actual != expected:
            failures.append("recorded type mismatch for {0}: {1}".format(name, actual))
    return failures


def evidence_class(backend):
    if backend == "mock":
        return "mock"
    if backend == "dynamixel":
        return "hardware"
    return "unknown"


def session_close_valid(session):
    validation = session.get("validation") or {}
    return bool(
        session.get("status") == "CLOSED_VALID"
        and session.get("stop_hold_confirmed") is True
        and session.get("recorder_exit_code") == 0
        and validation.get("metadata_readable") is True
        and validation.get("required_topics_present") is True
        and not validation.get("failures")
        and len(validation.get("per_motor_valid_angle_samples", [])) == 4
        and len(validation.get("per_motor_valid_current_samples", [])) == 4
        and all(value > 0 for value in validation.get("per_motor_valid_angle_samples", []))
        and all(value > 0 for value in validation.get("per_motor_valid_current_samples", []))
    )


def _add_failure(validation, message):
    if message not in validation["failures"]:
        validation["failures"].append(message)


def _resolve_session(input_path):
    path = os.path.abspath(os.path.expanduser(str(input_path)))
    session_path = os.path.join(path, "session.json") if os.path.isdir(path) else path
    with open_text(session_path, "r") as stream:
        session = json.load(stream)
    run_dir = os.path.dirname(session_path)
    bag_path = os.path.join(run_dir, "bag.bag")
    if not os.path.exists(bag_path):
        configured = session.get("bag_relative_path", "")
        if os.path.isabs(configured):
            bag_path = configured
        else:
            candidate = os.path.abspath(os.path.join(os.getcwd(), configured))
            if os.path.exists(candidate):
                bag_path = candidate
    return session_path, session, os.path.abspath(bag_path)


def _run_bag_info(bag_path, timeout_s, validation):
    try:
        returncode, stdout, stderr = run_command(
            ["rosbag", "info", str(bag_path)], timeout_s)
    except (OSError, IOError, CommandTimeout) as exc:
        _add_failure(validation, "rosbag info failed: {0}".format(exc))
        return {"returncode": None, "stdout": "", "stderr": str(exc)}
    result = {
        "returncode": returncode,
        "stdout": stdout,
        "stderr": stderr,
    }
    validation["metadata_readable"] = returncode == 0
    if returncode != 0:
        _add_failure(validation, "rosbag info returned {0}".format(returncode))
    return result


def _read_bag(bag_path, session, validation):
    try:
        import rosbag
    except ImportError as exc:
        _add_failure(validation, "rosbag Python API unavailable: {0}".format(exc))
        return {}, {}

    bag = None
    try:
        bag = rosbag.Bag(str(bag_path), "r")
        info = bag.get_type_and_topic_info()
        recorded_types = {
            name: entry.msg_type for name, entry in info.topics.items()
        }
        counts = {name: 0 for name in REQUIRED_TOPICS}
        telemetry_records = []
        sequences = {
            "/fish/flapping_command": [],
            "/fish/motor_telemetry": [],
        }
        experiment_messages = []
        gait_messages = []
        observed_backends = {
            name: set() for name in (
                "/fish/flapping_command", "/fish/motor_telemetry", "/fish/gait_state")
        }
        observed_revisions = {name: set() for name in observed_backends}
        malformed_telemetry = False
        for topic, message, _stamp in bag.read_messages(topics=list(REQUIRED_TOPICS)):
            counts[topic] = counts.get(topic, 0) + 1
            if topic == "/fish/motor_telemetry":
                record = {
                    "valid": list(message.valid),
                    "position_rad": list(message.position_rad),
                    "current_a": list(message.current_a),
                }
                if not all(len(record[key]) == 4 for key in record):
                    malformed_telemetry = True
                telemetry_records.append(record)
                sequences[topic].append(int(message.sequence))
                observed_backends[topic].add(message.backend)
                observed_revisions[topic].add(int(message.active_config_revision))
            elif topic == "/fish/flapping_command":
                sequences[topic].append(int(message.sequence))
                observed_backends[topic].add(message.backend)
                observed_revisions[topic].add(int(message.active_config_revision))
            elif topic == "/fish/gait_state":
                gait_messages.append(message)
                observed_backends[topic].add(message.backend)
                observed_revisions[topic].add(int(message.active_config_revision))
            elif topic == "/fish/experiment_state":
                experiment_messages.append(message)
    except Exception as exc:
        _add_failure(validation, "rosbag message audit failed: {0}".format(exc))
        return {}, {}
    finally:
        if bag is not None:
            bag.close()

    for failure in topic_contract_failures(session.get("required_topics"), recorded_types):
        _add_failure(validation, failure)
    validation["required_topics_present"] = all(
        recorded_types.get(name) == type_name
        for name, type_name in REQUIRED_TOPICS.items()
    )
    if malformed_telemetry:
        _add_failure(validation, "telemetry arrays must have four entries")
    validation.update(count_valid_motor_samples(telemetry_records))
    for key in ("per_motor_valid_angle_samples", "per_motor_valid_current_samples"):
        for index, count in enumerate(validation[key]):
            if count == 0:
                _add_failure(validation, "no valid {0} samples for motor index {1}".format(
                    key, index))

    command_stats = sequence_statistics(sequences["/fish/flapping_command"])
    telemetry_stats = sequence_statistics(sequences["/fish/motor_telemetry"])
    validation["command_sequence_gap_count"] = command_stats["gap_count"]
    validation["telemetry_sequence_gap_count"] = telemetry_stats["gap_count"]
    validation["sequence_gap_count"] = (
        command_stats["gap_count"] + telemetry_stats["gap_count"]
    )
    validation["sequence_statistics"] = {
        "flapping_command": command_stats,
        "motor_telemetry": telemetry_stats,
    }

    backend = session.get("backend")
    if evidence_class(backend) == "unknown":
        _add_failure(validation, "unsupported backend label: {0}".format(backend))
    for topic, values in observed_backends.items():
        if not values:
            _add_failure(validation, "no backend evidence on {0}".format(topic))
        elif values != {backend}:
            _add_failure(validation, "backend mismatch on {0}: {1}".format(
                topic, sorted(values)))

    initial_revision = session.get("initial_active_config_revision")
    for topic, values in observed_revisions.items():
        if initial_revision not in values:
            _add_failure(validation, "initial active revision absent on {0}".format(topic))

    run_id = session.get("run_id")
    experiment_states = {message.state for message in experiment_messages}
    for state in sorted(REQUIRED_EXPERIMENT_STATES - experiment_states):
        _add_failure(validation, "required experiment state absent: {0}".format(state))
    if not any(
        message.state == "STARTING" and message.recorder_ready
        for message in experiment_messages
    ):
        _add_failure(validation, "record-ready STARTING state absent")
    if any(message.run_id != run_id for message in experiment_messages):
        _add_failure(validation, "experiment run_id mismatch")
    if not any(message.controller_state == "HOLDING" for message in gait_messages):
        _add_failure(validation, "HOLDING gait feedback absent")
    for topic in (
        "/fish/flapping_command",
        "/fish/motor_telemetry",
        "/fish/joint_states",
        "/fish/gait_state",
        "/fish/experiment_state",
        "/fish/diagnostics",
    ):
        if counts.get(topic, 0) == 0:
            _add_failure(validation, "no messages on {0}".format(topic))

    details = {
        "topic_message_counts": counts,
        "recorded_topic_types": recorded_types,
        "experiment_states": sorted(experiment_states),
        "controller_states": sorted({
            message.controller_state for message in gait_messages
        }),
    }
    return recorded_types, details


def audit_run(input_path, info_timeout_s=30.0):
    """Audit a run directory or session.json and return a JSON-safe report."""
    validation = empty_validation()
    try:
        session_path, session, bag_path = _resolve_session(input_path)
    except (OSError, IOError, ValueError, TypeError) as exc:
        _add_failure(validation, "cannot load session: {0}".format(exc))
        return {
            "valid": False,
            "run_id": None,
            "backend": "unknown",
            "evidence_class": "unknown",
            "validation": validation,
        }

    report = {
        "valid": False,
        "schema_version": session.get("schema_version"),
        "run_id": session.get("run_id"),
        "backend": session.get("backend"),
        "evidence_class": evidence_class(session.get("backend")),
        "session_path": str(session_path),
        "bag_path": str(bag_path),
        "validation": validation,
    }
    if session.get("schema_version") != 1:
        _add_failure(validation, "unsupported session schema_version")
    if not session.get("run_id"):
        _add_failure(validation, "session run_id is empty")
    for field in ("software_revision", "fluent_reference_revision_or_digest"):
        if not isinstance(session.get(field), text_type) or not session.get(field):
            _add_failure(validation, "session revision field missing: " + field)
    if not os.path.isfile(bag_path):
        _add_failure(validation, "bag file missing: {0}".format(bag_path))
        return report
    report["bag_info"] = _run_bag_info(bag_path, info_timeout_s, validation)
    _recorded_types, details = _read_bag(bag_path, session, validation)
    report.update(details)
    if session.get("stop_hold_confirmed") is not True:
        _add_failure(validation, "session stop_hold_confirmed is not true")
    report["valid"] = bool(
        validation["metadata_readable"]
        and validation["required_topics_present"]
        and not validation["failures"]
    )
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Audit one fish rosbag session.")
    parser.add_argument("path", help="Run directory containing session.json, or its path")
    parser.add_argument("--info-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    try:
        report = audit_run(args.path, info_timeout_s=args.info_timeout)
    except Exception as exc:
        report = {
            "valid": False,
            "error": "audit aborted: {0}".format(exc),
        }
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0 if report.get("valid") is True else 1


if __name__ == "__main__":
    sys.exit(main())
