import unittest

from fish_tools.experiment_manager import decide_set_running
from fish_tools.keyboard_console import SpaceDebouncer, stop_wait_complete, target_for_state


class ToggleTargetTest(unittest.TestCase):
    def test_target_comes_only_from_experiment_state(self):
        self.assertIs(target_for_state("IDLE"), True)
        self.assertIs(target_for_state("HOLDING"), True)
        self.assertIs(target_for_state("RUNNING"), False)
        for state in ("STARTING", "STOPPING", "ERROR", "UNKNOWN"):
            self.assertIsNone(target_for_state(state))

    def test_debounce_and_busy_do_not_generate_requests(self):
        gate = SpaceDebouncer(debounce_s=0.35)
        self.assertIs(gate.accept("IDLE", False, 10.0), True)
        self.assertIsNone(gate.accept("IDLE", False, 10.1))
        self.assertIsNone(gate.accept("RUNNING", False, 10.2))
        self.assertIs(gate.accept("RUNNING", False, 10.4), False)
        self.assertIsNone(gate.accept("STARTING", True, 11.0))
        self.assertIsNone(gate.accept("STOPPING", False, 11.0))

    def test_exit_wait_requires_completed_transition(self):
        self.assertFalse(stop_wait_complete("STOPPING", True))
        self.assertTrue(stop_wait_complete("HOLDING", False))
        self.assertTrue(stop_wait_complete("ERROR", False))
        self.assertFalse(stop_wait_complete("RUNNING", False))


class ManagerRequestDecisionTest(unittest.TestCase):
    def test_duplicate_targets_are_idempotent(self):
        self.assertEqual(decide_set_running("STARTING", True, True)[:2], (True, True))
        self.assertEqual(decide_set_running("STOPPING", True, False)[:2], (True, True))
        self.assertEqual(decide_set_running("RUNNING", False, True)[:2], (True, True))
        self.assertEqual(decide_set_running("HOLDING", False, False)[:2], (True, True))

    def test_opposite_targets_are_rejected_during_transition(self):
        accepted, duplicate, _ = decide_set_running("STARTING", True, False)
        self.assertFalse(accepted)
        self.assertFalse(duplicate)
        accepted, duplicate, _ = decide_set_running("STOPPING", True, True)
        self.assertFalse(accepted)
        self.assertFalse(duplicate)

    def test_new_start_and_normal_stop_are_queued_once(self):
        self.assertEqual(decide_set_running("IDLE", False, True)[:2], (True, False))
        self.assertEqual(decide_set_running("HOLDING", False, True)[:2], (True, False))
        self.assertEqual(decide_set_running("RUNNING", False, False)[:2], (True, False))


if __name__ == "__main__":
    unittest.main()
