from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.mads_h6 import H6Mads, MISMATCH_FRAMES

SafetyModel = car.CarParams.SafetyModel


def test_gentle_from_off_is_lat_only():
  m = H6Mads()
  want_enable, want_cancel, override = m.update(engaged=False, user_brake=False, lkas_tap=True, acc_enable=False)
  assert want_enable
  assert not want_cancel
  assert not m.long_enabled
  assert override  # same frame as enable so longActive never pulses
  want_enable, want_cancel, override = m.update(engaged=True, user_brake=False, lkas_tap=False, acc_enable=False)
  assert not want_enable
  assert not want_cancel
  assert override


def test_gentle_from_lat_only_cancels():
  m = H6Mads()
  m.update(engaged=False, user_brake=False, lkas_tap=True, acc_enable=False)
  want_enable, want_cancel, override = m.update(engaged=True, user_brake=False, lkas_tap=True, acc_enable=False)
  assert want_cancel
  assert not want_enable
  assert not override


def test_gentle_from_both_drops_acc():
  m = H6Mads()
  m.update(engaged=False, user_brake=False, lkas_tap=False, acc_enable=True)
  assert m.long_enabled
  want_enable, want_cancel, override = m.update(engaged=True, user_brake=False, lkas_tap=True, acc_enable=False)
  assert not want_cancel
  assert not m.long_enabled
  assert override


def test_detent_engages_both():
  m = H6Mads()
  want_enable, want_cancel, override = m.update(engaged=False, user_brake=False, lkas_tap=False, acc_enable=True)
  assert want_enable
  assert not want_cancel
  assert m.long_enabled
  assert not override
  _, _, override = m.update(engaged=True, user_brake=False, lkas_tap=False, acc_enable=False)
  assert not override


def test_detent_from_lat_only_adds_acc():
  m = H6Mads()
  m.update(engaged=False, user_brake=False, lkas_tap=True, acc_enable=False)
  want_enable, want_cancel, override = m.update(engaged=True, user_brake=False, lkas_tap=False, acc_enable=True)
  assert m.long_enabled
  assert not want_cancel
  assert not override


def test_brake_drops_acc_keeps_lat():
  m = H6Mads()
  m.update(engaged=False, user_brake=False, lkas_tap=False, acc_enable=True)
  want_enable, want_cancel, override = m.update(engaged=True, user_brake=True, lkas_tap=False, acc_enable=False)
  assert not m.long_enabled
  assert not want_cancel
  assert override


def test_brake_while_lat_only_stays_lat():
  m = H6Mads()
  m.update(engaged=False, user_brake=False, lkas_tap=True, acc_enable=False)
  _, want_cancel, override = m.update(engaged=True, user_brake=True, lkas_tap=False, acc_enable=False)
  assert not want_cancel
  assert override


def test_reset_clears_acc():
  m = H6Mads()
  m.update(engaged=False, user_brake=False, lkas_tap=False, acc_enable=True)
  m.reset()
  assert not m.long_enabled


def test_wheel_does_not_enable_acc_without_acc_enable():
  m = H6Mads()
  m.update(engaged=False, user_brake=False, lkas_tap=True, acc_enable=False)
  _, _, override = m.update(engaged=True, user_brake=False, lkas_tap=False, acc_enable=False)
  assert override
  assert not m.long_enabled


def test_same_cycle_brake_and_detent_is_lat_only():
  # Braking while moving and pressing the stalk detent in the same cycle: the brake wins, so
  # ENABLE is raised together with the long override and state.py enters State.overriding
  # directly. Without this the FSM would spend a cycle in State.enabled believing long is live.
  m = H6Mads()
  want_enable, want_cancel, override = m.update(engaged=False, user_brake=True, lkas_tap=False, acc_enable=True)
  assert want_enable
  assert not want_cancel
  assert override
  assert not m.long_enabled


def test_detent_without_brake_still_takes_long():
  m = H6Mads()
  want_enable, _, override = m.update(engaged=False, user_brake=False, lkas_tap=False, acc_enable=True)
  assert want_enable
  assert not override
  assert m.long_enabled


def test_brake_release_after_engaging_under_brake_does_not_resume_long():
  # long stays off until the driver asks for it again -- release alone must not silently resume ACC
  m = H6Mads()
  m.update(engaged=False, user_brake=True, lkas_tap=False, acc_enable=True)
  _, _, override = m.update(engaged=True, user_brake=False, lkas_tap=False, acc_enable=False)
  assert override
  assert not m.long_enabled


def _panda(controls_allowed, safety_model=SafetyModel.gwm):
  return SimpleNamespace(controlsAllowed=controls_allowed, safetyModel=safety_model)


def _run(m, panda_states, frames, engaged=True):
  for _ in range(frames):
    m.data_sample(panda_states, engaged)


def test_mismatch_trips_after_half_a_second():
  m = H6Mads()
  _run(m, [_panda(False)], MISMATCH_FRAMES - 1)
  assert not m.mismatch
  m.data_sample([_panda(False)], True)
  assert m.mismatch


def test_mismatch_tolerates_panda_state_skew():
  # pandaStates is a 10 Hz socket: a stale pre-arm sample for a few frames must not trip
  m = H6Mads()
  _run(m, [_panda(False)], 20)
  assert not m.mismatch
  _run(m, [_panda(True)], 5)
  assert not m.mismatch
  assert m.mismatch_counter == 0


def test_mismatch_counter_resets_when_panda_agrees():
  m = H6Mads()
  _run(m, [_panda(False)], MISMATCH_FRAMES - 1)
  m.data_sample([_panda(True)], True)
  assert m.mismatch_counter == 0
  _run(m, [_panda(False)], MISMATCH_FRAMES - 1)
  assert not m.mismatch


def test_no_mismatch_while_disengaged():
  m = H6Mads()
  _run(m, [_panda(False)], MISMATCH_FRAMES * 3, engaged=False)
  assert not m.mismatch
  assert m.mismatch_counter == 0


def test_mismatch_ignores_silent_pandas():
  # nothing is expected to allow controls in silent/noOutput, so this is not a disagreement
  m = H6Mads()
  _run(m, [_panda(False, SafetyModel.silent), _panda(False, SafetyModel.noOutput)], MISMATCH_FRAMES * 2)
  assert not m.mismatch


def test_mismatch_when_any_panda_allows():
  m = H6Mads()
  _run(m, [_panda(False, SafetyModel.silent), _panda(True)], MISMATCH_FRAMES * 2)
  assert not m.mismatch


def test_mismatch_latches_until_disengage():
  m = H6Mads()
  _run(m, [_panda(False)], MISMATCH_FRAMES)
  assert m.mismatch
  _run(m, [_panda(True)], 10)
  assert m.mismatch  # IMMEDIATE_DISABLE lands within a frame; do not un-flag on a late agreement
  m.data_sample([_panda(True)], False)
  assert not m.mismatch


def test_reset_clears_mismatch():
  m = H6Mads()
  _run(m, [_panda(False)], MISMATCH_FRAMES)
  assert m.mismatch
  m.reset()
  assert not m.mismatch
  assert m.mismatch_counter == 0


def test_route_000000fd_phantom_engage_is_caught():
  # the real failure: 1.4 s engaged with controls_allowed=0 the whole time. Upstream's 200-frame
  # counter never reached its threshold; this one trips with ~0.9 s to spare.
  m = H6Mads()
  frames = int(1.4 / DT_CTRL)
  assert frames < 200
  _run(m, [_panda(False)], frames)
  assert m.mismatch
