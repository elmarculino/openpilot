from openpilot.selfdrive.selfdrived.mads_h6 import H6Mads


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
