from openpilot.cereal import custom, log
from openpilot.selfdrive.car.gwm_lat_state import (
  REASONS,
  REASON_DRIVER_OVERRIDE,
  REASON_EPS_NOT_OBEYING,
  REASON_INACTIVE,
  REASON_NONE,
  REASON_STANDSTILL,
  GwmLatTracker,
  lat_block_reason,
)


def _reason(enabled=True, lat_active=True, standstill=False, override_active=False, a_rx=1):
  return lat_block_reason(enabled=enabled, lat_active=lat_active, standstill=standstill,
                          override_active=override_active, a_rx=a_rx)


def test_reasons_match_the_log_schema():
  # card assigns these strings straight into the capnp enum; a mismatch raises on the car mid-drive
  assert list(REASONS) == [str(e) for e in custom.GwmLatState.Reason.schema.enumerants]


def test_reason_priority():
  assert _reason() == REASON_NONE
  assert _reason(enabled=False, override_active=True, a_rx=2) == REASON_NONE
  assert _reason(lat_active=False, standstill=True) == REASON_STANDSTILL
  assert _reason(lat_active=False) == REASON_INACTIVE
  # the driver's hand explains a refusing EPS too, so it wins
  assert _reason(override_active=True, a_rx=2) == REASON_DRIVER_OVERRIDE
  assert _reason(a_rx=2) == REASON_EPS_NOT_OBEYING
  assert _reason(a_rx=0) == REASON_EPS_NOT_OBEYING


def test_tracker_counts_consecutive_frames_per_reason():
  t = GwmLatTracker()
  for _ in range(3):
    t.update(REASON_EPS_NOT_OBEYING)
  assert t.frames == 3
  t.update(REASON_DRIVER_OVERRIDE)  # a new reason restarts the count
  assert (t.reason, t.frames) == (REASON_DRIVER_OVERRIDE, 1)
  t.update(REASON_NONE)
  assert (t.reason, t.frames) == (REASON_NONE, 0)


def test_gwm_lat_state_round_trips_through_a_log():
  ev = log.Event.new_message()
  out = ev.init('gwmLatState')
  out.reason = REASON_EPS_NOT_OBEYING
  out.frames = 0xFFFF
  out.aRx = 2
  out.driverTorque = -42.5
  out.angleErrorDeg = 3.25
  with log.Event.from_bytes(ev.to_bytes()) as back:
    assert str(back.gwmLatState.reason) == REASON_EPS_NOT_OBEYING
    assert back.gwmLatState.aRx == 2
    assert back.gwmLatState.angleErrorDeg == 3.25
