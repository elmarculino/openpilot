from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.gwm_lat_state import GwmLatTracker


class _Pm:
  def __init__(self):
    self.sent = []

  def send(self, name, msg):
    self.sent.append((name, msg))


def _car(eps_stock_values, override_active=False, mk4=True):
  c = Car.__new__(Car)
  c.gwm_lat = GwmLatTracker()
  c.gwm_pm = _Pm() if mk4 else None
  c.CI = SimpleNamespace(CS=SimpleNamespace(eps_stock_values=eps_stock_values),
                         CC=SimpleNamespace(override_active=override_active))
  return c


def _cs_cc(enabled=True, lat_active=True):
  CS = car.CarState.new_message(steeringTorque=40., steeringAngleDeg=10.)
  CC = car.CarControl.new_message(enabled=enabled, latActive=lat_active)
  CC.actuators.steeringAngleDeg = 12.5
  return CS, CC


def test_publishes_reason_and_angle_error():
  c = _car({"A_RX_STEER_REQUESTED": 2})
  CS, CC = _cs_cc()
  for _ in range(3):
    c.publish_gwm_lat_state(CS, CC)
  name, msg = c.gwm_pm.sent[-1]
  gs = msg.gwmLatState
  assert name == 'gwmLatState'
  assert (str(gs.reason), gs.frames, gs.aRx) == ("epsNotObeying", 3, 2)
  assert gs.angleErrorDeg == 2.5
  assert gs.driverTorque == 40.


def test_driver_override_wins():
  c = _car({"A_RX_STEER_REQUESTED": 2}, override_active=True)
  c.publish_gwm_lat_state(*_cs_cc())
  assert str(c.gwm_pm.sent[-1][1].gwmLatState.reason) == "driverOverride"


def test_before_the_first_eps_frame_reads_as_obeying():
  c = _car({})
  c.publish_gwm_lat_state(*_cs_cc())
  assert str(c.gwm_pm.sent[-1][1].gwmLatState.reason) == "none"


def test_publish_errors_never_escape():
  # runs in the actuation loop: an exception here would take carState down and disengage
  c = _car(None)
  c.publish_gwm_lat_state(*_cs_cc())
  assert c.gwm_pm.sent == []


def test_nothing_published_off_mk4():
  c = _car({"A_RX_STEER_REQUESTED": 2}, mk4=False)
  c.publish_gwm_lat_state(*_cs_cc())  # gwm_pm is None: returns without touching anything
