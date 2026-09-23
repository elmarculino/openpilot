"""MADS-lite set-speed initialization (card.py -> VCruiseHelper.should_initialize).

Lat-only keeps carControl.enabled true, so the upstream "initialize on the enabled rising edge" never
fires when a detent press later adds ACC. These drive the helper the way card.py does, frame by frame,
without acados (test_cruise_speed.py pulls in the long MPC).
"""
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, LONG_ENGAGE_WINDOW

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def _cp(brand="gwm", pcm_cruise=False):
  return car.CarParams(brand=brand, pcmCruise=pcm_cruise)


class CardSim:
  """The two lines of card.state_update that touch the set speed, plus the CC/CS_prev bookkeeping."""

  def __init__(self, CP):
    self.h = VCruiseHelper(CP)
    self.enabled_prev = False
    self.long_prev = False
    self.CS_prev = car.CarState()

  def step(self, kph, enabled, long_active, buttons=()):
    CS = car.CarState(vEgo=kph * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=t, pressed=p) for t, p in buttons]
    self.h.update_v_cruise(CS, enabled, is_metric=True)
    if self.h.should_initialize(CS, enabled, self.enabled_prev, long_active, self.long_prev):
      self.h.initialize_v_cruise(self.CS_prev, False)
    self.enabled_prev, self.long_prev, self.CS_prev = enabled, long_active, CS
    return self.h.v_cruise_kph


def _lat_only_then_detent(sim, lag=2, lat_kph=30):
  # gentle DOWN at lat_kph: enabled rises, longActive stays false
  sim.step(lat_kph, False, False)
  assert sim.step(lat_kph, True, False) == lat_kph
  # driver accelerates by foot to 100 km/h, steering only
  for _ in range(300):
    sim.step(100, True, False)
  # detent: setCruise press reaches card, longActive comes back `lag` frames later
  sim.step(100, True, False, [(ButtonType.setCruise, True)])
  for _ in range(lag - 1):
    sim.step(100, True, False)
  return sim.step(100, True, True)


def test_detent_from_lat_only_uses_current_speed():
  assert _lat_only_then_detent(CardSim(_cp())) == 100


def test_detent_with_a_slow_round_trip_still_initializes():
  assert _lat_only_then_detent(CardSim(_cp()), lag=LONG_ENGAGE_WINDOW - 1) == 100


def test_unrelated_long_edge_after_the_window_does_not_reinitialize():
  assert _lat_only_then_detent(CardSim(_cp()), lag=LONG_ENGAGE_WINDOW + 5) == 30


def test_gas_override_end_keeps_the_set_speed():
  # ACC live at 80, driver overtakes on the gas to 110 and lifts: longActive rises again with no
  # setCruise press, so the set speed must stay 80 -- this is the normal "overtake, lift, resume".
  sim = CardSim(_cp())
  sim.step(80, False, False, [(ButtonType.setCruise, True)])
  assert sim.step(80, True, True) == 80
  for _ in range(100):
    sim.step(110, True, False)
  assert sim.step(110, True, True) == 80


def test_detent_while_long_already_active_does_not_reinitialize():
  sim = CardSim(_cp())
  sim.step(80, False, False, [(ButtonType.setCruise, True)])
  assert sim.step(80, True, True) == 80
  sim.step(95, True, True, [(ButtonType.setCruise, True)])
  assert sim.step(95, True, True) == 80


def test_press_then_disengage_does_not_carry_over():
  # a press that lands while openpilot is going down must not arm a later, unrelated long edge
  sim = CardSim(_cp())
  sim.step(30, False, False)
  sim.step(30, True, False)
  sim.step(60, True, False, [(ButtonType.setCruise, True)])
  sim.step(70, False, False)
  assert sim.step(70, True, False) == 70  # gentle DOWN again: enabled edge -> init 70
  for _ in range(20):
    sim.step(90, True, False)
  # a longActive edge with no press of its own must not be credited to the stale one
  assert sim.step(90, True, True) == 70


def test_other_brands_keep_upstream_behaviour():
  # without MADS-lite a longActive edge with enabled already high is always an override ending
  # (50 km/h: non-GWM brands floor the initial set speed at V_CRUISE_INITIAL = 40)
  sim = CardSim(_cp(brand="toyota"))
  assert _lat_only_then_detent(sim, lat_kph=50) == 50
