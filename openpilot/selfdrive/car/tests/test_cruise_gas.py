"""Set the max speed with the gas pedal (GWM, openpilot long): release the pedal above the set speed and
the set speed becomes the speed you released at. Drives VCruiseHelper the way card.py does."""
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, GAS_SET_SPEED_MARGIN, V_CRUISE_MAX

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def _cp(brand="gwm", pcm_cruise=False):
  return car.CarParams(brand=brand, pcmCruise=pcm_cruise)


class CardSim:
  def __init__(self, CP):
    self.h = VCruiseHelper(CP)
    self.enabled_prev = False
    self.long_prev = False
    self.CS_prev = car.CarState()

  def step(self, kph, enabled, long_active, gas=False, n=1, buttons=()):
    for _ in range(n):
      CS = car.CarState(vEgo=kph * CV.KPH_TO_MS, gasPressed=gas, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=t, pressed=p) for t, p in buttons]
      self.h.update_v_cruise(CS, enabled, is_metric=True)
      self.h.update_gas_set_speed(CS, enabled, long_active, self.long_prev)
      if self.h.should_initialize(CS, enabled, self.enabled_prev, long_active, self.long_prev):
        self.h.initialize_v_cruise(self.CS_prev, False)
      self.enabled_prev, self.long_prev, self.CS_prev = enabled, long_active, CS
    return self.h.v_cruise_kph


def _engaged_at(sim, kph):
  sim.step(kph, False, False)
  return sim.step(kph, True, True, n=10)


def _press_gas(sim, kph):
  sim.step(kph, True, True, gas=True)  # first frame: carControl still has longActive
  return sim.step(kph, True, False, gas=True, n=50)


def test_release_above_set_speed_raises_it():
  sim = CardSim(_cp())
  assert _engaged_at(sim, 80) == 80
  _press_gas(sim, 97.4)
  assert sim.step(97.4, True, False) == 97  # pedal up
  assert sim.step(97.4, True, True, n=10) == 97


def test_set_speed_unchanged_while_the_pedal_is_down():
  sim = CardSim(_cp())
  _engaged_at(sim, 80)
  assert _press_gas(sim, 110) == 80


def test_small_overshoot_keeps_the_set_speed():
  sim = CardSim(_cp())
  _engaged_at(sim, 80)
  _press_gas(sim, 80 + GAS_SET_SPEED_MARGIN)
  assert sim.step(80 + GAS_SET_SPEED_MARGIN, True, False) == 80


def test_never_lowers_the_set_speed():
  sim = CardSim(_cp())
  _engaged_at(sim, 100)
  sim.step(100, True, True, buttons=[(ButtonType.accelCruise, True)])
  sim.step(100, True, True, buttons=[(ButtonType.accelCruise, False)])
  _press_gas(sim, 60)
  assert sim.step(60, True, False) == 101


def test_clipped_to_max():
  sim = CardSim(_cp())
  _engaged_at(sim, 120)
  _press_gas(sim, 170)
  assert sim.step(170, True, False) == V_CRUISE_MAX


def test_gas_in_lat_only_does_nothing():
  # MADS lat-only: no ACC to override, the set speed waits for the detent
  sim = CardSim(_cp())
  _engaged_at(sim, 80)
  sim.step(80, True, False, n=50)  # brake: ACC off, lateral stays
  sim.step(110, True, False, gas=True, n=50)
  assert sim.step(110, True, False) == 80


def test_disengage_on_the_gas_does_nothing():
  sim = CardSim(_cp())
  _engaged_at(sim, 80)
  _press_gas(sim, 110)
  sim.step(110, False, False, gas=True)
  assert sim.step(110, False, False) == 80


def test_other_brands_unchanged():
  sim = CardSim(_cp(brand="toyota"))
  _engaged_at(sim, 80)
  _press_gas(sim, 110)
  assert sim.step(110, True, False) == 80
