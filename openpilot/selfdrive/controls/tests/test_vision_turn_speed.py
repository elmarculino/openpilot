import math

import numpy as np
from types import SimpleNamespace

from openpilot.cereal import custom, log
from openpilot.selfdrive.controls.lib.vision_turn_speed import (
  MIN_V,
  SmartCruiseControlVision,
  VisionState,
  V_TARGET_UNSET,
  _ENTERING_PRED_LAT_ACC_TH,
  _TURNING_LAT_ACC_TH,
  _A_LAT_REG_MAX,
  _IN_CURVE_MIN_ACC_V,
  _TURNING_ACC_V,
  _A_TARGET_STEP_DOWN,
  _A_TARGET_STEP_UP,
)

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource

# m/s, not km/h: longitudinal_planner converts vCruise before handing it to SCC-V, and
# V_TARGET_UNSET is inf precisely so a unit mix-up cannot hide inside min() (PR #2 review).
V_CRUISE_MS = 30.0


def _sm(pred_lat_acc: float, n: int = 33, curvature: float = 0.0):
  z = [pred_lat_acc] * n
  x = [1.0] * n
  return {
    "modelV2": SimpleNamespace(
      orientationRate=SimpleNamespace(z=z),
      velocity=SimpleNamespace(x=x),
    ),
    "controlsState": SimpleNamespace(curvature=curvature),
  }


def _empty_sm():
  return {
    "modelV2": SimpleNamespace(
      orientationRate=SimpleNamespace(z=[]),
      velocity=SimpleNamespace(x=[]),
    ),
    "controlsState": SimpleNamespace(curvature=0.0),
  }


def test_disabled_until_long_active():
  scc = SmartCruiseControlVision(enabled=True)
  scc.update(_sm(2.0), False, False, MIN_V + 5, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.disabled
  assert not scc.is_active
  assert scc.output_v_target == V_TARGET_UNSET


def test_stays_disabled_when_toggle_off():
  scc = SmartCruiseControlVision(enabled=False)
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  for _ in range(3):
    scc.update(sm, True, False, MIN_V + 5, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.disabled
  assert not scc.is_active


def test_enters_on_predicted_lat_acc():
  scc = SmartCruiseControlVision(enabled=True)
  v_ego = MIN_V + 5
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.enabled
  scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.entering
  assert scc.is_active
  assert scc.output_v_target < V_CRUISE_MS
  assert scc.output_a_target < 0.0


def test_drops_when_long_goes_inactive():
  scc = SmartCruiseControlVision(enabled=True)
  v_ego = MIN_V + 5
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.is_active
  scc.update(sm, False, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.disabled
  assert not scc.is_active
  assert scc.output_v_target == V_TARGET_UNSET


def test_long_override_suspends_and_resumes():
  scc = SmartCruiseControlVision(enabled=True)
  v_ego = MIN_V + 5
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  scc.update(sm, True, True, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.overriding
  assert not scc.is_active
  scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.enabled


def test_empty_model_path_does_not_raise():
  scc = SmartCruiseControlVision(enabled=True)
  scc.update(_empty_sm(), True, False, MIN_V + 5, 0.0, V_CRUISE_MS)
  assert scc.max_pred_lat_acc == 0.0


def test_missing_model_fields_do_not_raise():
  scc = SmartCruiseControlVision(enabled=True)
  scc.update({}, True, False, MIN_V + 5, 0.0, V_CRUISE_MS)
  assert scc.max_pred_lat_acc == 0.0


def _drive_into_turn(scc, v_ego):
  # enabled -> entering -> turning
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5, curvature=0.02)
  for _ in range(3):
    scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.turning
  assert scc.current_lat_acc >= _TURNING_LAT_ACC_TH


def test_model_dropout_mid_turn_releases_decel():
  # a dropout must not latch `turning` forever: current_lat_acc has to be reset alongside
  # max_pred_lat_acc, otherwise the stale value never satisfies the leaving threshold.
  v_ego = MIN_V + 5
  scc = SmartCruiseControlVision(enabled=True)
  _drive_into_turn(scc, v_ego)

  scc.update({}, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.current_lat_acc == 0.0
  assert scc.state == VisionState.leaving
  assert scc.output_a_target >= 0.0

  scc.update({}, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.enabled
  assert not scc.is_active
  assert scc.output_v_target == V_TARGET_UNSET


def test_empty_model_path_mid_turn_releases_decel():
  v_ego = MIN_V + 5
  scc = SmartCruiseControlVision(enabled=True)
  _drive_into_turn(scc, v_ego)

  scc.update(_empty_sm(), True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.current_lat_acc == 0.0
  assert scc.state == VisionState.leaving
  assert scc.output_a_target >= 0.0


# --- logging (README items 6 and 8) -------------------------------------------------------------
# The whole point of these fields is that a single route is enough to tune the thresholds above.
# They are published by LongitudinalPlanner.publish(); building a planner needs acados, so these
# exercise the schema and the Python->capnp coupling instead, which is where the breakage lives.

def test_turn_speed_is_its_own_plan_source():
  # item 6: SCC-V braking used to be labelled `cruise`, making it indistinguishable in a route
  names = [str(e) for e in LongitudinalPlanSource.schema.enumerants]
  assert "turnSpeed" in names
  # appended, never renumbered: an existing ordinal would silently remap old routes
  assert names.index("cruise") == 0 and names.index("e2e") == 4


def test_sccv_debug_fields_exist():
  # item 8: the state machine was invisible in a route
  fields = custom.SmartCruiseControlVisionState.schema.fieldnames
  assert set(fields) == {"state", "currentLatAcc", "maxPredLatAcc", "vTarget", "aTarget"}


def test_every_vision_state_is_loggable():
  # publish() assigns VisionState(...).name straight into the capnp enum, so a state added to the
  # IntEnum without a matching enumerant raises on the car, in plannerd, mid-drive.
  loggable = {str(e) for e in custom.SmartCruiseControlVisionState.State.schema.enumerants}
  assert {s.name for s in VisionState} == loggable
  for state in VisionState:
    assert custom.SmartCruiseControlVisionState.State.schema.enumerants[state.name] == state.value


def test_sccv_state_round_trips_through_a_log():
  scc = SmartCruiseControlVision(enabled=True)
  _drive_into_turn(scc, MIN_V + 5)

  ev = log.Event.new_message()
  sccv_out = ev.init('sccvState')
  sccv_out.state = VisionState(scc.state).name
  sccv_out.currentLatAcc = float(scc.current_lat_acc)
  sccv_out.maxPredLatAcc = float(scc.max_pred_lat_acc)
  sccv_out.vTarget = float(scc.v_target)
  sccv_out.aTarget = float(scc.a_target)

  with log.Event.from_bytes(ev.to_bytes()) as read_back:
    sccv = read_back.sccvState
    assert str(sccv.state) == "turning"
    assert sccv.currentLatAcc >= _TURNING_LAT_ACC_TH
    # Float32 round-trip, so compare against the source values rather than pinning magnitudes:
    # in `turning` the commanded accel only goes negative past _TURNING_ACC_BP[1].
    # 1e-6 is just outside float32's ~1.2e-7 relative precision
    assert math.isclose(sccv.currentLatAcc, scc.current_lat_acc, rel_tol=1e-6)
    assert math.isclose(sccv.maxPredLatAcc, scc.max_pred_lat_acc, rel_tol=1e-6)
    assert math.isclose(sccv.vTarget, scc.v_target, rel_tol=1e-6)
    assert math.isclose(sccv.aTarget, scc.a_target, rel_tol=1e-6)


def test_curve_solution_is_si():
  # The reviewer's units concern, pinned on the arithmetic instead of the call site:
  # v_target = sqrt(_A_LAT_REG_MAX / max_curve) and max_curve = max_pred_lat_acc / v_ego^2, all SI.
  # _sm() feeds velocity.x = 1.0, so max_pred_lat_acc is the orientationRate value verbatim.
  v_ego = MIN_V + 5
  scc = SmartCruiseControlVision(enabled=True)
  _drive_into_turn(scc, v_ego)
  expected = math.sqrt(_A_LAT_REG_MAX * v_ego ** 2 / (_ENTERING_PRED_LAT_ACC_TH + 0.5))
  assert math.isclose(scc.v_target, expected, rel_tol=1e-9)
  # and it is a road speed in m/s, not a km/h number that slipped through
  assert MIN_V < scc.v_target < V_CRUISE_MS


def test_output_v_target_never_lands_under_the_entry_gate():
  # The no-overshoot term is a decel, so `max(v_target, MIN_V) + a_target * 4 s` used to fall
  # ~4 m/s below the floor -- under the 20 km/h gate the state machine itself respects
  # (PR #2 review). Worst case: active, v_target already at the floor, hardest decel.
  scc = SmartCruiseControlVision(enabled=True)
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)  # straight road, curve ahead: the cap applies
  for _ in range(2):
    scc.update(sm, True, False, MIN_V + 5, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.entering
  scc.v_target = 0.0
  scc.a_target = -1.0
  assert scc.get_v_target_from_control() == MIN_V


V_HWY = 25.0


def _step(scc, pred, lat=0.0, n=1, v_ego=V_HWY, a_ego=0.0):
  sm = _sm(pred, curvature=lat / v_ego ** 2)
  for _ in range(n):
    scc.update(sm, True, False, v_ego, a_ego, V_CRUISE_MS)


def test_speed_cap_before_the_curve_not_in_it():
  # route 1c5: the cap stayed on inside the curve and held the cruise accel at -1.2 m/s^2
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 2.0, n=2)
  assert scc.state == VisionState.entering
  assert scc.output_v_target < V_HWY  # straight, curve ahead: slow down
  _step(scc, 2.0, lat=0.7)
  assert scc.state == VisionState.entering
  assert scc.output_v_target == V_TARGET_UNSET
  _step(scc, 2.0, lat=2.0, n=3)
  assert scc.state == VisionState.turning
  assert scc.output_v_target == V_TARGET_UNSET


def test_in_curve_decel_is_gentle():
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 1.6, lat=1.0, n=100)
  assert scc.state == VisionState.entering and scc.in_curve
  assert _IN_CURVE_MIN_ACC_V[0] <= scc.output_a_target < 0.


def test_sharper_curve_ahead_keeps_the_approach_response():
  # review round 1: a shallow bend before a tightening curve must not lose the pre-curve braking
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.0, lat=0.61, n=100)
  assert scc.in_curve
  assert scc.output_a_target == -1.0


def test_tighter_curve_under_a_fixed_margin_still_brakes():
  # review round 2: lat 2.2 with 3.39 predicted (curve speed ~19 m/s at 25) settled at +0.06
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.39, lat=2.2, n=100)
  assert scc.state == VisionState.turning
  assert scc.output_a_target < -0.9


def test_turning_never_accelerates_toward_a_sharper_curve():
  scc = SmartCruiseControlVision(enabled=True)
  for pred in (2.3, 2.4, 2.6, 2.9, 3.2):  # lat 1.7: from 0.6 m/s^2 sharper ahead on
    _step(scc, pred, lat=1.7, n=100)
    assert scc.state == VisionState.turning
    assert scc.output_a_target <= 1e-9
  _step(scc, 3.0, lat=1.2, n=100)  # leaving, sharper bend next
  assert scc.state == VisionState.leaving
  assert scc.output_a_target == -1.0


def test_response_is_continuous_in_the_prediction():
  # review round 2: a prediction dithering around one value must not flip braking on and off
  for lat in (0.7, 1.7, 2.5):
    prev = None
    for pred in np.arange(lat, lat + 2.0, 0.01):
      scc = SmartCruiseControlVision(enabled=True)
      _step(scc, max(pred, 1.6), lat=lat, n=200)
      if prev is not None:
        assert abs(scc.output_a_target - prev) < 0.06
      prev = scc.output_a_target


def test_reacts_without_waiting_for_the_slew_up():
  # review round 2: from +0.375 in `turning`, a sharper curve appearing must not keep a positive request
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 2.0, lat=1.7, n=100)
  start = scc.output_a_target
  _step(scc, 3.0, lat=1.7)
  assert scc.output_a_target == start - _A_TARGET_STEP_DOWN
  _step(scc, 3.0, lat=1.7, n=int(start / _A_TARGET_STEP_DOWN) + 1)
  assert scc.output_a_target <= 0.


def test_high_lat_acc_backstop():
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.5, lat=3.5, n=100)
  assert scc.state == VisionState.turning
  assert scc.output_a_target == _TURNING_ACC_V[-1]


def test_backstop_reachable_with_a_low_prediction():
  # review round 2: deep in a sharp bend while the model already predicts its end
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 1.4, lat=3.5, n=100)
  assert scc.state == VisionState.turning
  assert scc.output_a_target == _TURNING_ACC_V[-1]


def test_leaving_into_the_next_curve_approaches_it():
  # review round 2: leaving -> enabled -> entering dropped the candidate for a frame
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.0, lat=1.2, n=5)
  _step(scc, 3.0, lat=1.7, n=5)
  _step(scc, 3.0, lat=1.2, n=5)
  assert scc.state == VisionState.leaving
  _step(scc, 3.0, lat=0.3)
  assert scc.state == VisionState.entering
  assert scc.has_candidate


def test_candidate_is_released_not_dropped():
  # review round 2: a settled -1.0 must not vanish when SCC-V goes inactive (here: model dropout)
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.5, lat=3.5, n=100)
  assert scc.output_a_target == -1.0
  prev = scc.output_a_target
  for _ in range(100):
    scc.update({}, True, False, V_HWY, 0.0, V_CRUISE_MS)
    if not scc.has_candidate:
      break
    assert 0. <= scc.output_a_target - prev <= _A_TARGET_STEP_UP + 1e-9
    prev = scc.output_a_target
  assert not scc.has_candidate
  assert prev >= 0.5 - _A_TARGET_STEP_UP


def test_in_curve_has_hysteresis():
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 1.8, lat=0.7, n=5)
  for k in range(20):
    _step(scc, 1.8, lat=0.59 if k % 2 else 0.61)
    assert scc.in_curve


def test_candidate_is_rate_limited():
  scc = SmartCruiseControlVision(enabled=True)
  prev = None
  for k in range(400):
    lat = [0.0, 0.61, 1.7, 0.3, 3.5][(k // 40) % 5]
    _step(scc, 3.0 if k % 7 else 1.0, lat=lat, a_ego=1.5 if k % 3 else -1.5)
    if scc.has_candidate and prev is not None:
      assert -_A_TARGET_STEP_DOWN - 1e-9 <= scc.output_a_target - prev <= _A_TARGET_STEP_UP + 1e-9
    prev = scc.output_a_target if scc.has_candidate else None


def test_gentle_bend_does_not_trigger():
  # route 1c5: predictions hovering at 1.1-1.3 flipped SCC-V on and off every 2-3 s
  scc = SmartCruiseControlVision(enabled=True)
  sm = _sm(1.4)
  for _ in range(10):
    scc.update(sm, True, False, 25.0, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.enabled
