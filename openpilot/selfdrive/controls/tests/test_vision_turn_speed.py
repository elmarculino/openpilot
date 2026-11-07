import math

import numpy as np
from types import SimpleNamespace

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.cereal import custom, log
from openpilot.selfdrive.controls.lib.vision_turn_speed import (
  MIN_V,
  SmartCruiseControlVision,
  VisionState,
  V_TARGET_UNSET,
  _ENTERING_PRED_LAT_ACC_TH,
  _TURNING_LAT_ACC_TH,
  _A_LAT_REG_MAX,
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
  # model plan at constant speed: model lat acc == lat acc at today's speed == pred
  sm = {"modelV2": SimpleNamespace(orientationRate=SimpleNamespace(z=[pred / v_ego] * 33),
                                   velocity=SimpleNamespace(x=[v_ego] * 33)),
        "controlsState": SimpleNamespace(curvature=lat / v_ego ** 2)}
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


def test_gentle_curve_has_no_standing_decel():
  # Fable/Sol votes: `entering` inside a long sweeper held -0.2..-0.3 with nothing sharper ahead
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 1.6, lat=1.4, n=100)
  assert scc.state == VisionState.entering and scc.in_curve
  assert scc.output_a_target >= 0.


def test_constant_radius_with_predicted_speed_up_does_not_brake():
  # Sol vote: v_ego 30, lat 1.2, model plan at 35 m/s -> model lat acc 1.63 enters SCC-V
  v_ego = 30.0
  k = 1.2 / v_ego ** 2
  scc = SmartCruiseControlVision(enabled=True)
  for _ in range(100):
    scc.update(_sm_curve(k, k, 35.0), True, False, v_ego, 0.0, V_CRUISE_MS)
  assert scc.state == VisionState.entering
  assert scc.output_a_target >= 0.


def test_sharper_curve_ahead_keeps_the_approach_response():
  # review round 1: a shallow bend before a tightening curve must not lose the pre-curve braking
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.0, lat=0.61, n=100)
  assert scc.in_curve
  assert math.isclose(scc.output_a_target, -1.0)


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
  assert math.isclose(scc.output_a_target, -1.0)


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
  assert math.isclose(scc.output_a_target, -1.0)
  prev = scc.output_a_target
  for _ in range(100):
    scc.update({}, True, False, V_HWY, 0.0, V_CRUISE_MS)
    if not scc.has_candidate:
      break
    assert 0. <= scc.output_a_target - prev <= _A_TARGET_STEP_UP + 1e-9
    prev = scc.output_a_target
  assert not scc.has_candidate
  assert prev >= ACCEL_MAX - _A_TARGET_STEP_UP  # released all the way: removal cannot move the plan


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
    a_ego = 1.5 if k % 3 else -1.5
    was_releasing = scc.releasing
    _step(scc, 3.0 if k % 7 else 1.0, lat=lat, a_ego=a_ego)
    straight = scc.state == VisionState.entering and not scc.in_curve  # unslewed by design (S1)
    if scc.has_candidate and prev is not None and not straight:
      if was_releasing and scc.is_active:
        # mid-release restart: may drop to the car's accel (the released target was not binding)
        assert scc.output_a_target >= min(prev, a_ego) - _A_TARGET_STEP_DOWN - 1e-9
      else:
        assert -_A_TARGET_STEP_DOWN - 1e-9 <= scc.output_a_target - prev <= _A_TARGET_STEP_UP + 1e-9
    prev = scc.output_a_target if scc.has_candidate else None


def test_gentle_bend_flicker_never_brakes():
  # route 1c5: in a long gentle bend the prediction hovered at 1.1-1.4 and SCC-V flipped on and off,
  # alternating -0.2 and +0.4; inside a curve with nothing sharper ahead it must not brake at all
  scc = SmartCruiseControlVision(enabled=True)
  for k in range(200):
    _step(scc, 1.4 if (k // 20) % 2 else 1.2, lat=1.0)
    if scc.has_candidate:
      assert scc.output_a_target >= 0.

def test_sharper_bend_hidden_by_a_predicted_slowdown():
  # review round 3: lat 2.2 at 25 m/s, bend ahead 50% tighter (3.3 at today's speed) but reached at
  # 18 m/s in the model's plan, so rate * vel is only 1.71 -- must still brake
  v_ego, vel_ahead = V_HWY, 18.0
  k_now = 2.2 / v_ego ** 2
  rate = 1.5 * k_now * vel_ahead
  sm = {"modelV2": SimpleNamespace(orientationRate=SimpleNamespace(z=[rate] * 33),
                                   velocity=SimpleNamespace(x=[vel_ahead] * 33)),
        "controlsState": SimpleNamespace(curvature=k_now)}
  scc = SmartCruiseControlVision(enabled=True)
  for _ in range(100):
    scc.update(sm, True, False, v_ego, 0.0, V_CRUISE_MS)
  assert abs(scc.max_pred_lat_acc - 1.71) < 0.01
  assert abs(scc.pred_lat_acc_at_v_ego - 3.3) < 0.01
  assert scc.output_a_target < -0.8  # 1.1 sharper at today's speed: most of the full -1.0 approach


def test_leaving_into_a_curve_respects_min_v():
  # review round 3: leaving -> entering skipped the 20 km/h gate
  v = MIN_V - 0.5
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.0, lat=1.7, n=5, v_ego=MIN_V + 5)
  _step(scc, 3.0, lat=1.2, n=5, v_ego=MIN_V + 5)
  assert scc.state == VisionState.leaving
  _step(scc, 3.0, lat=0.3, v_ego=v)
  assert scc.state == VisionState.enabled


def test_activation_does_not_prolong_harder_braking():
  # review round 3: car at -2.0 (lead) entering a gentle approach: start at SCC-V's own solution
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 1.6, n=1, a_ego=-2.0)  # disabled -> enabled
  _step(scc, 1.6, n=1, a_ego=-2.0)  # enabled -> entering
  assert scc.state == VisionState.entering
  assert scc.output_a_target > -0.4


def test_straight_approach_brakes_at_once_while_accelerating():
  # sol-loop S1: the straight approach is the driver-approved path and must not be slewed
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.0, n=2, a_ego=0.5)
  assert scc.state == VisionState.entering and not scc.in_curve
  assert scc.output_a_target == -1.0
  assert scc.output_v_target < V_HWY - 4 * 0.9  # the cap keeps its full no-overshoot term


def test_straight_entry_threshold_unchanged():
  # sol-loop S2: a 1.4 prediction on a straight still activates, as in sunnypilot
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 1.4, n=2)
  assert scc.state == VisionState.entering
  assert scc.output_a_target < -0.2


def test_mid_release_restart_keeps_a_binding_target():
  # sol-loop S3: released target -0.4 was what the planner did; the car lags at -0.8
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.5, lat=3.5, n=100)
  while scc.output_a_target < -0.4 - 1e-9:
    scc.update({}, True, False, V_HWY, -0.8, V_CRUISE_MS)
  assert scc.releasing
  released = scc.output_a_target
  sm = {"modelV2": SimpleNamespace(orientationRate=SimpleNamespace(z=[3.0 / V_HWY] * 33),
                                   velocity=SimpleNamespace(x=[V_HWY] * 33)),
        "controlsState": SimpleNamespace(curvature=0.7 / V_HWY ** 2)}
  scc.update(sm, True, False, V_HWY, -0.8, V_CRUISE_MS, a_plan_prev=released)
  assert scc.is_active and scc.in_curve
  assert math.isclose(scc.output_a_target, released - _A_TARGET_STEP_DOWN)


def test_in_curve_latch_tracks_while_long_is_off():
  # review round 3: override on a straight, bend during the override, re-enable at 0.5 (inside the
  # hysteresis band): the latch must reflect the bend, not the straight
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 0.5, n=5)
  assert not scc.in_curve
  for _ in range(20):
    scc.update(_sm(2.0, curvature=0.8 / V_HWY ** 2), False, False, V_HWY, 0.0, V_CRUISE_MS)
  assert scc.in_curve
  _step(scc, 2.0, lat=0.5, n=3)
  assert scc.in_curve
  assert scc.output_v_target == V_TARGET_UNSET


def _sm_curve(k_now, k_ahead, vel_ahead):
  return {"modelV2": SimpleNamespace(orientationRate=SimpleNamespace(z=[k_ahead * vel_ahead] * 33),
                                     velocity=SimpleNamespace(x=[vel_ahead] * 33)),
          "controlsState": SimpleNamespace(curvature=k_now)}


def test_predicted_speed_up_is_not_a_sharper_curve():
  # review round 4: constant-radius bend at 25 m/s / 2.2 m/s^2, model plan accelerating to 32 m/s
  k = 2.2 / V_HWY ** 2
  scc = SmartCruiseControlVision(enabled=True)
  for _ in range(100):
    scc.update(_sm_curve(k, k, 32.0), True, False, V_HWY, 0.0, V_CRUISE_MS)
  assert scc.max_pred_lat_acc > 3.4  # what the old comparison saw
  assert scc.sharper_ahead < 0.01
  assert scc.output_a_target > -0.45


def test_tighter_bend_activates_despite_a_predicted_slowdown():
  # review round 4: lat 1.2 now, bend reaching 3.3 at today's speed, model slowing so rate * vel = 1.2
  k_now, k_ahead = 1.2 / V_HWY ** 2, 3.3 / V_HWY ** 2
  vel = (1.2 / k_ahead) ** 0.5
  scc = SmartCruiseControlVision(enabled=True)
  for _ in range(100):
    scc.update(_sm_curve(k_now, k_ahead, vel), True, False, V_HWY, 0.0, V_CRUISE_MS)
  assert scc.max_pred_lat_acc < _ENTERING_PRED_LAT_ACC_TH
  assert scc.is_active
  assert scc.output_a_target < -0.9


def test_reactivation_mid_release_restarts_from_the_car():
  # review round 4: release at +1.4 while the car does 0, then a new curve wants -1.0
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 3.5, lat=3.5, n=100)
  for _ in range(48):  # -1.0 -> +1.4 while released
    scc.update({}, True, False, V_HWY, 0.0, V_CRUISE_MS)
  assert scc.releasing and scc.output_a_target > 1.3
  _step(scc, 3.0, lat=0.7)
  assert scc.is_active
  assert scc.output_a_target <= 0.


def test_no_jump_while_active():
  # the mid-release restart must not apply during normal operation
  scc = SmartCruiseControlVision(enabled=True)
  _step(scc, 2.0, n=3)
  _step(scc, 1.0, lat=1.7, n=100, a_ego=-0.2)
  assert scc.state == VisionState.turning
  prev = scc.output_a_target
  assert prev > 0.2
  _step(scc, 3.0, lat=1.7, a_ego=-0.2)
  assert prev - scc.output_a_target <= _A_TARGET_STEP_DOWN + 1e-9
