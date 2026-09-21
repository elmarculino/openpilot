import math
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.vision_turn_speed import (
  MIN_V,
  SmartCruiseControlVision,
  VisionState,
  V_TARGET_UNSET,
  _ENTERING_PRED_LAT_ACC_TH,
  _TURNING_LAT_ACC_TH,
)

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource


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
  scc.update(_sm(2.0), False, False, MIN_V + 5, 0.0, 30.0)
  assert scc.state == VisionState.disabled
  assert not scc.is_active
  assert scc.output_v_target == V_TARGET_UNSET


def test_stays_disabled_when_toggle_off():
  scc = SmartCruiseControlVision(enabled=False)
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  for _ in range(3):
    scc.update(sm, True, False, MIN_V + 5, 0.0, 30.0)
  assert scc.state == VisionState.disabled
  assert not scc.is_active


def test_enters_on_predicted_lat_acc():
  scc = SmartCruiseControlVision(enabled=True)
  v_ego = MIN_V + 5
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  scc.update(sm, True, False, v_ego, 0.0, 30.0)
  assert scc.state == VisionState.enabled
  scc.update(sm, True, False, v_ego, 0.0, 30.0)
  assert scc.state == VisionState.entering
  assert scc.is_active
  assert scc.output_v_target < 30.0
  assert scc.output_a_target < 0.0


def test_drops_when_long_goes_inactive():
  scc = SmartCruiseControlVision(enabled=True)
  v_ego = MIN_V + 5
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  scc.update(sm, True, False, v_ego, 0.0, 30.0)
  scc.update(sm, True, False, v_ego, 0.0, 30.0)
  assert scc.is_active
  scc.update(sm, False, False, v_ego, 0.0, 30.0)
  assert scc.state == VisionState.disabled
  assert not scc.is_active
  assert scc.output_v_target == V_TARGET_UNSET


def test_long_override_suspends_and_resumes():
  scc = SmartCruiseControlVision(enabled=True)
  v_ego = MIN_V + 5
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5)
  scc.update(sm, True, False, v_ego, 0.0, 30.0)
  scc.update(sm, True, True, v_ego, 0.0, 30.0)
  assert scc.state == VisionState.overriding
  assert not scc.is_active
  scc.update(sm, True, False, v_ego, 0.0, 30.0)
  assert scc.state == VisionState.enabled


def test_empty_model_path_does_not_raise():
  scc = SmartCruiseControlVision(enabled=True)
  scc.update(_empty_sm(), True, False, MIN_V + 5, 0.0, 30.0)
  assert scc.max_pred_lat_acc == 0.0


def test_missing_model_fields_do_not_raise():
  scc = SmartCruiseControlVision(enabled=True)
  scc.update({}, True, False, MIN_V + 5, 0.0, 30.0)
  assert scc.max_pred_lat_acc == 0.0


def _drive_into_turn(scc, v_ego):
  # enabled -> entering -> turning
  sm = _sm(_ENTERING_PRED_LAT_ACC_TH + 0.5, curvature=0.02)
  for _ in range(3):
    scc.update(sm, True, False, v_ego, 0.0, 30.0)
  assert scc.state == VisionState.turning
  assert scc.current_lat_acc >= _TURNING_LAT_ACC_TH


def test_model_dropout_mid_turn_releases_decel():
  # a dropout must not latch `turning` forever: current_lat_acc has to be reset alongside
  # max_pred_lat_acc, otherwise the stale value never satisfies the leaving threshold.
  v_ego = MIN_V + 5
  scc = SmartCruiseControlVision(enabled=True)
  _drive_into_turn(scc, v_ego)

  scc.update({}, True, False, v_ego, 0.0, 30.0)
  assert scc.current_lat_acc == 0.0
  assert scc.state == VisionState.leaving
  assert scc.output_a_target >= 0.0

  scc.update({}, True, False, v_ego, 0.0, 30.0)
  assert scc.state == VisionState.enabled
  assert not scc.is_active
  assert scc.output_v_target == V_TARGET_UNSET


def test_empty_model_path_mid_turn_releases_decel():
  v_ego = MIN_V + 5
  scc = SmartCruiseControlVision(enabled=True)
  _drive_into_turn(scc, v_ego)

  scc.update(_empty_sm(), True, False, v_ego, 0.0, 30.0)
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
  fields = log.LongitudinalPlan.Sccv.schema.fieldnames
  assert set(fields) == {"state", "currentLatAcc", "maxPredLatAcc", "vTarget", "aTarget"}


def test_every_vision_state_is_loggable():
  # publish() assigns VisionState(...).name straight into the capnp enum, so a state added to the
  # IntEnum without a matching enumerant raises on the car, in plannerd, mid-drive.
  loggable = {str(e) for e in log.LongitudinalPlan.SmartCruiseControlVisionState.schema.enumerants}
  assert {s.name for s in VisionState} == loggable
  for state in VisionState:
    assert log.LongitudinalPlan.SmartCruiseControlVisionState.schema.enumerants[state.name] == state.value


def test_sccv_state_round_trips_through_a_log():
  scc = SmartCruiseControlVision(enabled=True)
  _drive_into_turn(scc, MIN_V + 5)

  ev = log.Event.new_message()
  plan = ev.init('longitudinalPlan')
  plan.longitudinalPlanSource = LongitudinalPlanSource.turnSpeed
  plan.sccv.state = VisionState(scc.state).name
  plan.sccv.currentLatAcc = float(scc.current_lat_acc)
  plan.sccv.maxPredLatAcc = float(scc.max_pred_lat_acc)
  plan.sccv.vTarget = float(scc.v_target)
  plan.sccv.aTarget = float(scc.a_target)

  with log.Event.from_bytes(ev.to_bytes()) as read_back:
    sccv = read_back.longitudinalPlan.sccv
    assert str(read_back.longitudinalPlan.longitudinalPlanSource) == "turnSpeed"
    assert str(sccv.state) == "turning"
    assert sccv.currentLatAcc >= _TURNING_LAT_ACC_TH
    # Float32 round-trip, so compare against the source values rather than pinning magnitudes:
    # in `turning` the commanded accel only goes negative past _TURNING_ACC_BP[1].
    # 1e-6 is just outside float32's ~1.2e-7 relative precision
    assert math.isclose(sccv.currentLatAcc, scc.current_lat_acc, rel_tol=1e-6)
    assert math.isclose(sccv.maxPredLatAcc, scc.max_pred_lat_acc, rel_tol=1e-6)
    assert math.isclose(sccv.vTarget, scc.v_target, rel_tol=1e-6)
    assert math.isclose(sccv.aTarget, scc.a_target, rel_tol=1e-6)
