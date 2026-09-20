from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.vision_turn_speed import (
  MIN_V,
  SmartCruiseControlVision,
  VisionState,
  V_TARGET_UNSET,
  _ENTERING_PRED_LAT_ACC_TH,
  _TURNING_LAT_ACC_TH,
)


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
