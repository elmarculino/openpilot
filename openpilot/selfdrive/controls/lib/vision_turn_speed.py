"""
Vision turn speed (SCC-V), ported from sunnypilot SmartCruiseControlVision.

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
MIT License.
"""
from enum import IntEnum

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

MIN_V = 20 * CV.KPH_TO_MS  # do not operate under 20 km/h
# Sentinel for "no speed request". Unitless so it cannot be confused with the km/h
# V_CRUISE_UNSET: output_v_target is consumed as m/s and only ever read through min().
V_TARGET_UNSET = float('inf')
PARAMS_UPDATE_PERIOD = 3.0  # seconds


class VisionState(IntEnum):
  disabled = 0
  enabled = 1
  entering = 2
  turning = 3
  leaving = 4
  overriding = 5


ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

_ENTERING_PRED_LAT_ACC_TH = 1.3
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1
_TURNING_LAT_ACC_TH = 1.6
_LEAVING_LAT_ACC_TH = 1.3
_FINISH_LAT_ACC_TH = 1.1
_A_LAT_REG_MAX = 2.0
_NO_OVERSHOOT_TIME_HORIZON = 4.0

_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]
_TURNING_ACC_V = [0.5, 0., -0.4]
_TURNING_ACC_BP = [1.5, 2.3, 3.]
_LEAVING_ACC = 0.5


class SmartCruiseControlVision:
  def __init__(self, enabled: bool | None = None):
    self.v_target = 0.
    self.a_target = 0.
    self.v_ego = 0.
    self.a_ego = 0.
    self.output_v_target = V_TARGET_UNSET
    self.output_a_target = 0.

    # An explicit `enabled` pins the toggle, and then Params is never touched. Deferred to
    # _read_enabled because common.params does a module-level ctypes.CDLL of libparams_c, which is
    # what kept test_vision_turn_speed from running off-device (PR #2 review).
    self.params = None
    self.frame = 0
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    # An explicit value pins the toggle (tests, replay); otherwise it tracks Params.
    self._enabled_override = enabled
    self.enabled = enabled if enabled is not None else self._read_enabled()
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON
    return V_TARGET_UNSET

  def _read_enabled(self) -> bool:
    from openpilot.common.params import Params, UnknownKeyName  # noqa: PLC0415
    if self.params is None:
      self.params = Params()
    try:
      return self.params.get_bool("SmartCruiseControlVision")
    except UnknownKeyName:
      return False

  def _update_params(self) -> None:
    if self._enabled_override is not None:
      return
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self._read_enabled()

  def _reset_calculations(self) -> None:
    # Both lat accels must go to zero: a stale current_lat_acc keeps the state machine latched in
    # `turning` (exit needs current_lat_acc <= _LEAVING_LAT_ACC_TH), commanding decel forever.
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.
    self.v_target = self.v_cruise_setpoint

  def _update_calculations(self, sm) -> None:
    if not self.long_enabled:
      return

    try:
      rate_plan = np.abs(sm['modelV2'].orientationRate.z)
      vel_plan = np.array(sm['modelV2'].velocity.x)
      curvature = abs(sm['controlsState'].curvature)
    # Broad on purpose: a model dropout must release the decel rather than kill plannerd.
    # test_missing_model_fields_do_not_raise / test_model_dropout_mid_turn_releases_decel cover
    # the KeyError and AttributeError paths -- do not narrow this.
    except (AttributeError, TypeError, ValueError, KeyError):
      self._reset_calculations()
      return

    n = min(len(rate_plan), len(vel_plan))
    if n == 0:
      self._reset_calculations()
      return

    self.current_lat_acc = self.v_ego ** 2 * curvature

    predicted_lat_accels = rate_plan[:n] * vel_plan[:n]
    self.max_pred_lat_acc = float(np.percentile(predicted_lat_accels, 97))
    if not np.isfinite(self.max_pred_lat_acc):
      self.max_pred_lat_acc = 0.

    v_ego = max(self.v_ego, 0.1)
    max_curve = self.max_pred_lat_acc / (v_ego ** 2)
    if max_curve < 1e-6:
      self.v_target = self.v_cruise_setpoint
    else:
      self.v_target = (_A_LAT_REG_MAX / max_curve) ** 0.5

  def _update_state_machine(self) -> tuple[bool, bool]:
    if self.state != VisionState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding
      else:
        if self.state == VisionState.enabled:
          if self.v_ego <= MIN_V:
            pass
          elif self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.entering
        elif self.state == VisionState.overriding:
          # reached only when long_override just went False (the elif above catches it otherwise)
          self.state = VisionState.enabled
        elif self.state == VisionState.entering:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.enabled
        elif self.state == VisionState.turning:
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving
        elif self.state == VisionState.leaving:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH:
            self.state = VisionState.enabled
    elif self.long_enabled and self.enabled:
      self.state = VisionState.overriding if self.long_override else VisionState.enabled

    return self.state in ENABLED_STATES, self.state in ACTIVE_STATES

  def _update_solution(self) -> float:
    if self.state not in ACTIVE_STATES:
      return self.a_ego
    if self.state == VisionState.entering:
      return float(np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V))
    if self.state == VisionState.turning:
      return float(np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V))
    if self.state == VisionState.leaving:
      return _LEAVING_ACC
    raise NotImplementedError(f"SCC-V state not supported: {self.state}")

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)
    self.is_enabled, self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()
    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()
    self.frame += 1
