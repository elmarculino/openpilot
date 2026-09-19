"""
Vision turn speed (SCC-V), ported from sunnypilot SmartCruiseControlVision.

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
MIT License.
"""
from enum import IntEnum
from typing import Any

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET

MIN_V = 20 * CV.KPH_TO_MS  # do not operate under 20 km/h
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
  v_target: float = 0.
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    try:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")
    except UnknownKeyName:
      self.enabled = True
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON
    return V_CRUISE_UNSET

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      try:
        self.enabled = self.params.get_bool("SmartCruiseControlVision")
      except UnknownKeyName:
        self.enabled = True

  def _update_calculations(self, sm: Any) -> None:
    if not self.long_enabled:
      return

    try:
      rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
      vel_plan = np.array(sm['modelV2'].velocity.x)
      curvature = abs(sm['controlsState'].curvature)
    except (AttributeError, TypeError, ValueError):
      self.max_pred_lat_acc = 0.
      self.v_target = self.v_cruise_setpoint
      return

    n = min(len(rate_plan), len(vel_plan))
    if n == 0:
      self.max_pred_lat_acc = 0.
      self.v_target = self.v_cruise_setpoint
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
          if not self.long_override:
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

  def update(self, sm: Any, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
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
