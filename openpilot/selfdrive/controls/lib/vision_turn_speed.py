"""
Vision turn speed (SCC-V), ported from sunnypilot SmartCruiseControlVision.

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
MIT License.
"""
from enum import IntEnum

import numpy as np

from opendbc.car.interfaces import ACCEL_MAX
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

# 1.5, not sunnypilot's 1.3: on long gentle bends (route 1c5) the prediction hovered at 1.1-1.3
# and SCC-V flipped enabled<->entering every 2-3 s, alternating -0.2 and +0.4 m/s^2.
_ENTERING_PRED_LAT_ACC_TH = 1.5
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1
_TURNING_LAT_ACC_TH = 1.6
_LEAVING_LAT_ACC_TH = 1.3
_FINISH_LAT_ACC_TH = 1.1
_A_LAT_REG_MAX = 2.0
_NO_OVERSHOOT_TIME_HORIZON = 4.0
# Brake for a curve before it, not in it. Route 1c5: the speed cap stayed on inside curves and,
# recomputed against the falling speed, held the cruise accel at its -1.2 m/s^2 floor (car measured
# -2.0) -- the "brakes mid-curve" complaint. So the cap only applies on the approach from a straight;
# inside a curve (measured lat acc, with hysteresis) SCC-V brakes only through its own candidate.
_IN_CURVE_ON_LAT_ACC = 0.6
_IN_CURVE_OFF_LAT_ACC = 0.4
# Inside a curve, how much sharper the curve ahead is (predicted minus current lat acc) decides the
# response, graded so a dithering prediction cannot flip it: any positive request fades out over
# 0.3-0.6 m/s^2 (never accelerate toward a sharper curve), then the pre-curve decel fades in up to
# its full value at 1.2.
_NO_ACCEL_AHEAD_BP = [0.3, 0.6]
_APPROACH_AHEAD_BP = [0.6, 1.2]
# In-curve floor: -0.4 up to 3.0 m/s^2 (sunnypilot's turning floor), then a backstop down to -1.0
# by 3.5 m/s^2 -- past that the curve is taken too fast to stay gentle. Reachable from `enabled`.
_IN_CURVE_MIN_ACC_BP = [3.0, 3.5]
_IN_CURVE_MIN_ACC_V = [-0.4, -1.0]
# SCC-V's candidate bypasses the cruise jerk limit, so it carries its own: braking builds at up to
# 2 m/s^3 and releases at 1 m/s^3. When SCC-V stops being active the candidate is not dropped but
# released at that rate up to the planner's ACCEL_MAX (where it can no longer bind), so the planner
# never sees a jump. On activation it ramps from the car's accel, but never from below its own solution.
_A_TARGET_STEP_DOWN = 2.0 * DT_MDL
_A_TARGET_STEP_UP = 1.0 * DT_MDL

_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]
_TURNING_ACC_V = [0.5, 0., -0.4, -1.0]
_TURNING_ACC_BP = [1.5, 2.3, 3., 3.5]
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
    self.pred_lat_acc_at_v_ego = 0.  # the curve ahead at today's speed, not the model's future speed
    self.in_curve = False
    self.has_candidate = False  # planner: SCC-V's accel candidate is live (active, or releasing)

  @property
  def sharper_ahead(self) -> float:
    """How much sharper the curve ahead is than the current one, both at the current speed (0 when
    not in a curve). The model's own lat acc uses its future speeds, and a predicted slowdown would
    hide the tighter bend (review round 3)."""
    if not self.in_curve:
      return 0.
    return max(max(self.max_pred_lat_acc, self.pred_lat_acc_at_v_ego) - self.current_lat_acc, 0.)

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.state == VisionState.entering and not self.in_curve:
      # MIN_V floors the curve solution, but the no-overshoot term is a decel (down to -1 m/s^2 in
      # `entering`), so 4 s of it lands ~4 m/s under the floor -- below the 20 km/h entry gate the
      # state machine itself respects. Clamp after adding it so a later threshold tweak cannot ask
      # for a near-stop by accident (PR #2 review). The floor is not a target: longitudinal_planner
      # only ever reads this through min() against v_cruise.
      return max(max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON, MIN_V)
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
    self.pred_lat_acc_at_v_ego = 0.
    self.v_target = self.v_cruise_setpoint

  def _update_calculations(self, sm) -> None:
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

    # tracked even while long is off, so the in-curve latch is current when it comes back
    self.current_lat_acc = self.v_ego ** 2 * curvature
    if not self.long_enabled:
      return

    n = min(len(rate_plan), len(vel_plan))
    if n == 0:
      self._reset_calculations()
      return

    moving = vel_plan[:n] > 1.
    if np.any(moving):
      pred_curvature = float(np.percentile(rate_plan[:n][moving] / vel_plan[:n][moving], 97))
      self.pred_lat_acc_at_v_ego = self.v_ego ** 2 * pred_curvature if np.isfinite(pred_curvature) else 0.
    else:
      self.pred_lat_acc_at_v_ego = 0.

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
          elif self.current_lat_acc >= _IN_CURVE_MIN_ACC_BP[0]:
            # already deep in a sharp curve the model no longer predicts: the backstop still applies
            self.state = VisionState.turning
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
            # straight into the next curve: approach it instead of dropping out and re-entering
            entering = self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH and self.v_ego > MIN_V
            self.state = VisionState.entering if entering else VisionState.enabled
    elif self.long_enabled and self.enabled:
      self.state = VisionState.overriding if self.long_override else VisionState.enabled

    return self.state in ENABLED_STATES, self.state in ACTIVE_STATES

  def _update_in_curve(self) -> None:
    if self.current_lat_acc >= _IN_CURVE_ON_LAT_ACC:
      self.in_curve = True
    elif self.current_lat_acc < _IN_CURVE_OFF_LAT_ACC:
      self.in_curve = False

  def _update_solution(self) -> float:
    if self.state not in ACTIVE_STATES:
      return self.a_ego
    approach = float(np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V))
    if self.state == VisionState.entering and not self.in_curve:
      return approach
    # in a curve, size the response to the bend ahead at today's speed as well (see sharper_ahead)
    pred_now = max(self.max_pred_lat_acc, self.pred_lat_acc_at_v_ego)
    approach = float(np.interp(pred_now, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V))
    if self.state == VisionState.turning:
      own = float(np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V))
    elif self.state == VisionState.leaving:
      own = _LEAVING_ACC
    elif self.state == VisionState.entering:  # already in a gentle curve
      own = max(approach, float(np.interp(self.current_lat_acc, _IN_CURVE_MIN_ACC_BP, _IN_CURVE_MIN_ACC_V)))
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")
    ahead = self.sharper_ahead
    if own > 0.:
      own *= float(np.interp(ahead, _NO_ACCEL_AHEAD_BP, [1., 0.]))
    w = float(np.interp(ahead, _APPROACH_AHEAD_BP, [0., 1.]))
    return min(own, own + w * (approach - own))

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)
    self._update_in_curve()
    self.is_enabled, self.is_active = self._update_state_machine()
    if self.is_active:
      solution = self._update_solution()
      # ramp from the live candidate; a new one starts from the car's accel, but not below its own
      # solution -- a car already braking harder (lead) must not have SCC-V prolong it (review round 3)
      prev = self.a_target if self.has_candidate else max(self.a_ego, solution)
      self.a_target = float(np.clip(solution, prev - _A_TARGET_STEP_DOWN, prev + _A_TARGET_STEP_UP))
      self.has_candidate = True
    elif self.has_candidate and self.state == VisionState.enabled and self.a_target < ACCEL_MAX:
      self.a_target += _A_TARGET_STEP_UP  # release, do not drop
    else:
      # idle, release done, or disabled / overriding (long off or the driver has it): no candidate
      self.has_candidate = False
      self.a_target = self._update_solution()
    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()
    self.frame += 1
