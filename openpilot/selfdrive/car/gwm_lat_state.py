"""H6 MK4: why steering is not being applied while openpilot is engaged (log only).

Steering can drop with nothing on screen: the carcontroller's driver-override latch hands the wheel
back at a sustained |torque| > 130, and the EPS reports A_RX_STEER_REQUESTED != 1. A_RX != 1 alone is
not proof the wheel stopped following (route 103 tracked within 0.6 deg with A_RX=2), so card.py
publishes the reason together with the angle error as `gwmLatState`, for route analysis. No alert is
raised from it until the logs show which combination really means "steering stopped".
"""

# Order must match custom.capnp GwmLatState.Reason (test_gwm_lat_state pins it).
REASON_NONE = "none"
REASON_DRIVER_OVERRIDE = "driverOverride"
REASON_EPS_NOT_OBEYING = "epsNotObeying"
REASON_STANDSTILL = "standstill"
REASON_INACTIVE = "inactive"
REASONS = (REASON_NONE, REASON_DRIVER_OVERRIDE, REASON_EPS_NOT_OBEYING, REASON_STANDSTILL, REASON_INACTIVE)

A_RX_OBEYING = 1


def lat_block_reason(*, enabled: bool, lat_active: bool, standstill: bool, override_active: bool, a_rx: int) -> str:
  if not enabled:
    return REASON_NONE
  if not lat_active:
    return REASON_STANDSTILL if standstill else REASON_INACTIVE
  if override_active:
    return REASON_DRIVER_OVERRIDE
  if a_rx != A_RX_OBEYING:
    return REASON_EPS_NOT_OBEYING
  return REASON_NONE


class GwmLatTracker:
  def __init__(self) -> None:
    self.reason = REASON_NONE
    self.frames = 0

  def update(self, reason: str) -> None:
    self.frames = self.frames + 1 if reason == self.reason and reason != REASON_NONE else int(reason != REASON_NONE)
    self.reason = reason
