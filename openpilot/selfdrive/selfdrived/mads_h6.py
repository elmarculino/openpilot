"""H6 MK4 MADS-lite: ACC and LKAS are independent.

Gentle DOWN toggles lateral only. Detent engages both. Brake/regen
drops ACC and keeps LKAS. Stalk UP cancels both (handled as buttonCancel
outside this helper).
"""
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog

SafetyModel = car.CarParams.SafetyModel
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)

# pandaStates publishes at 10 Hz against a 100 Hz control loop, so sm['pandaStates'] can still hold a
# pre-arm sample for ~10 frames after MADS engages. 50 frames (0.5 s) clears that skew with margin.
MISMATCH_FRAMES = int(0.5 / DT_CTRL)

# Why longitudinal is overridden, for the log. The event raised for it is `gasPressedOverride`,
# reused for its ET.OVERRIDE_LONGITUDINAL and empty AlertSize.none alert -- so without this a
# brake-held override reads as "gas pressed" with the driver's foot nowhere near the accelerator.
# Names, and their order, must match custom.capnp MadsState.OverrideSource; test_mads_h6 pins it.
OVERRIDE_NONE = "none"
OVERRIDE_BRAKE = "brake"
OVERRIDE_LATERAL_ONLY = "lateralOnly"
OVERRIDE_SOURCES = (OVERRIDE_NONE, OVERRIDE_BRAKE, OVERRIDE_LATERAL_ONLY)


def uses_h6_mads(CP) -> bool:
  """Single source of truth for the MADS-lite gate.

  selfdrived (which events to raise) and car_events (which to suppress) must agree, so keep
  the condition here rather than repeating it.
  """
  return CP.brand == 'gwm' and not CP.pcmCruise


class H6Mads:
  def __init__(self) -> None:
    self.long_enabled = False
    self.mismatch_counter = 0
    self.mismatch = False
    self.override_source = OVERRIDE_NONE

  def reset(self) -> None:
    self.long_enabled = False
    self.mismatch_counter = 0
    self.mismatch = False
    self.override_source = OVERRIDE_NONE

  def data_sample(self, panda_states, engaged: bool) -> None:
    """Flag a MADS engagement the panda is not backing.

    selfdrived already counts this (`mismatch_counter >= 200`) but needs 2 s, and pcmCruise=False
    cars arm the panda off a stalk gesture the panda tracks itself -- so the two layers can disagree
    from the very first frame with nothing in the log but pandaStates. Route 000000fd--3227e9ca98:
    openpilot held `enabled` for 1.4 s while the panda sat at controls_allowed=0 and rejected every
    TX, too short to reach 200 frames, so no alert and no event were ever raised. Trip at 0.5 s.
    """
    if not engaged:
      self.mismatch_counter = 0
      self.mismatch = False
      return

    allowed = [ps.controlsAllowed for ps in panda_states if ps.safetyModel not in IGNORED_SAFETY_MODES]
    # an empty list means every panda is silent/noOutput: nothing is expected to allow controls
    if allowed and not any(allowed):
      self.mismatch_counter += 1
    else:
      self.mismatch_counter = 0

    if self.mismatch_counter >= MISMATCH_FRAMES and not self.mismatch:
      # latched until disengage: the event is IMMEDIATE_DISABLE, so that follows within a frame
      self.mismatch = True
      cloudlog.error("mads_h6: engaged with no panda allowing controls, disengaging")

  def update(self, *, engaged: bool, user_brake: bool, lkas_tap: bool, acc_enable: bool) -> tuple[bool, bool, bool]:
    """Returns (want_enable, want_cancel, override_long)."""
    want_enable = False
    want_cancel = False

    if user_brake:
      self.long_enabled = False
      self.override_source = OVERRIDE_BRAKE

    if lkas_tap:
      if not engaged:
        want_enable = True
        self.long_enabled = False
        self.override_source = OVERRIDE_LATERAL_ONLY
      elif self.long_enabled:
        self.long_enabled = False
        self.override_source = OVERRIDE_LATERAL_ONLY
      else:
        want_cancel = True

    if acc_enable:
      want_enable = True
      want_cancel = False
      # A brake in this same cycle still wins: engaging while braking is lat-only until release.
      # Without this, ENABLE arrives without OVERRIDE_LONGITUDINAL and state.py:86 lands on
      # State.enabled instead of State.overriding -- the FSM would believe long is live with a
      # foot on the pedal (the panda blocks the TX either way, but the two layers must agree).
      self.long_enabled = not user_brake
      self.override_source = OVERRIDE_BRAKE if user_brake else OVERRIDE_NONE

    override_long = (engaged or want_enable) and not self.long_enabled and not want_cancel
    # The source only means anything while overriding, so clear it rather than let a stale cause
    # sit in the log for the rest of the drive.
    if not override_long:
      self.override_source = OVERRIDE_NONE
    return want_enable, want_cancel, override_long
