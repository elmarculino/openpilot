"""H6 MK4 MADS-lite: ACC and LKAS are independent.

Gentle DOWN toggles lateral only. Detent engages both. Brake/regen
drops ACC and keeps LKAS. Stalk UP cancels both (handled as buttonCancel
outside this helper).
"""


def uses_h6_mads(CP) -> bool:
  """Single source of truth for the MADS-lite gate.

  selfdrived (which events to raise) and car_events (which to suppress) must agree, so keep
  the condition here rather than repeating it.
  """
  return CP.brand == 'gwm' and not CP.pcmCruise


class H6Mads:
  def __init__(self) -> None:
    self.long_enabled = False

  def reset(self) -> None:
    self.long_enabled = False

  def update(self, *, engaged: bool, user_brake: bool, lkas_tap: bool, acc_enable: bool) -> tuple[bool, bool, bool]:
    """Returns (want_enable, want_cancel, override_long)."""
    want_enable = False
    want_cancel = False

    if user_brake:
      self.long_enabled = False

    if lkas_tap:
      if not engaged:
        want_enable = True
        self.long_enabled = False
      elif self.long_enabled:
        self.long_enabled = False
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

    override_long = (engaged or want_enable) and not self.long_enabled and not want_cancel
    return want_enable, want_cancel, override_long
