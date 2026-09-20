"""H6 MK4 MADS-lite: ACC and LKAS are independent.

Gentle DOWN toggles lateral only. Detent engages both. Brake/regen
drops ACC and keeps LKAS. Stalk UP cancels both (handled as buttonCancel
outside this helper).
"""
from __future__ import annotations


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
      self.long_enabled = True

    override_long = engaged and not self.long_enabled and not want_cancel
    return want_enable, want_cancel, override_long
