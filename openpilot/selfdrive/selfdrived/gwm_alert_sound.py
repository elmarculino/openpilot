"""H6 MK4 sound policy: beep only for things the driver did not ask for.

Two sounds are dropped, matched on the exact alert_type ("<event>/<event type>"):
  buttonEnable/enable      the driver pulled the stalk DOWN and got what they asked for; the
                           cluster icon confirms it
  steerSaturated/warning   "turn exceeds limit" is advisory: routes 107/109 had 23-49 s of repeated
                           beeping and every disengage of 109 came within 15 s of a burst

Everything else keeps its sound, on purpose:
  buttonCancel/userDisable stalk UP still chimes, so a real "off" sounds different from a gentle
                           DOWN that changed nothing (gentle DOWN no longer turns steering off)
  */noEntry                a refused engage must never be silent, or silence reads as "engaged"
  gear / park / faults     leaving Drive or any forced disengage may not be what the driver did
"""
from openpilot.cereal import log

AudibleAlert = log.SelfdriveState.AudibleAlert

SILENT_ALERT_TYPES = frozenset({
  "buttonEnable/enable",
  "steerSaturated/warning",
})


def gwm_alert_sound(alert_type: str, sound):
  if alert_type in SILENT_ALERT_TYPES:
    return AudibleAlert.none
  return sound
