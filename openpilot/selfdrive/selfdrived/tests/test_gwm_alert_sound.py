from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventName
from openpilot.selfdrive.selfdrived.gwm_alert_sound import gwm_alert_sound
from openpilot.selfdrive.selfdrived.mads_h6 import H6Mads
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

AudibleAlert = log.SelfdriveState.AudibleAlert
EVENT_NAMES = {v: k for k, v in EventName.schema.enumerants.items()}


def _sd(h6_mk4=True, brand="gwm"):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = SimpleNamespace(brand=brand)
  sd.AM = AlertManager()
  sd.mads = H6Mads() if h6_mk4 else None
  return sd


def _sound(sd, event, et):
  alert = EVENTS[event][et]
  if callable(alert):
    alert = alert(SimpleNamespace(brand=sd.CP.brand), SimpleNamespace(), None, False, 0, None)
  alert.alert_type = f"{EVENT_NAMES[event]}/{et}"
  alert.event_type = et
  sd.AM.add_many(0, [alert])
  sd.AM.process_alerts(0, set())
  return sd.alert_sound()


def test_stalk_engage_is_silent():
  assert _sound(_sd(), EventName.buttonEnable, ET.ENABLE) == AudibleAlert.none


def test_stalk_up_disengage_is_silent():
  assert _sound(_sd(), EventName.buttonCancel, ET.USER_DISABLE) == AudibleAlert.none


def test_turn_exceeds_limit_is_visual_only():
  assert _sound(_sd(), EventName.steerSaturated, ET.WARNING) == AudibleAlert.none


def test_refused_engage_always_beeps():
  # silence reads as "engaged", so no refusal may be muted -- including for the silenced event names
  for event in (EventName.buttonEnable, EventName.buttonCancel, EventName.parkBrake, EventName.wrongCarMode,
                EventName.reverseGear, EventName.doorOpen):
    if ET.NO_ENTRY in EVENTS[event]:
      assert _sound(_sd(), event, ET.NO_ENTRY) != AudibleAlert.none, EVENT_NAMES[event]


def test_leaving_drive_and_forced_disengages_beep():
  assert _sound(_sd(), EventName.reverseGear, ET.USER_DISABLE) == AudibleAlert.warningImmediate
  assert _sound(_sd(), EventName.wrongCarMode, ET.USER_DISABLE) == AudibleAlert.disengage
  assert _sound(_sd(), EventName.parkBrake, ET.USER_DISABLE) == AudibleAlert.disengage
  assert _sound(_sd(), EventName.controlsMismatch, ET.IMMEDIATE_DISABLE) != AudibleAlert.none
  assert _sound(_sd(), EventName.fcw, ET.PERMANENT) != AudibleAlert.none
  assert _sound(_sd(), EventName.steerTempUnavailableSilent, ET.WARNING) == AudibleAlert.prompt


def test_other_cars_unchanged():
  # MK3 is GWM too, but runs pcmCruise (no MADS-lite): stock sounds
  for brand in ("gwm", "toyota"):
    assert _sound(_sd(h6_mk4=False, brand=brand), EventName.buttonEnable, ET.ENABLE) == AudibleAlert.engage
    assert _sound(_sd(h6_mk4=False, brand=brand), EventName.buttonCancel, ET.USER_DISABLE) == AudibleAlert.disengage
  for brand in ("gwm", "toyota"):
    assert _sound(_sd(h6_mk4=False, brand=brand), EventName.steerSaturated, ET.WARNING) == AudibleAlert.promptRepeat


def test_policy_matches_the_exact_alert_type():
  assert gwm_alert_sound("buttonEnable/enable", AudibleAlert.engage) == AudibleAlert.none
  assert gwm_alert_sound("buttonEnable/noEntry", AudibleAlert.refuse) == AudibleAlert.refuse
  assert gwm_alert_sound("buttonEnableX/enable", AudibleAlert.engage) == AudibleAlert.engage
