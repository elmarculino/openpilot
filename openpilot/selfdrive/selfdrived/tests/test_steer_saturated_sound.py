from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventName
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD, STEER_SATURATED_SOUND_COOLDOWN

AudibleAlert = log.SelfdriveState.AudibleAlert


def _sd(brand):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = SimpleNamespace(brand=brand)
  sd.AM = AlertManager()
  sd.steer_saturated_alert_start = -1
  sd.steer_saturated_sound_frame = -10**9
  sd.steer_saturated_audible = True
  return sd


def _alert(brand):
  alert = EVENTS[EventName.steerSaturated][ET.WARNING](SimpleNamespace(brand=brand), None, None, False, 0, None)
  alert.alert_type = "steerSaturated/warning"
  alert.event_type = ET.WARNING
  return alert


def _run(sd, brand, frame, n=1):
  sounds = []
  for f in range(frame, frame + n):
    sd.AM.add_many(f, [_alert(brand)])
    sd.AM.process_alerts(f, set())
    sounds.append(sd.alert_sound())
  return sounds


def test_gwm_prompt_once_then_cooldown():
  sd = _sd("gwm")
  assert _run(sd, "gwm", 0, 50) == [AudibleAlert.prompt] * 50
  # a new alert a few seconds later on the same curve stays silent
  later = int(5. / DT_CTRL)
  assert set(_run(sd, "gwm", later, 50)) == {AudibleAlert.none}
  # after the cooldown it sounds again
  after = int((STEER_SATURATED_SOUND_COOLDOWN + 1.) / DT_CTRL)
  assert _run(sd, "gwm", after, 1) == [AudibleAlert.prompt]


def test_other_brands_unchanged():
  sd = _sd("toyota")
  assert _run(sd, "toyota", 0, 1) == [AudibleAlert.promptRepeat]
  assert _run(sd, "toyota", int(5. / DT_CTRL), 1) == [AudibleAlert.promptRepeat]
