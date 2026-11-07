from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized

from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.gm.values import CAR as GM
from opendbc.car.gwm.values import CAR as GWM
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD


class TestLatControl(OpenpilotTestCase):

  @parameterized.expand([(HONDA.HONDA_CIVIC, LatControlPID), (TOYOTA.TOYOTA_RAV4, LatControlTorque),
                         (NISSAN.NISSAN_LEAF, LatControlAngle), (GM.CHEVROLET_BOLT_EUV, LatControlTorque)])
  def test_saturation(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False

    params = log.VehicleParameters.new_message()

    # Saturate for curvature limited and controller limited
    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, True, 0.2)
    assert lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, False, 0.2)
    assert not lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 1, False, 0.2)
    assert lac_log.saturated

  def test_gwm_saturation_threshold_is_speed_dependent(self):
    CarInterface = interfaces[GWM.GWM_HAVAL_H6_MK4]
    CP = CarInterface.get_non_essential_params(GWM.GWM_HAVAL_H6_MK4)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)
    controller = LatControlAngle(CP.as_reader(), CI, DT_CTRL)
    # low speed: lat-accel equivalent dominates; highway: falls back to the fixed 2.5 deg floor
    assert controller.saturation_threshold(10 / 3.6, VM) > 2 * STEER_ANGLE_SATURATION_THRESHOLD
    assert controller.saturation_threshold(60 / 3.6, VM) > controller.saturation_threshold(100 / 3.6, VM)
    assert controller.saturation_threshold(150 / 3.6, VM) == STEER_ANGLE_SATURATION_THRESHOLD

    nissan_CP = interfaces[NISSAN.NISSAN_LEAF].get_non_essential_params(NISSAN.NISSAN_LEAF)
    nissan = LatControlAngle(nissan_CP.as_reader(), interfaces[NISSAN.NISSAN_LEAF](nissan_CP), DT_CTRL)
    assert nissan.saturation_threshold(10 / 3.6, VehicleModel(nissan_CP)) == STEER_ANGLE_SATURATION_THRESHOLD
