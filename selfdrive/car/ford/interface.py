from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.ford.fordcan import CanBus
from openpilot.selfdrive.car.ford.values import Ecu, FordFlags
from openpilot.selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "ford"
    ret.dashcamOnly = bool(ret.flags & FordFlags.CANFD)

    ret.radarUnavailable = True
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 1.0

    # 优化纵向控制参数，提高弯道减速和加速响应
    ret.longitudinalTuning.kpBP = [0., 10.0 * CV.MPH_TO_MS, 20.0 * CV.MPH_TO_MS, 30.0 * CV.MPH_TO_MS]
    ret.longitudinalTuning.kpV = [0.5, 0.4, 0.3, 0.2]
    ret.longitudinalTuning.kiBP = [0., 10.0 * CV.MPH_TO_MS, 20.0 * CV.MPH_TO_MS]
    ret.longitudinalTuning.kiV = [0.1, 0.05, 0.02]
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.0]

    # 增加车辆模型参数，优化弯道减速
    ret.mass = 2242.0 + 136.0  # 车辆质量（kg），包括乘客和行李
    ret.wheelbase = 3.025      # 轴距（m）
    ret.steerRatio = 16.0      # 转向比
    ret.tireStiffnessFront = 1e6  # 前轮胎刚度
    ret.tireStiffnessRear = 1e6   # 后轮胎刚度

    CAN = CanBus(fingerprint=fingerprint)
    cfgs = [get_safety_config(car.CarParams.SafetyModel.ford)]
    if CAN.main >= 4:
      cfgs.insert(0, get_safety_config(car.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = cfgs

    ret.experimentalLongitudinalAvailable = True
    if experimental_long:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_FORD_LONG_CONTROL
      ret.openpilotLongitudinalControl = True

    if ret.flags & FordFlags.CANFD:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_FORD_CANFD
    else:
      # 检查车辆是否支持横向和纵向控制
      pscm_config = next((fw for fw in car_fw if fw.ecu == Ecu.eps and b'\x22\xDE\x01' in fw.request), None)
      if pscm_config:
        if len(pscm_config.fwVersion) != 24:
          ret.dashcamOnly = True
        else:
          config_tja = pscm_config.fwVersion[7]  # Traffic Jam Assist
          config_lca = pscm_config.fwVersion[8]  # Lane Centering Assist
          if config_tja != 0xFF or config_lca != 0xFF:
            ret.dashcamOnly = True

    # 自动变速箱检测
    found_ecus = [fw.ecu for fw in car_fw]
    if Ecu.shiftByWire in found_ecus or 0x5A in fingerprint[CAN.main] or docs:
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual
      ret.minEnableSpeed = 20.0 * CV.MPH_TO_MS

    # 盲点监测
    ret.enableBsm = 0x3A6 in fingerprint[CAN.main] and 0x3A7 in fingerprint[CAN.main]

    # LCA 可以在静止时转向
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1.
    ret.centerToFront = ret.wheelbase * 0.44
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # 创建按钮事件，处理驾驶员的输入
    ret.buttonEvents = create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise})

    events = self.create_common_events(ret, extra_gears=[GearShifter.manumatic])
    if not self.CS.vehicle_sensors_valid:
      events.add(car.CarEvent.EventName.vehicleSensorsInvalid)

    ret.events = events.to_msg()

    return ret
