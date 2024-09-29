from cereal import car
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.ford.fordcan import CanBus
from openpilot.selfdrive.car.ford.values import DBC, CarControllerParams, FordFlags
from openpilot.selfdrive.car.interfaces import CarStateBase

GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    if CP.transmissionType == TransmissionType.automatic:
      self.shifter_values = can_define.dv["Gear_Shift_by_Wire_FD1"]["TrnRng_D_RqGsm"]

    self.vehicle_sensors_valid = False

    self.prev_distance_button = 0
    self.distance_button = 0

    # 初始化滤波器和状态变量
    self.steering_angle_filtered = 0.0
    self.yaw_rate_filtered = 0.0
    self.alpha = 0.1  # 滤波系数，可根据需要调整

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # 确保车辆传感器数据有效
    self.vehicle_sensors_valid = cp.vl["SteeringPinion_Data"]["StePinCompAnEst_D_Qf"] == 3

    # 车辆速度和加速度
    ret.vEgoRaw = cp.vl["BrakeSysFeatures"]["Veh_V_ActlBrk"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = cp.vl["DesiredTorqBrk"]["VehStop_D_Stat"] == 1

    # 滤波后的偏航率，用于提高曲率计算的准确性
    raw_yaw_rate = cp.vl["Yaw_Data_FD1"]["VehYaw_W_Actl"]
    self.yaw_rate_filtered = (1 - self.alpha) * self.yaw_rate_filtered + self.alpha * raw_yaw_rate
    ret.yawRate = self.yaw_rate_filtered

    # 油门信息
    ret.gas = cp.vl["EngVehicleSpThrottle"]["ApedPos_Pc_ActlArb"] / 100.
    ret.gasPressed = ret.gas > 1e-6

    # 刹车信息
    ret.brake = cp.vl["BrakeSnData_4"]["BrkTot_Tq_Actl"] / 32756.
    ret.brakePressed = cp.vl["EngBrakeData"]["BpedDrvAppl_D_Actl"] == 2
    ret.parkingBrake = cp.vl["DesiredTorqBrk"]["PrkBrkStatus"] in (1, 2)

    # 转向角度，添加滤波以减少噪声
    raw_steering_angle = cp.vl["SteeringPinion_Data"]["StePinComp_An_Est"]
    self.steering_angle_filtered = (1 - self.alpha) * self.steering_angle_filtered + self.alpha * raw_steering_angle
    ret.steeringAngleDeg = self.steering_angle_filtered

    ret.steeringTorque = cp.vl["EPAS_INFO"]["SteeringColumnTorque"]
    ret.steeringPressed = self.update_steering_pressed(
      abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    ret.steerFaultTemporary = cp.vl["EPAS_INFO"]["EPAS_Failure"] == 1
    ret.steerFaultPermanent = cp.vl["EPAS_INFO"]["EPAS_Failure"] in (2, 3)
    ret.espDisabled = cp.vl["Cluster_Info1_FD1"]["DrvSlipCtlMde_D_Rq"] != 0

    if self.CP.flags & FordFlags.CANFD:
      ret.steerFaultTemporary |= cp.vl["Lane_Assist_Data3_FD1"]["LatCtlSte_D_Stat"] not in (1, 2, 3)

    # 巡航控制状态
    is_metric = cp.vl["INSTRUMENT_PANEL"]["METRIC_UNITS"] == 1 if not self.CP.flags & FordFlags.CANFD else False
    ret.cruiseState.speed = cp.vl["EngBrakeData"]["Veh_V_DsplyCcSet"] * \
                            (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    ret.cruiseState.enabled = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (4, 5)
    ret.cruiseState.available = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (3, 4, 5)
    ret.cruiseState.nonAdaptive = cp.vl["Cluster_Info1_FD1"]["AccEnbl_B_RqDrv"] == 0
    ret.cruiseState.standstill = cp.vl["EngBrakeData"]["AccStopMde_D_Rq"] == 3
    ret.accFaulted = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (1, 2)
    if not self.CP.openpilotLongitudinalControl:
      ret.accFaulted = ret.accFaulted or cp_cam.vl["ACCDATA"]["CmbbDeny_B_Actl"] == 1

    # 档位信息
    if self.CP.transmissionType == TransmissionType.automatic:
      gear = self.shifter_values.get(cp.vl["Gear_Shift_by_Wire_FD1"]["TrnRng_D_RqGsm"])
      ret.gearShifter = self.parse_gear_shifter(gear)
    elif self.CP.transmissionType == TransmissionType.manual:
      ret.clutchPressed = cp.vl["Engine_Clutch_Data"]["CluPdlPos_Pc_Meas"] > 0
      if bool(cp.vl["BCM_Lamp_Stat_FD1"]["RvrseLghtOn_B_Stat"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # 安全信息
    ret.stockFcw = bool(cp_cam.vl["ACCDATA_3"]["FcwVisblWarn_B_Rq"])
    ret.stockAeb = bool(cp_cam.vl["ACCDATA_2"]["CmbbBrkDecel_B_Rq"])

    # 按钮状态
    ret.leftBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 1
    ret.rightBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 2
    ret.genericToggle = bool(cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"])
    self.prev_distance_button = self.distance_button
    self.distance_button = cp.vl["Steering_Data_FD1"]["AccButtnGapTogglePress"]

    # 车门和安全带状态
    ret.doorOpen = any([
      cp.vl["BodyInfo_3_FD1"]["DrStatDrv_B_Actl"],
      cp.vl["BodyInfo_3_FD1"]["DrStatPsngr_B_Actl"],
      cp.vl["BodyInfo_3_FD1"]["DrStatRl_B_Actl"],
      cp.vl["BodyInfo_3_FD1"]["DrStatRr_B_Actl"]
    ])
    ret.seatbeltUnlatched = cp.vl["RCMStatusMessage2_FD1"]["FirstRowBuckleDriver"] == 2

    # 盲点监测
    if self.CP.enableBsm:
      cp_bsm = cp_cam if self.CP.flags & FordFlags.CANFD else cp
      ret.leftBlindspot = cp_bsm.vl["Side_Detect_L_Stat"]["SodDetctLeft_D_Stat"] != 0
      ret.rightBlindspot = cp_bsm.vl["Side_Detect_R_Stat"]["SodDetctRight_D_Stat"] != 0

    # 保存原始按钮值和状态
    self.buttons_stock_values = cp.vl["Steering_Data_FD1"]
    self.acc_tja_status_stock_values = cp_cam.vl["ACCDATA_3"]
    self.lkas_status_stock_values = cp_cam.vl["IPMA_Data"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ("VehicleOperatingModes", 100),
      ("BrakeSysFeatures", 50),
      ("Yaw_Data_FD1", 100),
      ("DesiredTorqBrk", 50),
      ("EngVehicleSpThrottle", 100),
      ("BrakeSnData_4", 50),
      ("EngBrakeData", 10),
      ("Cluster_Info1_FD1", 10),
      ("SteeringPinion_Data", 100),
      ("EPAS_INFO", 50),
      ("Steering_Data_FD1", 10),
      ("BodyInfo_3_FD1", 2),
      ("RCMStatusMessage2_FD1", 10),
    ]

    if CP.flags & FordFlags.CANFD:
      messages += [
        ("Lane_Assist_Data3_FD1", 33),
      ]
    else:
      messages += [
        ("INSTRUMENT_PANEL", 1),
      ]

    if CP.transmissionType == TransmissionType.automatic:
      messages += [
        ("Gear_Shift_by_Wire_FD1", 10),
      ]
    elif CP.transmissionType == TransmissionType.manual:
      messages += [
        ("Engine_Clutch_Data", 33),
        ("BCM_Lamp_Stat_FD1", 1),
      ]

    if CP.enableBsm and not (CP.flags & FordFlags.CANFD):
      messages += [
        ("Side_Detect_L_Stat", 5),
        ("Side_Detect_R_Stat", 5),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).main)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("ACCDATA", 50),
      ("ACCDATA_2", 50),
      ("ACCDATA_3", 5),
      ("IPMA_Data", 1),
    ]

    if CP.enableBsm and CP.flags & FordFlags.CANFD:
      messages += [
        ("Side_Detect_L_Stat", 5),
        ("Side_Detect_R_Stat", 5),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).camera)
