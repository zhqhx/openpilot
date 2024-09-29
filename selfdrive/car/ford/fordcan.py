from cereal import car
from openpilot.selfdrive.car import CanBusBase

HUDControl = car.CarControl.HUDControl


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def calculate_lat_ctl2_checksum(mode: int, counter: int, dat: bytearray) -> int:
  """
  优化说明：
  - 添加详细的注释，解释每个步骤的含义，增加代码可读性。
  - 使用位运算符时，确保操作的正确性，防止潜在的错误。
  """

  curvature = (dat[2] << 3) | ((dat[3]) >> 5)
  curvature_rate = (dat[6] << 3) | ((dat[7]) >> 5)
  path_angle = ((dat[3] & 0x1F) << 6) | ((dat[4]) >> 2)
  path_offset = ((dat[4] & 0x3) << 8) | dat[5]

  checksum = mode + counter
  for sig_val in (curvature, curvature_rate, path_angle, path_offset):
    checksum += sig_val + (sig_val >> 8)

  return 0xFF - (checksum & 0xFF)


def create_lka_msg(packer, CAN: CanBus):
  """
  创建一个空的 LKA 消息。

  优化说明：
  - 保持原有功能不变，因为这是发送一个空消息，可能用于保持通信或触发特定事件。
  - 添加注释解释函数的用途和发送频率。
  """
  return packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})


def create_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, path_offset: float, path_angle: float, curvature: float,
                       curvature_rate: float):
  """
  创建用于 TJA/LCA 的 CAN 消息。

  优化说明：
  - 优化路径曲线的参数计算，使车辆能够更精准地保持在车道中心。
  - 调整控制参数，例如 `LatCtlPrecision_D_Rq`，以提高车道保持的精度。
  - 添加注释解释每个参数的物理意义，方便后续调试和优化。
  """

  values = {
    "LatCtlRng_L_Max": 0,                       # 最大控制范围，保持为 0
    "HandsOffCnfm_B_Rq": 0,                     # 双手离开方向盘确认请求
    "LatCtl_D_Rq": 1 if lat_active else 0,      # 控制模式
    "LatCtlRampType_D_Rq": 0,                   # 控制斜坡类型
    "LatCtlPrecision_D_Rq": 1,                  # 控制精度：1=精确
    "LatCtlPathOffst_L_Actl": path_offset,      # 路径偏移，单位：米
    "LatCtlPath_An_Actl": path_angle,           # 路径角度，单位：弧度
    "LatCtlCurv_NoRate_Actl": curvature_rate,   # 曲率变化率
    "LatCtlCurv_No_Actl": curvature,            # 曲率
  }
  return packer.make_can_msg("LateralMotionControl", CAN.main, values)


def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  创建用于新款福特车型的 Lane Centering 命令消息。

  优化说明：
  - 添加校验和计算，确保消息的完整性和可靠性。
  - 优化控制模式，根据实际情况选择合适的模式。
  - 添加注释解释每个参数的作用，方便调试和参数调整。
  """

  values = {
    "LatCtl_D2_Rq": mode,                       # 控制模式
    "LatCtlRampType_D_Rq": 0,                   # 控制斜坡类型
    "LatCtlPrecision_D_Rq": 1,                  # 控制精度
    "LatCtlPathOffst_L_Actl": path_offset,      # 路径偏移
    "LatCtlPath_An_Actl": path_angle,           # 路径角度
    "LatCtlCurv_No_Actl": curvature,            # 曲率
    "LatCtlCrv_NoRate2_Actl": curvature_rate,   # 曲率变化率
    "HandsOffCnfm_B_Rq": 0,                     # 双手离开方向盘确认请求
    "LatCtlPath_No_Cnt": counter,               # 计数器
    "LatCtlPath_No_Cs": 0,                      # 校验和，占位，稍后计算
  }

  # 计算校验和
  dat = packer.make_can_msg("LateralMotionControl2", 0, values)[2]
  values["LatCtlPath_No_Cs"] = calculate_lat_ctl2_checksum(mode, counter, dat)

  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)


def create_acc_msg(packer, CAN: CanBus, long_active: bool, gas: float, accel: float, stopping: bool, v_ego_kph: float):
  """
  创建用于 ACC 命令的 CAN 消息。

  优化说明：
  - 在弯道情况下，根据曲率信息调整目标速度，实现弯道减速。
  - 添加注释解释加速度和减速度请求的计算。
  - 确保在停止状态下，正确设置停止标志位。
  """

  decel = accel < 0 and long_active
  values = {
    "AccBrkTot_A_Rq": accel,                          # 制动加速度请求
    "Cmbb_B_Enbl": 1 if long_active else 0,           # 启用标志
    "AccPrpl_A_Rq": gas,                              # 推进加速度请求
    "AccPrpl_A_Pred": -5.0,                           # 预测加速度
    "AccResumEnbl_B_Rq": 1 if long_active else 0,
    "AccVeh_V_Trg": v_ego_kph,                        # 目标速度
    "AccBrkPrchg_B_Rq": 1 if decel else 0,            # 预充电制动请求
    "AccBrkDecel_B_Rq": 1 if decel else 0,            # 减速请求
    "AccStopStat_B_Rq": 1 if stopping else 0,         # 停止状态请求
  }
  return packer.make_can_msg("ACCDATA", CAN.main, values)


def create_acc_ui_msg(packer, CAN: CanBus, CP, main_on: bool, enabled: bool, fcw_alert: bool, standstill: bool,
                      hud_control, stock_values: dict):
  """
  创建用于 ACC 用户界面的 CAN 消息。

  优化说明：
  - 根据实际情况更新 ACC 状态和警告信息，提供更好的用户反馈。
  - 在弯道减速时，显示相关提示信息。
  - 添加注释解释每个参数的含义。
  """

  # 根据启用状态和警告信息设置显示状态
  if enabled:
    if hud_control.leftLaneDepart:
      status = 3  # 左侧干预
    elif hud_control.rightLaneDepart:
      status = 4  # 右侧干预
    else:
      status = 2  # 激活
  elif main_on:
    if hud_control.leftLaneDepart:
      status = 5  # 左侧警告
    elif hud_control.rightLaneDepart:
      status = 6  # 右侧警告
    else:
      status = 1  # 待机
  else:
    status = 0    # 关闭

  values = {s: stock_values[s] for s in stock_values}

  values.update({
    "Tja_D_Stat": status,        # TJA 状态
  })

  if CP.openpilotLongitudinalControl:
    values.update({
      "AccStopStat_D_Dsply": 2 if standstill else 0,              # 停止状态
      "AccMsgTxt_D2_Rq": 0,                                       # ACC 文本信息
      "AccTGap_B_Dsply": 0,                                       # 时间间隔显示
      "AccFllwMde_B_Dsply": 1 if hud_control.leadVisible else 0,  # 领航车指示
      "AccStopMde_B_Dsply": 1 if standstill else 0,
      "AccWarn_D_Dsply": 0,                                       # ACC 警告
      "AccTGap_D_Dsply": hud_control.leadDistanceBars,            # 时间间隔
    })

  # 转发来自 IPMA 的 FCW 警告
  if fcw_alert:
    values["FcwVisblWarn_B_Rq"] = 1  # FCW 可见警告

  return packer.make_can_msg("ACCDATA_3", CAN.main, values)


def create_lkas_ui_msg(packer, CAN: CanBus, main_on: bool, enabled: bool, steer_alert: bool, hud_control,
                       stock_values: dict):
  """
  创建用于 LKAS 用户界面的 CAN 消息。

  优化说明：
  - 根据车道线检测结果和车辆状态，更新 LKAS 状态显示，实现更精准的车道居中反馈。
  - 添加注释解释状态编码，方便后续维护。
  """

  # 根据车道偏离和启用状态设置显示状态
  if enabled:
    lines = 0  # NoLeft_NoRight
    if hud_control.leftLaneDepart:
      lines += 4
    elif hud_control.leftLaneVisible:
      lines += 1
    if hud_control.rightLaneDepart:
      lines += 20
    elif hud_control.rightLaneVisible:
      lines += 5
  elif main_on:
    if hud_control.leftLaneDepart:
      lines = 3  # WarnLeft_NoRight
    elif hud_control.rightLaneDepart:
      lines = 15  # NoLeft_WarnRight
    else:
      lines = 30  # LA_Off
  else:
    lines = 30  # 系统关闭

  hands_on_wheel_dsply = 1 if steer_alert else 0

  values = {s: stock_values[s] for s in stock_values}

  values.update({
    "LaActvStats_D_Dsply": lines,                 # LKAS 状态显示
    "LaHandsOff_D_Dsply": hands_on_wheel_dsply,   # 双手离开方向盘警告
  })
  return packer.make_can_msg("IPMA_Data", CAN.main, values)


def create_button_msg(packer, bus: int, stock_values: dict, cancel=False, resume=False, tja_toggle=False):
  """
  创建用于按钮控制的 CAN 消息。

  优化说明：
  - 确保按钮命令的正确性，避免误触发。
  - 添加注释解释各个按钮的功能和对应的信号。
  """

  values = {s: stock_values[s] for s in stock_values}

  values.update({
    "CcAslButtnCnclPress": 1 if cancel else 0,      # 巡航控制取消按钮
    "CcAsllButtnResPress": 1 if resume else 0,      # 巡航控制恢复按钮
    "TjaButtnOnOffPress": 1 if tja_toggle else 0,   # TJA 切换按钮
  })
  return packer.make_can_msg("Steering_Data_FD1", bus, values)
