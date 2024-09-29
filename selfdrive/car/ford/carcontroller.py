from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.ford import fordcan
from openpilot.selfdrive.car.ford.values import CarControllerParams, FordFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX

LongCtrlState = car.CarControl.Actuators.LongControlState
VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.CAN = fordcan.CanBus(CP)
    self.frame = 0

    self.apply_curvature_last = 0.0
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None

    # 初始化 PID 控制器的状态变量
    self.curvature_integral = 0.0
    self.curvature_error_last = 0.0
    self.DT_CTRL = 0.01  # 控制循环时间步长，单位为秒

    # 定义弯道减速的参数
    self.MIN_SPEED = 5.0  # 最低速度，单位为 m/s
    self.CURVATURE_SPEED_FACTOR = 50.0  # 曲率影响系数，可根据实际情况调整

  def apply_ford_curvature_limits(self, apply_curvature, apply_curvature_last, current_curvature, v_ego_raw):
    # 定义 PID 控制器参数
    Kp = 0.1
    Ki = 0.01
    Kd = 0.01

    # 计算曲率误差
    curvature_error = apply_curvature - current_curvature

    # 更新积分项
    self.curvature_integral += curvature_error * self.DT_CTRL
    # 防止积分项过大（积分防饱和）
    self.curvature_integral = clip(self.curvature_integral, -0.1, 0.1)

    # 计算微分项
    curvature_derivative = (curvature_error - self.curvature_error_last) / self.DT_CTRL
    self.curvature_error_last = curvature_error

    # 计算 PID 输出
    pid_output = (Kp * curvature_error) + (Ki * self.curvature_integral) + (Kd * curvature_derivative)

    # 更新目标曲率
    apply_curvature = apply_curvature_last + pid_output

    # 限制曲率范围
    apply_curvature = clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)

    return apply_curvature

  def update(self, CC, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### ACC 按钮处理 ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### 横向控制 ###
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        # 获取当前曲率
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
        # 应用改进的曲率限制函数
        apply_curvature = self.apply_ford_curvature_limits(actuators.curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw)
      else:
        apply_curvature = 0.0
        # 如果横向控制未激活，重置 PID 控制器状态
        self.curvature_integral = 0.0
        self.curvature_error_last = 0.0

      self.apply_curvature_last = apply_curvature

      if self.CP.flags & FordFlags.CANFD:
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0.0, 0.0, -apply_curvature, 0.0, counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0.0, 0.0, -apply_curvature, 0.0))

    # 发送 LKA 消息
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### 纵向控制 ###
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # 获取当前曲率的绝对值
      current_curvature = abs(self.apply_curvature_last)
      # 计算基于曲率的目标速度
      desired_speed = max(V_CRUISE_MAX - (current_curvature * self.CURVATURE_SPEED_FACTOR), self.MIN_SPEED)
      # 计算需要的加速度
      accel_curvature_adjusted = (desired_speed - CS.out.vEgo) / self.DT_CTRL
      # 取规划器加速度和曲率调整加速度的最小值
      accel = min(actuators.accel, accel_curvature_adjusted)
      # 限制加速度范围
      accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      # 计算油门和制动
      gas = accel
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, v_ego_kph=V_CRUISE_MAX))

    ### UI 显示 ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, hud_control,
                                                 CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.copy()
    new_actuators.curvature = self.apply_curvature_last

    self.frame += 1
    return new_actuators, can_sends
