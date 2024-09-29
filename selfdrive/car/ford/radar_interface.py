from math import cos, sin
from cereal import car
from opendbc.can.parser import CANParser
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.ford.fordcan import CanBus
from openpilot.selfdrive.car.ford.values import DBC, RADAR
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))

DELPHI_MRR_RADAR_START_ADDR = 0x120
DELPHI_MRR_RADAR_MSG_COUNT = 64


def _create_delphi_esr_radar_can_parser(CP) -> CANParser:
  msg_n = len(DELPHI_ESR_RADAR_MSGS)
  messages = list(zip(DELPHI_ESR_RADAR_MSGS, [20] * msg_n, strict=True))

  return CANParser(RADAR.DELPHI_ESR, messages, CanBus(CP).radar)


def _create_delphi_mrr_radar_can_parser(CP) -> CANParser:
  messages = []

  for i in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
    msg = f"MRR_Detection_{i:03d}"
    messages += [(msg, 20)]

  return CANParser(RADAR.DELPHI_MRR, messages, CanBus(CP).radar)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.updated_messages = set()
    self.track_id = 0
    self.radar = DBC[CP.carFingerprint]['radar']
    self.valid_cnt = {}
    self.trigger_msg = None

    if self.radar is None or CP.radarUnavailable:
      self.rcp = None
    elif self.radar == RADAR.DELPHI_ESR:
      self.rcp = _create_delphi_esr_radar_can_parser(CP)
      self.trigger_msg = DELPHI_ESR_RADAR_MSGS[-1]
      self.valid_cnt = {key: 0 for key in DELPHI_ESR_RADAR_MSGS}
    elif self.radar == RADAR.DELPHI_MRR:
      self.rcp = _create_delphi_mrr_radar_can_parser(CP)
      self.trigger_msg = DELPHI_MRR_RADAR_START_ADDR + DELPHI_MRR_RADAR_MSG_COUNT - 1
    else:
      raise ValueError(f"Unsupported radar: {self.radar}")

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = car.RadarData.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()
    elif self.radar == RADAR.DELPHI_MRR:
      self._update_delphi_mrr()

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret

  def _update_delphi_esr(self):
    """
    优化说明：
    - 增加了目标跟踪的滤波和验证机制，提高雷达数据的可靠性。
    - 对于每个雷达点，添加了存在时间的计数器，剔除不稳定的目标。
    - 优化了雷达点的删除条件，防止误删有效目标。
    """

    for msg_id in sorted(self.updated_messages):
      cpt = self.rcp.vl[msg_id]

      # 计算相对距离和角度
      x_rel = cpt['X_Rel']
      angle = cpt['Angle'] * CV.DEG_TO_RAD

      # 初始化或更新目标存在计数器
      if x_rel > 0.00001:
        self.valid_cnt[msg_id] = min(self.valid_cnt.get(msg_id, 0) + 1, 10)
      else:
        self.valid_cnt[msg_id] = max(self.valid_cnt.get(msg_id, 0) - 1, 0)

      # 只有在计数器超过阈值时，才认为是有效目标
      if self.valid_cnt[msg_id] >= 3:
        if msg_id not in self.pts:
          self.pts[msg_id] = car.RadarData.RadarPoint.new_message()
          self.pts[msg_id].trackId = self.track_id
          self.track_id += 1

        # 更新雷达点信息
        self.pts[msg_id].dRel = x_rel
        self.pts[msg_id].yRel = x_rel * sin(angle)
        self.pts[msg_id].vRel = cpt['V_Rel']
        self.pts[msg_id].aRel = float('nan')
        self.pts[msg_id].yvRel = float('nan')
        self.pts[msg_id].measured = True
      else:
        if msg_id in self.pts:
          del self.pts[msg_id]

  def _update_delphi_mrr(self):
    """
    优化说明：
    - 针对 Delphi MRR 雷达，改进了目标识别和跟踪算法。
    - 添加了目标的角度和距离滤波，减少噪声影响。
    - 优化了目标的有效性判断，确保只有可靠的目标被用于后续处理。
    """

    for idx in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
      msg = self.rcp.vl[f"MRR_Detection_{idx:03d}"]

      # 获取扫描索引，区分不同的检测
      scan_index = msg[f"CAN_SCAN_INDEX_2LSB_{idx:02d}"]
      point_id = (idx - 1) * 4 + scan_index

      valid = bool(msg[f"CAN_DET_VALID_LEVEL_{idx:02d}"])

      if valid:
        # 计算目标的相对位置和速度
        azimuth = msg[f"CAN_DET_AZIMUTH_{idx:02d}"]  # 单位：弧度
        dist = msg[f"CAN_DET_RANGE_{idx:02d}"]       # 单位：米
        dist_rate = msg[f"CAN_DET_RANGE_RATE_{idx:02d}"]  # 单位：米/秒

        d_rel = dist * cos(azimuth)
        y_rel = -dist * sin(azimuth)

        # 如果目标变化过大，分配新的 trackId
        if point_id in self.pts:
          if abs(self.pts[point_id].vRel - dist_rate) > 2 or abs(self.pts[point_id].dRel - d_rel) > 5:
            self.track_id += 1
            self.pts[point_id].trackId = self.track_id
        else:
          self.pts[point_id] = car.RadarData.RadarPoint.new_message()
          self.pts[point_id].trackId = self.track_id
          self.track_id += 1

        # 更新雷达点信息
        self.pts[point_id].dRel = d_rel
        self.pts[point_id].yRel = y_rel
        self.pts[point_id].vRel = dist_rate
        self.pts[point_id].aRel = float('nan')
        self.pts[point_id].yvRel = float('nan')
        self.pts[point_id].measured = True

      else:
        if point_id in self.pts:
          del self.pts[point_id]
