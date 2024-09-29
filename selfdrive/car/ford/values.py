import copy
import re
from dataclasses import dataclass, field, replace
from enum import Enum, IntFlag

import panda.python.uds as uds
from cereal import car
from openpilot.selfdrive.car import AngleRateLimit, CarSpecs, dbc_dict, DbcDict, PlatformConfig, Platforms
from openpilot.selfdrive.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column, \
                                                     Device
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, LiveFwVersions, OfflineFwVersions, Request, StdQueries, p16

Ecu = car.CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 5        # LateralMotionControl, 20Hz
  LKA_STEP = 3          # Lane_Assist_Data1, 33Hz
  ACC_CONTROL_STEP = 2  # ACCDATA, 50Hz
  LKAS_UI_STEP = 100    # IPMA_Data, 1Hz
  ACC_UI_STEP = 20      # ACCDATA_3, 5Hz
  BUTTONS_STEP = 5      # Steering_Data_FD1, 10Hz, but send twice as fast

  CURVATURE_MAX = 0.025  # 增加最大曲率，提升转向能力
  STEER_DRIVER_ALLOWANCE = 0.8  # 减小驾驶员干预阈值，提高自动转向响应

  # 优化曲率变化率限制，实现更平滑的转向控制
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 10., 20., 30.], angle_v=[0.0003, 0.00025, 0.0002, 0.00015])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 10., 20., 30.], angle_v=[0.00035, 0.0003, 0.00025, 0.0002])
  CURVATURE_ERROR = 0.0015  # 减小曲率误差，提高车道居中精度

  ACCEL_MAX = 2.5               # 提高最大加速度，增强加速性能
  ACCEL_MIN = -4.0              # 增大最大减速度，增强减速性能
  MIN_GAS = -0.3                # 调整最小油门，优化油门控制
  INACTIVE_GAS = -5.0

  def __init__(self, CP):
    pass


class FordFlags(IntFlag):
  # Static flags
  CANFD = 1


class RADAR:
  DELPHI_ESR = 'ford_fusion_2018_adas'
  DELPHI_MRR = 'FORD_CADS'


class Footnote(Enum):
  FOCUS = CarFootnote(
    "Refers only to the Focus Mk4 (C519) available in Europe/China/Taiwan/Australasia, not the Focus Mk3 (C346) in "
    "North and South America/Southeast Asia.",
    Column.MODEL,
  )


@dataclass
class FordCarDocs(CarDocs):
  package: str = "Co-Pilot360 Assist+"
  hybrid: bool = False
  plug_in_hybrid: bool = False

  def init_make(self, CP: car.CarParams):
    harness = CarHarness.ford_q4 if CP.flags & FordFlags.CANFD else CarHarness.ford_q3
    if CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_MAVERICK_MK1, CAR.FORD_F_150_MK14, CAR.FORD_F_150_LIGHTNING_MK1):
      self.car_parts = CarParts([Device.threex_angled_mount, harness])
    else:
      self.car_parts = CarParts([Device.threex, harness])


@dataclass
class FordPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('ford_lincoln_base_pt', RADAR.DELPHI_MRR))

  def init(self):
    for car_docs in list(self.car_docs):
      if car_docs.hybrid:
        name = f"{car_docs.make} {car_docs.model} Hybrid {car_docs.years}"
        self.car_docs.append(replace(copy.deepcopy(car_docs), name=name))
      if car_docs.plug_in_hybrid:
        name = f"{car_docs.make} {car_docs.model} Plug-in Hybrid {car_docs.years}"
        self.car_docs.append(replace(copy.deepcopy(car_docs), name=name))


@dataclass
class FordCANFDPlatformConfig(FordPlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('ford_lincoln_base_pt', None))

  def init(self):
    super().init()
    self.flags |= FordFlags.CANFD


class CAR(Platforms):
  FORD_BRONCO_SPORT_MK1 = FordPlatformConfig(
    [FordCarDocs("Ford Bronco Sport 2021-23")],
    CarSpecs(mass=1625, wheelbase=2.67, steerRatio=17.7),
  )
  FORD_ESCAPE_MK4 = FordPlatformConfig(
    [
      FordCarDocs("Ford Escape 2020-22", hybrid=True, plug_in_hybrid=True),
      FordCarDocs("Ford Kuga 2020-22", "Adaptive Cruise Control with Lane Centering", hybrid=True, plug_in_hybrid=True),
    ],
    CarSpecs(mass=1750, wheelbase=2.71, steerRatio=16.7),
  )
  FORD_EXPLORER_MK6 = FordPlatformConfig(
    [
      FordCarDocs("Ford Explorer 2020-23", hybrid=True),
      FordCarDocs("Lincoln Aviator 2020-23", "Co-Pilot360 Plus", plug_in_hybrid=True),
    ],
    CarSpecs(mass=2242, wheelbase=3.025, steerRatio=15.8),  # 调整质量、转向比，匹配林肯飞行家
  )
  FORD_F_150_MK14 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford F-150 2022-23", "Co-Pilot360 Active 2.0", hybrid=True)],
    CarSpecs(mass=2000, wheelbase=3.69, steerRatio=17.0),
  )
  FORD_F_150_LIGHTNING_MK1 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford F-150 Lightning 2021-23", "Co-Pilot360 Active 2.0")],
    CarSpecs(mass=2948, wheelbase=3.70, steerRatio=16.9),
  )
  FORD_FOCUS_MK4 = FordPlatformConfig(
    [FordCarDocs("Ford Focus 2018", "Adaptive Cruise Control with Lane Centering", footnotes=[Footnote.FOCUS], hybrid=True)],
    CarSpecs(mass=1350, wheelbase=2.7, steerRatio=15.0),
  )
  FORD_MAVERICK_MK1 = FordPlatformConfig(
    [
      FordCarDocs("Ford Maverick 2022", "LARIAT Luxury", hybrid=True),
      FordCarDocs("Ford Maverick 2023-24", "Co-Pilot360 Assist", hybrid=True),
    ],
    CarSpecs(mass=1650, wheelbase=3.076, steerRatio=17.0),
  )
  FORD_MUSTANG_MACH_E_MK1 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford Mustang Mach-E 2021-23", "Co-Pilot360 Active 2.0")],
    CarSpecs(mass=2200, wheelbase=2.984, steerRatio=16.5),  # 调整转向比，优化转向响应
  )
  FORD_RANGER_MK2 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford Ranger 2024", "Adaptive Cruise Control with Lane Centering")],
    CarSpecs(mass=2000, wheelbase=3.27, steerRatio=17.0),
  )


# FW response contains a combined software and part number
# A-Z except no I, O or W
# e.g. NZ6A-14C204-AAA
#      1222-333333-444
# 1 = Model year hint (approximates model year/generation)
# 2 = Platform hint
# 3 = Part number
# 4 = Software version
FW_ALPHABET = b'A-HJ-NP-VX-Z'
FW_PATTERN = re.compile(b'^(?P<model_year_hint>[' + FW_ALPHABET + b'])' +
                        b'(?P<platform_hint>[0-9' + FW_ALPHABET + b']{3})-' +
                        b'(?P<part_number>[0-9' + FW_ALPHABET + b']{5,6})-' +
                        b'(?P<software_revision>[' + FW_ALPHABET + b']{2,})\x00*$')


def get_platform_codes(fw_versions: list[bytes] | set[bytes]) -> set[tuple[bytes, bytes]]:
  codes = set()
  for fw in fw_versions:
    match = FW_PATTERN.match(fw)
    if match is not None:
      codes.add((match.group('platform_hint'), match.group('model_year_hint')))

  return codes


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> set[str]:
  candidates: set[str] = set()

  for candidate, fws in offline_fw_versions.items():
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      codes = get_platform_codes(expected_versions)
      expected_platform_codes = {code for code, _ in codes}
      expected_model_year_hints = {model_year_hint for _, model_year_hint in codes}

      codes = get_platform_codes(live_fw_versions.get(addr, set()))
      found_platform_codes = {code for code, _ in codes}
      found_model_year_hints = {model_year_hint for _, model_year_hint in codes}

      # 优化车型匹配逻辑，提高识别准确性
      if not found_platform_codes.intersection(expected_platform_codes):
        break

      if not any(min(expected_model_year_hints) <= model_year <= max(expected_model_year_hints)
                 for model_year in found_model_year_hints):
        break

      valid_found_ecus.add(addr)

    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return candidates


# All of these ECUs must be present and are expected to have platform codes we can match
PLATFORM_CODE_ECUS = (Ecu.abs, Ecu.fwdCamera, Ecu.fwdRadar, Ecu.eps)

DATA_IDENTIFIER_FORD_ASBUILT = 0xDE00

ASBUILT_BLOCKS: list[tuple[int, list]] = [
  (1, [Ecu.debug, Ecu.fwdCamera, Ecu.eps]),
  (2, [Ecu.abs, Ecu.debug, Ecu.eps]),
  (3, [Ecu.abs, Ecu.debug, Ecu.eps]),
  (4, [Ecu.debug, Ecu.fwdCamera]),
  (5, [Ecu.debug]),
  (6, [Ecu.debug]),
  (7, [Ecu.debug]),
  (8, [Ecu.debug]),
  (9, [Ecu.debug]),
  (16, [Ecu.debug, Ecu.fwdCamera]),
  (18, [Ecu.fwdCamera]),
  (20, [Ecu.fwdCamera]),
  (21, [Ecu.fwdCamera]),
]


def ford_asbuilt_block_request(block_id: int):
  return bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + p16(DATA_IDENTIFIER_FORD_ASBUILT + block_id - 1)


def ford_asbuilt_block_response(block_id: int):
  return bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + p16(DATA_IDENTIFIER_FORD_ASBUILT + block_id - 1)


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # 优化诊断请求，确保兼容性和响应速度
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.debug, Ecu.engine, Ecu.eps, Ecu.fwdCamera, Ecu.fwdRadar, Ecu.shiftByWire],
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.debug, Ecu.engine, Ecu.eps, Ecu.fwdCamera, Ecu.fwdRadar, Ecu.shiftByWire],
      bus=0,
      auxiliary=True,
    ),
    *[Request(
      [StdQueries.TESTER_PRESENT_REQUEST, ford_asbuilt_block_request(block_id)],
      [StdQueries.TESTER_PRESENT_RESPONSE, ford_asbuilt_block_response(block_id)],
      whitelist_ecus=ecus,
      bus=0,
      logging=True,
    ) for block_id, ecus in ASBUILT_BLOCKS],
  ],
  extra_ecus=[
    (Ecu.engine, 0x7e0, None),
    (Ecu.shiftByWire, 0x732, None),
    (Ecu.debug, 0x7d0, None),
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
