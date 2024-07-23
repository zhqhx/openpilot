#!/usr/bin/env python3
import os
import gc
import math
import time
import ctypes
import numpy as np
from pathlib import Path

from cereal import messaging, car
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.modeld.runners import ModelRunner, Runtime
from openpilot.selfdrive.modeld.models.commonmodel_pyx import sigmoid

REG_SCALE = 0.25
MODEL_WIDTH = 1440
MODEL_HEIGHT = 960
FEATURE_LEN = 1536
CONTEXT_LEN = 100
OUTPUT_SIZE = 82 + 10 + FEATURE_LEN
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')
MODEL_PATHS = {
  ModelRunner.SNPE: Path(__file__).parent / 'models/dmonitoring_model_q.dlc',
  ModelRunner.ONNX: Path(__file__).parent / 'models/dmonitoring_model.onnx'}

class DriverStateResult(ctypes.Structure):
  _fields_ = [
    ("face_orientation", ctypes.c_float*3),
    ("face_position", ctypes.c_float*3),
    ("face_orientation_std", ctypes.c_float*3),
    ("face_position_std", ctypes.c_float*3),
    ("face_prob", ctypes.c_float),
    ("_unused_a", ctypes.c_float*8),
    ("left_eye_prob", ctypes.c_float),
    ("_unused_b", ctypes.c_float*8),
    ("right_eye_prob", ctypes.c_float),
    ("left_blink_prob", ctypes.c_float),
    ("right_blink_prob", ctypes.c_float),
    ("sunglasses_prob", ctypes.c_float),
    ("occluded_prob", ctypes.c_float),
    ("ready_prob", ctypes.c_float*4),
    ("not_ready_prob", ctypes.c_float*2)]

class DMonitoringModelResult(ctypes.Structure):
  _fields_ = [
    ("driver_state_lhd", DriverStateResult),
    ("driver_state_rhd", DriverStateResult),
    ("alerts_prob", ctypes.c_float*6),
    ("awareness", ctypes.c_float*2),
    ("rhd_prob", ctypes.c_float),
    ("active_mode_prob", ctypes.c_float),
    ("feature", ctypes.c_float*FEATURE_LEN)]

class ModelState:
  inputs: dict[str, np.ndarray]
  output: np.ndarray
  model: ModelRunner

  def __init__(self):
    assert ctypes.sizeof(DMonitoringModelResult) == OUTPUT_SIZE * ctypes.sizeof(ctypes.c_float)
    self.output = np.zeros(OUTPUT_SIZE, dtype=np.float32)
    self.inputs = {
      'input_img': np.zeros(MODEL_HEIGHT * MODEL_WIDTH, dtype=np.uint8),
      'features': np.zeros((CONTEXT_LEN-1) * FEATURE_LEN, dtype=np.float32),
      'awareness': np.zeros(1, dtype=np.float32),
      'enabled': np.zeros(1, dtype=np.float32),
      'steering_pressed': np.zeros(1, dtype=np.float32),
      'gas_pressed': np.zeros(1, dtype=np.float32),
      'wrong_gear': np.zeros(1, dtype=np.float32),
      'rpy_calib': np.zeros(3, dtype=np.float32),
      'v_ego': np.zeros(1, dtype=np.float32),
      'standstill': np.zeros(1, dtype=np.float32),
      'brake_disengage_prob': np.zeros(1, dtype=np.float32)}

    self.model = ModelRunner(MODEL_PATHS, self.output, Runtime.DSP, True, None)
    for name in self.inputs.keys():
      self.model.addInput(name, self.inputs[name] if name != 'input_img' else None)

    self.inputs['awareness'] = 1.

  def run(self, buf:VisionBuf, inputs: dict[str, np.ndarray], history_valid: bool) -> tuple[DMonitoringModelResult, float]:
    v_offset = buf.height - MODEL_HEIGHT
    h_offset = (buf.width - MODEL_WIDTH) // 2
    buf_data = buf.data.reshape(-1, buf.stride)
    input_data = self.inputs['input_img'].reshape(MODEL_HEIGHT, MODEL_WIDTH)
    input_data[:] = buf_data[v_offset:v_offset+MODEL_HEIGHT, h_offset:h_offset+MODEL_WIDTH]

    for name in inputs.keys():
      self.inputs[name] = inputs[name]

    t1 = time.perf_counter()
    self.model.setInputBuffer("input_img", self.inputs['input_img'].view(np.float32))
    self.model.execute()
    t2 = time.perf_counter()

    model_result = ctypes.cast(self.output.ctypes.data, ctypes.POINTER(DMonitoringModelResult)).contents

    self.inputs['features'][:-FEATURE_LEN] = self.inputs['features'][FEATURE_LEN:]
    self.inputs['features'][-FEATURE_LEN:] = model_result.feature

    if history_valid:
      self.inputs['awareness'] = model_result.awareness[0]

    return model_result, t2 - t1


def fill_driver_state(msg, ds_result: DriverStateResult):
  msg.faceOrientation = [x * REG_SCALE for x in ds_result.face_orientation]
  msg.faceOrientationStd = [math.exp(x) for x in ds_result.face_orientation_std]
  msg.facePosition = [x * REG_SCALE for x in ds_result.face_position[:2]]
  msg.facePositionStd = [math.exp(x) for x in ds_result.face_position_std[:2]]
  msg.faceProb = sigmoid(ds_result.face_prob)
  msg.leftEyeProb = sigmoid(ds_result.left_eye_prob)
  msg.rightEyeProb = sigmoid(ds_result.right_eye_prob)
  msg.leftBlinkProb = sigmoid(ds_result.left_blink_prob)
  msg.rightBlinkProb = sigmoid(ds_result.right_blink_prob)
  msg.sunglassesProb = sigmoid(ds_result.sunglasses_prob)
  msg.occludedProb = sigmoid(ds_result.occluded_prob)
  msg.readyProb = [sigmoid(x) for x in ds_result.ready_prob]
  msg.notReadyProb = [sigmoid(x) for x in ds_result.not_ready_prob]

def fill_driverstatev2_packet(msg):
  # ds = msg.driverStateV2
  # ds.frameId = frame_id
  # ds.modelExecutionTime = execution_time
  # ds.dspExecutionTime = dsp_execution_time
  # ds.poorVisionProb = sigmoid(model_result.poor_vision_prob)
  # ds.wheelOnRightProb = sigmoid(model_result.wheel_on_right_prob)
  # ds.rawPredictions = model_output.tobytes() if SEND_RAW_PRED else b''
  # fill_driver_state(ds.leftDriverData, model_result.driver_state_lhd)
  # fill_driver_state(ds.rightDriverData, model_result.driver_state_rhd)
  return msg

def fill_monitoringstate_packet(msg):
  return msg


def main():
  gc.disable()
  set_realtime_priority(1)

  model = ModelState()
  cloudlog.warning("models loaded, dmonitoringmodeld starting")
  Params().put_bool("DmModelInitialized", True)

  cloudlog.warning("connecting to driver stream")
  vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_DRIVER, True)
  while not vipc_client.connect(False):
    time.sleep(0.1)
  assert vipc_client.is_connected()
  cloudlog.warning(f"connected with buffer size: {vipc_client.buffer_len}")

  sm = SubMaster(["liveCalibration", "carState", "controlsState", "modelV2"])
  pm = PubMaster(["driverMonitoringState", "driverStateV2"])

  frames_run = 0

  while True:
    buf = vipc_client.recv()
    if buf is None:
      continue

    sm.update(0)

    sm_valid = sm.all_checks()
    history_valid = frames_run >= CONTEXT_LEN - 1

    if sm_valid:
      inputs: dict[str, np.ndarray] = {
        'enabled': np.array(sm['controlsState'].enabled, dtype=np.float32),
        'steering_pressed': np.array(sm['carState'].steeringPressed, dtype=np.float32),
        'gas_pressed': np.array(sm['carState'].gasPressed, dtype=np.float32),
        'wrong_gear': np.array(sm['carState'].gearShifter in [car.CarState.GearShifter.reverse, car.CarState.GearShifter.park], dtype=np.float32),
        'rpy_calib': np.array(sm['liveCalibration'].rpyCalib, dtype=np.float32),
        'v_ego': np.array(sm['carState'].vEgo, dtype=np.float32),
        'standstill': np.array(sm['carState'].standstill, dtype=np.float32),
        'brake_disengage_prob': np.array(sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs[0], dtype=np.float32),
      }

    t1 = time.perf_counter()
    if sm_valid:
      model_result, dsp_execution_time = model.run(buf, inputs, history_valid)
      frames_run += 1
    t2 = time.perf_counter()

    # TODO: combine into new msg?
    driverstate_msg = messaging.new_message('driverStateV2', valid=sm_valid and history_valid)
    monitoringstate_msg = messaging.new_message('driverMonitoringState', valid=sm_valid and history_valid)

    if driverstate_msg.valid:
      fill_driverstatev2_packet(driverstate_msg)
    if monitoringstate_msg.valid:
      fill_monitoringstate_packet(monitoringstate_msg)

    pm.send("driverStateV2", driverstate_msg)
    pm.send("driverMonitoringState", monitoringstate_msg)


if __name__ == "__main__":
  main()
