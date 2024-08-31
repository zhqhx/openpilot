#!/usr/bin/env python3
import os
import time
import json
import argparse
import threading
from dataclasses import dataclass
from inputs import devices, get_gamepad

import cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.realtime import Ratekeeper
from openpilot.common.numpy_fast import interp, clip
from openpilot.common.params import Params
from openpilot.tools.lib.kbhit import KBHit


@dataclass
class Gamepad:
  cancel_button: str  # west button
  accel_axis: str  # right trigger or right thumb stick, vertical
  neg_accel_axis: str | None  # left trigger, optional
  steer_axis: str  # right thumb stick, horizontal


JS_EXPO = 0.4
GAMEPADS = {
  'Sony Interactive Entertainment DualSense Wireless Controller': Gamepad('BTN_WEST', 'ABS_RZ', 'ABS_Z', 'ABS_RX'),
  'Generic X-Box pad': Gamepad('BTN_NORTH', 'ABS_RZ', 'ABS_Z', 'ABS_RX'),
}
CONFIG_FILE = os.path.join(BASEDIR, 'tools', 'joystick', 'config.json')


class Keyboard:
  def __init__(self):
    self.kb = KBHit()
    self.axis_increment = 0.05  # 5% of full actuation each key press
    self.axes_map = {'w': 'gb', 's': 'gb',
                     'a': 'steer', 'd': 'steer'}
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False

  def update(self):
    key = self.kb.getch().lower()
    self.cancel = False
    if key == 'r':
      self.axes_values = {ax: 0. for ax in self.axes_values}
    elif key == 'c':
      self.cancel = True
    elif key in self.axes_map:
      axis = self.axes_map[key]
      incr = self.axis_increment if key in ['w', 'a'] else -self.axis_increment
      self.axes_values[axis] = clip(self.axes_values[axis] + incr, -1, 1)
    else:
      return False
    return True


class Joystick:
  def __init__(self):
    # Check for present and supported gamepad, some use different axes
    if len(devices.gamepads) == 0:
      raise Exception("No gamepad found")
    if (gamepad := devices.gamepads[0]).name not in GAMEPADS:
      raise Exception(f"Gamepad not supported, add to GAMEPADS: '{gamepad.name}'")

    self.config = GAMEPADS[gamepad.name]

    self.min_axis_value = {self.config.accel_axis: 0., self.config.steer_axis: 0.}
    self.max_axis_value = {self.config.accel_axis: 255., self.config.steer_axis: 255.}
    self.axes_values = {self.config.accel_axis: 0., self.config.steer_axis: 0.}
    self.axes_order = [self.config.accel_axis, self.config.steer_axis]
    self.cancel = False

  def update(self):
    joystick_event = get_gamepad()[0]
    event = (joystick_event.code, joystick_event.state)

    # remap second accel axis to negative accel if present
    if event[0] == self.config.neg_accel_axis:
      print(event)
      event = (self.config.accel_axis, -event[1])

    if event[0] == self.config.cancel_button:
      if event[1] == 1:
        self.cancel = True
      elif event[1] == 0:   # state 0 is falling edge
        self.cancel = False
    elif event[0] in self.axes_values:
      self.max_axis_value[event[0]] = max(event[1], self.max_axis_value[event[0]])
      self.min_axis_value[event[0]] = min(event[1], self.min_axis_value[event[0]])

      norm = -interp(event[1], [self.min_axis_value[event[0]], self.max_axis_value[event[0]]], [-1., 1.])
      norm = norm if abs(norm) > 0.02 else 0.  # center can be noisy, deadzone of 2%
      self.axes_values[event[0]] = JS_EXPO * norm ** 3 + (1 - JS_EXPO) * norm  # less action near center for fine control
    else:
      return False
    return True


def send_thread(joystick):
  joystick_sock = messaging.pub_sock('testJoystick')
  rk = Ratekeeper(100, print_delay_threshold=None)
  while 1:
    dat = messaging.new_message('testJoystick')
    dat.testJoystick.axes = [joystick.axes_values[a] for a in joystick.axes_order]
    dat.testJoystick.buttons = [joystick.cancel]
    joystick_sock.send(dat.to_bytes())
    print('\n' + ', '.join(f'{name}: {round(v, 3)}' for name, v in joystick.axes_values.items()))
    rk.keep_time()


def joystick_thread(joystick):
  Params().put_bool('JoystickDebugMode', True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while True:
    joystick.update()


def clear_queue():
  time.sleep(1)
  # t = time.monotonic()
  # while (time.monotonic() - t) < 1:
  #   get_gamepad()


def calibrate_gamepad():
  print('\n⚠️ Calibrating gamepad, press a button to bind to CANCEL...')
  while not (event := get_gamepad()[0]).code.startswith('BTN_'):
    pass
  cancel_button = event.code
  print(f'✅ Recorded CANCEL button: {cancel_button}')
  clear_queue()

  print('\nMove the right thumb stick to the right to bind to STEER...')
  # note that we get a lot of spurious events with noisy trigger inputs, so we need to wait for a large diff:
  while not ((event := get_gamepad()[0]).code.startswith('ABS_') and abs(event.state) > 200):
    pass
  steer_axis = event.code
  print(f'✅ Recorded STEER axis: {steer_axis}')

  clear_queue()

  print('\nMove the left thumb stick up to bind to ACCEL...')
  while not ((event := get_gamepad()[0]).code.startswith('ABS_') and abs(event.state) > 200):
    pass
  accel_axis = event.code
  print(f'✅ Recorded ACCEL axis: {accel_axis}')
  print('✅ Done!')


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                               'openpilot must be offroad before starting joysticked.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--keyboard', action='store_true', help='Use your keyboard instead of a joystick')
  args = parser.parse_args()

  # if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
  #   print("The car must be off before running joystickd.")
  #   exit()

  print()
  if args.keyboard:
    print('Gas/brake control: `W` and `S` keys')
    print('Steering control: `A` and `D` keys')
    print('Buttons')
    print('- `R`: Resets axes')
    print('- `C`: Cancel cruise control')
  else:
    print('Using joystick, make sure to run cereal/messaging/bridge on your device if running over the network!')

    if not os.path.exists(CONFIG_FILE):
      calibrate_gamepad()

  # joystick = Keyboard() if args.keyboard else Joystick()
  # joystick_thread(joystick)
