#!/usr/bin/env python3

import os, sys, struct, array, time
from fcntl import ioctl

from common import IterableObject, Blackboard

class Gamepad(IterableObject):
  def __init__(self):
    super(Gamepad, self).__init__()
    self._blackboard = Blackboard()

    self._blackboard["axis_names"] = {
      # These constants were borrowed from linux/input.h
      0x00 : 'lx', # modified to match PSJoystick
      0x01 : 'ly', # modified to match PSJoystick
      0x02 : 'lz',
      0x03 : 'rx', # modified to match PSJoystick
      0x04 : 'ry', # modified to match PSJoystick
      0x05 : 'rz',
      0x06 : 'trottle',
      0x07 : 'rudder',
      0x08 : 'wheel',
      0x09 : 'gas',
      0x0a : 'brake',
      0x10 : 'dpadx', # modified to match PSJoystick
      0x11 : 'dpady', # modified to match PSJoystick
      0x12 : 'hat1x',
      0x13 : 'hat1y',
      0x14 : 'hat2x',
      0x15 : 'hat2y',
      0x16 : 'hat3x',
      0x17 : 'hat3y',
      0x18 : 'pressure',
      0x19 : 'distance',
      0x1a : 'tilt_x',
      0x1b : 'tilt_y',
      0x1c : 'tool_width',
      0x20 : 'volume',
      0x28 : 'misc',
    }
    self._blackboard["axis_map"] = []
    self._blackboard["axis_states"] = {}

    self._blackboard["button_names"] = {
      0x120 : 'trigger',
      0x121 : 'thumb',
      0x122 : 'thumb2',
      0x123 : 'top',
      0x124 : 'top2',
      0x125 : 'pinkie',
      0x126 : 'base',
      0x127 : 'base2',
      0x128 : 'base3',
      0x129 : 'base4',
      0x12a : 'base5',
      0x12b : 'base6',
      0x12f : 'dead',
      0x130 : 'a',
      0x131 : 'b',
      0x132 : 'c',
      0x133 : 'x',
      0x134 : 'y',
      0x135 : 'z',
      0x136 : 'L1', # modified to match PSJoystick
      0x137 : 'R1', # modified to match PSJoystick
      0x138 : 'tl2',
      0x139 : 'tr2',
      0x13a : 'select',
      0x13b : 'start',
      0x13c : 'mode',
      0x13d : 'thumbl',
      0x13e : 'thumbr',
      0x220 : 'dpad_up',
      0x221 : 'dpad_down',
      0x222 : 'dpad_left',
      0x223 : 'dpad_right',
      # XBox 360 controller uses these codes.
      0x2c0 : 'dpad_left',
      0x2c1 : 'dpad_right',
      0x2c2 : 'dpad_up',
      0x2c3 : 'dpad_down',
    }
    self._blackboard["button_map"] = []
    self._blackboard["button_states"] = {}

    self._blackboard["js_device"] = ""
    # self._blackboard["js_object"] = None
    self._js_object = None # skip lock protection / overhead for now
    self._blackboard["js_name"] = ""

    self.external_blackboard = None

  def do_init(self, *args, **kwargs):
    tries = 0
    while not os.path.isdir('/dev/input') and tries < 500:
      tries += 1
      time.sleep(0.5)

    # Iterate over the joystick devices
    tries = 0
    while len(self._blackboard["js_device"]) == 0 and tries < 10:
      print('finding joystick devices')
      for fn in os.listdir('/dev/input'):
        if fn.startswith('js'):
          self._blackboard["js_device"] = "/dev/input/" + fn
      tries += 1
      time.sleep(1.0)

    if len(self._blackboard["js_device"]) == 0:
      raise Exception("js device not found!")

    # Open the joystick device.
    print('Opening %s...' % self._blackboard["js_device"])
    js_object = open(
      self._blackboard["js_device"], 'rb')

    # Get the device name.
    #buf = bytearray(63)
    buf = array.array('B', [0] * 64)
    ioctl(js_object,
      0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
    self._blackboard["js_name"] = buf.tobytes().rstrip(
      b'\x00').decode('utf-8')
    print('Device name: %s' % self._blackboard["js_name"])

    # Get the axis map.
    buf = array.array('B', [0])
    ioctl(js_object, 0x80016a11, buf) # JSIOCGAXES
    num_axes = buf[0]
    buf = array.array('B', [0] * 0x40)
    ioctl(js_object, 0x80406a32, buf) # JSIOCGAXMAP
    for axis in buf[:num_axes]:
      axis_name = self._blackboard["axis_names"].get(
        axis, 'unknown(0x%02x)' % axis)
      self._blackboard["axis_map"].append(axis_name)
      self._blackboard["axis_states"][axis_name] = 0.0
    # print('%d axes found: %s' % (num_axes,
    #   ', '.join(self._blackboard["axis_map"])))

    # Get the button map.
    buf = array.array('B', [0])
    ioctl(js_object, 0x80016a12, buf) # JSIOCGBUTTONS
    num_buttons = buf[0]
    buf = array.array('H', [0] * 200)
    ioctl(js_object, 0x80406a34, buf) # JSIOCGBTNMAP
    for btn in buf[:num_buttons]:
      btn_name = self._blackboard["button_names"].get(
        btn, 'unknown(0x%03x)' % btn)
      self._blackboard["button_map"].append(btn_name)
      self._blackboard["button_states"][btn_name] = 0
    # print('%d buttons found: %s' % (num_buttons,
    #   ', '.join(button_map)))

    print('%d axes found, %d buttons found' % (
      num_axes, num_buttons))

    # self._blackboard["js_object"] = js_object
    self.js_object = js_object

    self.external_blackboard = None
    self.init_external_blackboard(*args)

  def init_external_blackboard(self, *args):
    # for child classes to override

    # base class
    # expect args[0] to be dict
    # exposes 1 str
    # and sets member var
    if type(args[0]) is not dict:
      raise Exception("external_blackboard not dict")
    self.external_blackboard = args[0]
    self.external_blackboard["gamepad_buffer"] = ""

  def produce(self, k, v):
    # for child classes to override

    # base class is pass-through to buffer
    self.external_blackboard["gamepad_buffer"] =\
      k + "," + str(v) + "\n"
    print("gamepad_buffer",
      self.external_blackboard["gamepad_buffer"])

  def teardown(self, blackboard):
    # for child classes to override

    blackboard["done"].acquire()
    blackboard["done_queue"].clear()
    blackboard["done"].notify_all()
    blackboard["done"].release()

    # this will not avoid the race condition
    # where an Event id dispatch'd
    # before / while shutdown happens
    # so there will be 1 hanging thread
    # that keeps program alive
    # this is why Events need to be
    # as small in scope as possible / cooperative

    # in this case, close'ing the underlying blocking
    # process will force that event to end
    # blackboard["gamepad"].js_object.close()
    self.js_object.close()

  def do_iterate(self, *args, **kwargs):
    evbuf = self.js_object.read(8)

    # this blocks, which is a good / bad thing
    # good in that in prevents error case 2
    # bad in that it causes error case 1

    if not evbuf:
      return

    t, v, ty, n = struct.unpack('IhBB', evbuf)

    if ty & 0x80:
      # initial event, ignore
      pass
    elif ty & 0x01: # btn
      if n < 0 or n >= len(self._blackboard["button_map"]):
        return
      btn_name = self._blackboard["button_map"][n]

      self._blackboard["button_states"][btn_name] = v
      # print("%s, %d" % (btn_name, v))

      if btn_name == "mode" and v == 0:
        '''
        # print("poking done")
        args[0].blackboard["done"].acquire()
        args[0].blackboard["done_queue"].clear()
        args[0].blackboard["done"].notify_all()
        args[0].blackboard["done"].release()

        # this will not avoid the race condition
        # where an Event id dispatch'd
        # before / while shutdown happens
        # so there will be 1 hanging thread
        # that keeps program alive
        # this is why Events need to be
        # as small in scope as possible / cooperative

        # in this case, close'ing the underlying blocking
        # process will force that event to end
        args[0].blackboard["gamepad"].js_object.close()

        # note that we are closing the blackboard's js_object
        # and NOT self.js_object
        # maybe this is the same
        # or maybe this is reasonable
        # because this shared resource is used in another 
        # EventThread, #notsure
        '''

        self.teardown(args[0].blackboard)
        return

      self.produce(btn_name, v)

    elif ty & 0x02: # axis
      if n < 0 or n >= len(self._blackboard["axis_map"]):
        return
      axis = self._blackboard["axis_map"][n]

      fvalue = v / 32767.0
      self._blackboard["axis_states"][axis] = fvalue
      # print("%s: %.3f" % (axis, fvalue))

      # x / y joystick signals are very noisy
      self.produce(axis, round(fvalue, 3))

  def do_cleanup(self):
    self.js_object.close()

if __name__ == '__main__':
  from common import bcolors, nonempty_queue_exists
  from common import IterateEvent, BlackboardQueueCVED

  import argparse
  from threading import Lock, Condition, Thread

  parser = argparse.ArgumentParser(
    description="")
  args = parser.parse_args()

  ############### overhead
  blackboard = {}

  # we purposely use the 'done' set to keep
  # this process alive, and the done_queue
  # being non-empty until some actor / event
  # pop's it empty and notifies 'done'
  # effectively killing the program
  blackboard["done"] = Condition()
  blackboard["done_mutex"] = Lock()
  blackboard["done_queue"] = [1]

  ############### actors
  gamepad = Gamepad()
  gamepad.init(blackboard)
  if not gamepad.initialized():
    print("couldn't initialize gamepad")
    sys.exit(1)
  blackboard["gamepad"] = gamepad

  ############### dispatches
  controller_ed = BlackboardQueueCVED(
    blackboard, "gamepad")
  blackboard["gamepad_thread"] = Thread(
    target=controller_ed.run,
    args=(blackboard,
      "gamepad",
      # "done",
      None,
      bcolors.CYAN))

  ############### events
  blackboard["IterateEvent"] = IterateEvent

  ############### process init
  blackboard["gamepad_cv"].acquire()
  blackboard["gamepad_queue"].append(
    ["IterateEvent", 1, "gamepad", "gamepad"])
  blackboard["gamepad_cv"].notify(1)
  blackboard["gamepad_cv"].release()

  ############### process lifecycle
  blackboard["gamepad_thread"].start()

  # really it is wait on a Condition that
  # all consumers notify
  # but predicate on all queues being empty
  blackboard["done"].acquire()
  while nonempty_queue_exists(blackboard,
    [
      # queue names that are admissible
      # if they are nonempty
    ],
    # verbose = True
    ):
    # ) or not blackboard["atomic_bool"]:
    blackboard["done"].wait()
  blackboard["done"].release()

  ### Shutdown procedure
  print("### SHUT DOWN ###")

  # purge all shared resources
  # set all heartbeat(hb)s to false
  for k in blackboard.keys():
    if k[-3:] != "_hb":
      continue
    print("notifying hb %s" % (k))
    blackboard[k] = False

  # notify all condition variables
  for k in blackboard.keys():
    if k[-3:] != "_cv":
      continue
    print("notifying cv %s" % (k))
    mutex_k = k[:-3] + "_mutex"
    blackboard[mutex_k].acquire()
    blackboard[k].notify_all()
    blackboard[mutex_k].release()

  # join threads
  for k in blackboard.keys():
    if k[-7:] != "_thread":
      continue
    print("joining", k)
    blackboard[k].join()

  gamepad.cleanup()

class CommandGamepad(Gamepad):
  def __init__(self):
    config = Configuration()
    self.joystick_interface = JoystickInterface(config)

    # joystickinterface expects a state
    # whereas gamepad captures deltas
    self._msg = {
      # discrete between 0 (off) and 1 (on)
      "R1": 0,
      "x": 0,
      "L1": 0,

      # continuous these all go between 0.0 and 1.0
      "ly": 0.0,
      "lx": 0.0,
      "rx": 0.0,
      "ry": 0.0,
      "dpady": 0.0,
      "dpadx": 0.0,

      # other
      "message_rate": 20, # 20 Hz

      # # unused
      # "L2": L2,
      # "R2": R2,
      # "square": square,
      # "circle": circle,
      # "triangle": triangle,
    }

    self._cmd_target_name = None

    self._iterable_target = None

    super(CommandGamepad, self).__init__()

  def init_external_blackboard(self, *args):
    # initialize / forward-declare
    # what pupper_ed needs

    if type(args[0]) is not dict:
      raise Exception("CommandGamepad expects dict, str, str")
    if type(args[1]) is not str:
      raise Exception("CommandGamepad expects dict, str, str")
    if type(args[2]) is not str:
      raise Exception("CommandGamepad expects dict, str, str")

    self.external_blackboard = args[0]

    self._cmd_target_name = args[1]

    self._iterable_target = args[2]

    mutex_name = self._cmd_target_name + "_mutex"
    if mutex_name not in self.external_blackboard:
      # without creating one explicitly
      # condition has underlying mutex
      self.external_blackboard[mutex_name] = Lock()

    cv_name = self._cmd_target_name + "_cv"
    if cv_name not in self.external_blackboard:
      self.external_blackboard[cv_name] = Condition(
        self.external_blackboard[mutex_name])

    queue_name = self._cmd_target_name + "_queue"
    if queue_name not in self.external_blackboard:
      self.external_blackboard[queue_name] = []

  def produce(self, k, v):
    # produce a command to cmd_target if it's something
    # pupper cares about
    if (k not in self._msg):
      return

    self._msg[k] = v

    cmd = self.joystick_interface.build_command(
      self.external_blackboard[self._iterable_target].state,
      self._msg)
    # print("cmd", cmd)

    self.external_blackboard[self._cmd_target_name + "_cv"].acquire()
    self.external_blackboard[self._cmd_target_name + "_queue"].append(
      [
        "CmdSetEvent",
        1,
        self._iterable_target,
        self._cmd_target_name,
        cmd])
    self.external_blackboard[self._cmd_target_name + "_cv"].notify(1)
    self.external_blackboard[self._cmd_target_name + "_cv"].release()

  def teardown(self, blackboard):
    blackboard[self._cmd_target_name + "_cv"].acquire()
    blackboard[self._cmd_target_name + "_queue"].clear()
    blackboard[self._cmd_target_name + "_cv"].notify_all()
    blackboard[self._cmd_target_name + "_cv"].release()

    super(CommandGamepad, self).teardown(blackboard)
