#!/usr/bin/env python3

'''
This script loads a specific leg
And tries to move it to arbitrary angles

USAGE:
./controller_shakeout.py --simulate 1/0

2021-05-13
audio,1 => move forward or back
audio,2 => move randomly side to side
audio,3 => tilt f/b or side/side

manual => engage control
horn => noop
head lights => tilt left / right randomly
rear lights => tilt f/b randomly

'''

# pupper
try:
  import pigpio
except:
  pass

from pupper.Config import PWMParams, ServoParams
import numpy as np
import time, random
from src.IMU import IMU
from src.Command import Command
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

# command
from src.Command import Command
from src.JoystickInterface import JoystickInterface

from common import IterateEvent
from common import isint, isfloat

from serial_bridge import *
from gamepad import *

class Pupper(IterableObject):
  def __init__(self, simulate = False, activated = False):
    super(Pupper, self).__init__()

    self._blackboard = None

    self.config = Configuration()

    self.hardware_interface = HardwareInterface(simulate)

    self.controller = Controller(
        self.config,
        four_legs_inverse_kinematics,
    )
    self.state = State()

    self.activated = activated
    if activated:
      self._cmd.activate_event = 1

    self._cmd = Command()

    self._last_t = time.time()

  def do_init(self, *args, **kwargs):
    self.controller.set_pose_to_default(self.state)
    self.hardware_interface.set_actuator_postions(
      self.state.joint_angles)

    if type(args[0]) is not dict:
      raise Exception(
        "expected init with blackboard")
    self._blackboard = args[0]

  def do_iterate(self, *args, **kwargs):
    cleanup = False
    if self._cmd.activate_event == 1:
      self.activated = not self.activated
      cleanup = True

    if self.activated:
      now = time.time()
      if now - self._last_t < self.config.dt:
        # print("too soon", now - self._last_t)
        time.sleep(self.config.dt)
        return

      self.controller.run(
        self.state,
        self._cmd)
      self.hardware_interface.set_actuator_postions(
        self.state.joint_angles)

      self._last_t = time.time()

    time.sleep(self.config.dt)

    if cleanup:
      # unset it immediately, do not wait for gamepad
      self._cmd.activate_event = 0

  def do_cleanup(self):
    pass

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
    print("cmd", cmd)

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

class CommandSerialBridge(SerialBridge):
  def __init__(self):
    config = Configuration()
    self.joystick_interface = JoystickInterface(config)

    # joystickinterface expects a state
    # whereas this captures deltas
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

    # because controller m00 gamepad is not updated
    # fix it here
    self._temp_mapping = {
      "tl" : "L1",
      "tr" : "R1",
      "hat0y" : "dpady",
      "hat0x" : "dpadx",
      "x" : "lx",
      "y" : "ly",
    }

    self._cmd_target_name = None

    super(CommandSerialBridge, self).__init__()

  def init_external_blackboard(self, *args):
    # initialize / forward-declare
    # what pupper_ed needs

    if type(args[2]) is not dict:
      raise Exception(
        self.__class__.__name__ + " expects dict, str, str")
    if type(args[3]) is not str:
      raise Exception(
        self.__class__.__name__ + " expects dict, str, str")
    if type(args[4]) is not str:
      raise Exception(
        self.__class__.__name__ + " expects dict, str, str")

    self.external_blackboard = args[2]
    self._cmd_target_name = args[3]
    self._iterable_target = args[4]

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

  def produce(self, data):
    if "," not in data:
      return

    k, v = data.split(",")

    if isfloat(v):
      v = float(v)
      if int(v) == v:
        v = int(v)

    if k in self._temp_mapping:
      k = self._temp_mapping[k]

    if k == "select" and v == 0:
      self.teardown(self.external_blackboard)
      return

    # produce a command to cmd_target if it's something
    # pupper cares about
    if (k not in self._msg):
      print("k not found", k)
      return

    self._msg[k] = v

    cmd = self.joystick_interface.build_command(
      self.external_blackboard[
        self._iterable_target].state,
      self._msg)
    print(cmd)

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

    super(CommandSerialBridge, self).teardown(blackboard)

class RoboAutoAdapter(object):
  def __init__(self):
    self._h = 0

  def adapt(self, fin_k, fin_v):
    pupper_k = None
    pupper_v = None
    repeat = 1

    if fin_v == "F":
      pupper_k = "ly"
      pupper_v = 0.5
    elif fin_v == "B":
      pupper_k = "ly"
      pupper_v = -0.5

    elif fin_v == "L":
      pupper_k = "lx"
      pupper_v = 0.5
    elif fin_v == "R":
      pupper_k = "lx"
      pupper_v = -0.5

    # yaw rate
    elif fin_v == "Q":
      pupper_k = "rx"
      pupper_v = -0.5
      repeat = 5
    elif fin_v == "q":
      pupper_k = "rx"
      pupper_v = -0.5
      repeat = 5

    # height
    elif fin_v == "T":
      pupper_k = "dpady"
      pupper_v = -0.5
      repeat = 5

    elif fin_v == "t":
      pupper_k = "dpady"
      pupper_v = -0.5
      repeat = 5

    # manual / auto activates
    elif fin_v == "M":
      pupper_k = "L1"
      pupper_v = 1
    elif fin_v == "A":
      pupper_k = "L1"
      pupper_v = 0

    # H toggles trot event
    elif fin_v == "H":
      pupper_k = "R1"

      if self._h == 0:
        self._h = 1
      else:
        self._h = 0

      pupper_v = self._h
      repeat = 5

    if pupper_k is None:
      return None, None

    return [pupper_k] * repeat, [pupper_v] * repeat

class AudioAdapter1(object):
  def __init__(self):
    # 5 audiostream events
    # to 'start' a random dance set
    self._moves = {
      # discrete between 0 (off) and 1 (on)
      "R1": 0, # 1 = trot_event trigger
      # "x": 0, # hop_toggle
      # "L1": 0, # 1 = activate_toggle trigger

      # continuous these all go between 0.0 and 1.0
      "ly": 0.0, # y_vel
      "lx": 0.0, # x_vel
      "dpady": 0.0, # height movement

      "rx": 0.0, # yaw_rate
      "ry": 0.0, # pitch
      "dpadx": 0.0, # roll_movement
    }

    self._move_keys = sorted(
      self._moves.keys())

    self._audiostream_state = 0
    self._last_move = None
    # cycle across
    # None, assign random and do
    # non-None, assign counter and do

    self._moves_to_do = 0
    # random between 1 an 10 moves
    # per routine

    # buffer moves and shoot off one per audiostream
    # event in case they arrive too quickly
    self._moves_queue = []

  def adapt(self, fin_k, fin_v):
    pupper_k = None
    pupper_v = None
    repeat = 1

    if fin_k == "audiostream":
      if self._audiostream_state < 5:
        self._audiostream_state += 1
      if self._audiostream_state == 5:
        # start a dance set
        if self._moves_to_do == 0:
          self._moves_to_do = random.randint(
            1, 10)

        cleanup = False
        if self._last_move is None:
          rand_idx = random.randint(
            0, len(self._move_keys))
          self._last_move = self._move_keys[rand_idx]
        else:
          cleanup = True

        if type(self._moves[self._last_move]) == int:
          if self._moves[self._last_move] == 0:
            self._moves[self._last_move] = 1
          else:
            self._moves[self._last_move] = 0
        else:
          if self._moves[self._last_move] < 1e-8:
            self._moves[self._last_move]\
              += random.uniform(-1.0, 1.0)
          else:
            self._moves[self._last_move]\
              -= self._moves[self._last_move]

        # add to _moves_queue
        self._moves_queue.append([
          self._last_move, self._moves[self._last_move]])

        if len(self._moves_queue) > 0:
          pupper_k, pupper_v = self._moves_queue.pop(0)
          repeat = 5

        if cleanup:
          self._last_move = None

        self._moves_to_do -= 1
        if self._moves_to_do == 0:
          self._audiostream_state = 0

    if pupper_k is None:
      return None, None

    return [pupper_k] * repeat, [pupper_v] * repeat

class FinSerialBridge(CommandSerialBridge):
  def __init__(self):
    config = Configuration()
    self.joystick_interface = JoystickInterface(config)

    # joystickinterface expects a state
    # whereas this captures deltas
    self._msg = {
      # discrete between 0 (off) and 1 (on)
      "R1": 0, # 1 = trot_event trigger
      "x": 0, # hop_toggle
      "L1": 0, # 1 = activate_toggle trigger

      # continuous these all go between 0.0 and 1.0
      "ly": 0.0, # y_vel
      "lx": 0.0, # x_vel
      "rx": 0.0, # yaw_rate
      "ry": 0.0, # pitch
      "dpady": 0.0, # height movement
      "dpadx": 0.0, # roll_movement

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

    audio_adapter = AudioAdapter1()
    self.adapters = {
      "serial" : RoboAutoAdapter(),
      "audio" : audio_adapter,
      "audiostream" : audio_adapter,
    }

    super(FinSerialBridge, self).__init__()

  def produce(self, data):
    if "," not in data:
      return

    fin_k, fin_v = data.split(",")

    if isfloat(fin_v):
      fin_v = float(fin_v)
      if int(fin_v) == fin_v:
        fin_v = int(fin_v)

    pupper_ks = None
    pupper_vs = None

    if fin_k in self.adapters:
      pupper_ks, pupper_vs =\
        self.adapters[fin_k].adapt(
          fin_k, fin_v)

    if pupper_ks is None or pupper_vs is None:
      return

    for i, pupper_k in enumerate(pupper_ks):
      pupper_v = pupper_vs[i]

      if pupper_k == "teardown":
        self.teardown(self.external_blackboard)
        return

      # produce a command to cmd_target if it's something
      # pupper cares about
      if (pupper_k not in self._msg):
        print("k not found", pupper_k)
        continue

      self._msg[pupper_k] = pupper_v

      cmd = self.joystick_interface.build_command(
        self.external_blackboard[
          self._iterable_target].state,
        self._msg)
      print(cmd)

      self.external_blackboard[
        self._cmd_target_name + "_cv"].acquire()
      self.external_blackboard[
        self._cmd_target_name + "_queue"].append(
        [
          "CmdSetEvent",
          1,
          self._iterable_target,
          self._cmd_target_name,
          cmd])
      self.external_blackboard[
        self._cmd_target_name + "_cv"].notify(1)
      self.external_blackboard[
        self._cmd_target_name + "_cv"].release()

class SimulatedFinSerialBridge(FinSerialBridge):
  def do_init(self, *args, **kwargs):
    self.external_blackboard = None
    self.init_external_blackboard(*args)

    self._simulate_data = {
      "serial" : [
        "F",
        "B",
        "L",
        "R",
        "H",
        "T",
        "t",
        "Q",
        "q"],
      "audiostream" : ["1"]
    }

    self._random_interval = 5.0
    self._queued_simulated_read_data = "serial,M"
    # load up activating pupper

  def do_iterate(self, *args, **kwargs):
    if len(self._blackboard["tx_buffer"]) > 0:
      self._serial_handle.write(
        str.encode(self._blackboard["tx_buffer"]))
      self._blackboard["tx_buffer"] = ""

    '''
    read_data = self._serial_handle.readline()
    read_data = read_data.decode('ascii').rstrip('\r\n')

    if len(read_data) > 0:
      if read_data == "end":
        self.teardown(args[0].blackboard)
        return
    '''

    time.sleep(self._random_interval)
    self.produce(self._queued_simulated_read_data)

    self._queued_simulated_read_data = "audiostream,1"
    # random_next = random.randint(0, len(
    #   self._simulate_data["serial"]))
    # self._queued_simulated_read_data = "serial," +\
    #   self._simulate_data["serial"][random_next]

    self._random_interval = random.uniform(3.0, 5.0)

class CmdSetEvent(IterateEvent):
  def dispatch(self, event_dispatch, *args, **kwargs):
    # set the pupper's cmd
    # print("updating pupper cmd")
    event_dispatch.blackboard[
      args[0]]._cmd = args[2]

    super(CmdSetEvent, self).dispatch(
      event_dispatch, *args, **kwargs)

  def finish(self, event_dispatch, *args, **kwargs):
    # do not self-produce
    # assume there will be another PupperEvent
    # to drive the next iterate
    if self._exception:
      print("exception caught")
      return

  @staticmethod
  def deserialize(blackboard, *args, **kwargs):
    tokens = args[0] # args is a tuple of 1 list, that list is tokens
    if len(tokens) != 4:
      raise Exception("expected 4 token")
      # event_id, iterable_object_key, ed_prefix, cmd
    return (int(tokens[0]), blackboard), (tokens[1], tokens[2], tokens[3]) # tuple

if __name__ == "__main__":
  from common import bcolors, nonempty_queue_exists
  from common import IterateEvent, BlackboardQueueCVED

  import argparse
  from threading import Lock, Condition, Thread

  parser = argparse.ArgumentParser(
    description="")
  parser.add_argument('--serial', type=str,
    default="/dev/ttyACM0")
  parser.add_argument('--baudrate', type=int,
    default=38400)
  parser.add_argument('--simulate', type=int,
    default=1)
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
  # gamepad = CommandGamepad()
  # gamepad.init(blackboard, "pupper", "pupper_iterable")
  # if not gamepad.initialized():
  #   print("couldn't initialize gamepad")
  #   # sys.exit(1)
  # else:
  #   blackboard["gamepad_iterable"] = gamepad

  #   # dispatch, actor consumes / produces
  #   gamepad_ed = BlackboardQueueCVED(
  #     blackboard, "gamepad")
  #   blackboard["gamepad_thread"] = Thread(
  #     target=gamepad_ed.run,
  #     args=(blackboard,
  #       "gamepad",
  #       # "done",
  #       None,
  #       bcolors.HEADER))

  #   ############### actor context setup
  #   blackboard["gamepad_cv"].acquire()
  #   blackboard["gamepad_queue"].append(
  #     ["IterateEvent", 1, "gamepad_iterable", "gamepad"])
  #   blackboard["gamepad_cv"].notify(1)
  #   blackboard["gamepad_cv"].release()

  if len(args.serial) > 0:
    sb = CommandSerialBridge()
    sb.init(
      args.serial,
      args.baudrate,
      blackboard,
      "pupper",
      "pupper_iterable")
    if not sb.initialized():
      print("no sb, running SimulatedFinSerialBridge")
      sb = SimulatedFinSerialBridge()
      sb.init(
        args.serial,
        args.baudrate,
        blackboard,
        "pupper",
        "pupper_iterable")

      blackboard["sb_iterable"] = sb

    # dispatch, actor consumes / produces
    sb_ed = BlackboardQueueCVED(
      blackboard, "sb")
    blackboard["sb_thread"] = Thread(
      target=sb_ed.run,
      args=(blackboard,
        "sb",
        # "done",
        None,
        bcolors.CYAN))

    ############### actor context setup
    blackboard["sb_cv"].acquire()
    blackboard["sb_queue"].append(
      ["IterateEvent", 1, "sb_iterable", "sb"])
    blackboard["sb_cv"].notify(1)
    blackboard["sb_cv"].release()

  pupper = Pupper(
    args.simulate > 0)
  pupper.init(blackboard)
  if not pupper.initialized():
    print("couldn't initialize pupper")
    sys.exit(1)
  blackboard["pupper_iterable"] = pupper

  # dispatch, actor consumes / produces
  pupper_ed = BlackboardQueueCVED(
    blackboard, "pupper")
  blackboard["pupper_thread"] = Thread(
    target=pupper_ed.run,
    args=(blackboard,
      "pupper",
      # "done",
      None,
      bcolors.GREEN))

  ############### actor context setup
  blackboard["pupper_cv"].acquire()
  blackboard["pupper_queue"].append(
    ["IterateEvent", 1, "pupper_iterable", "pupper"])
  blackboard["pupper_cv"].notify(1)
  blackboard["pupper_cv"].release()

  ############### dispatches
  # gamepad_ed = BlackboardQueueCVED(
  #   blackboard, "gamepad")
  # blackboard["gamepad_thread"] = Thread(
  #   target=gamepad_ed.run,
  #   args=(blackboard,
  #     "gamepad",
  #     # "done",
  #     None,
  #     bcolors.CYAN))

  ############### actor context setup
  # blackboard["gamepad_cv"].acquire()
  # blackboard["gamepad_queue"].append(
  #   ["IterateEvent", 1, "gamepad", "gamepad"])
  # blackboard["gamepad_cv"].notify(1)
  # blackboard["gamepad_cv"].release()

  ############### events
  blackboard["IterateEvent"] = IterateEvent
  blackboard["CmdSetEvent"] = CmdSetEvent

  ############### process lifecycle
  for k in blackboard.keys():
    if k[-7:] != "_thread":
      continue
    print("starting", k)
    blackboard[k].start()

  # really it is wait on a Condition that
  # all consumers notify
  # but predicate on all queues being empty
  blackboard["done"].acquire()
  while nonempty_queue_exists(blackboard,
    [
      # queue names that are admissible
      # if they are nonempty
      "sb_queue",
      "pupper_queue"
    ],
    verbose = True
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

  for k in blackboard.keys():
    if k[-9:] != "_iterable":
      continue
    print("cleanup", k)
    blackboard[k].cleanup()
