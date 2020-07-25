#!/usr/bin/env python3

'''
This script loads a specific leg
And tries to move it to arbitrary angles

USAGE:
'''

# pupper
import pigpio
from pupper.Config import PWMParams, ServoParams
import numpy as np
import time
from src.IMU import IMU
from src.Command import Command
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

# command
from gamepad import *
from src.Command import Command
from src.JoystickInterface import JoystickInterface

from common import IterateEvent

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

    super(CommandGamepad, self).__init__()

  def init_external_blackboard(self, *args):
    # initialize / forward-declare
    # what pupper_ed needs

    if type(args[0]) is not dict:
      raise Exception("CommandGamepad expects dict, str")
    if type(args[1]) is not str:
      raise Exception("CommandGamepad expects dict, str")

    self.external_blackboard = args[0]

    self._cmd_target_name = args[1]

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
    # produce a command to cmd_target

    self._msg[k] = v
    cmd = self.joystick_interface.build_command(
      self.external_blackboard["pupper"].state,
      self._msg)

    print("made command!", cmd)

    self.external_blackboard[self._cmd_target_name + "_cv"].acquire()
    self.external_blackboard[self._cmd_target_name + "_queue"].append(
      [
        "CmdSetEvent",
        1,
        self._cmd_target_name,
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

class Pupper(IterableObject):
  def __init__(self):
    super(Pupper, self).__init__()

    self.config = Configuration()
    self.hardware_interface = HardwareInterface()
    self.controller = Controller(
        self.config,
        four_legs_inverse_kinematics,
    )
    self.state = State()

    self._blackboard = None

    self._cmd = Command()

  def do_init(self, *args, **kwargs):
    self.controller.set_pose_to_default(self.state)
    self.hardware_interface.set_actuator_postions(
      self.state.joint_angles)

    if type(args[0]) is not dict:
      raise Exception(
        "expected init with blackboard")
    self._blackboard = args[0]

  def do_iterate(self, *args, **kwargs):
    self.controller.run(
      self.state,
      self._cmd)

    self.hardware_interface.set_actuator_postions(
      self.state.joint_angles)

  def do_cleanup(self):
    pass

class CmdSetEvent(IterateEvent):
  def dispatch(self, event_dispatch, *args, **kwargs):
    # set the pupper's cmd
    event_dispatch.blackboard[
      "pupper"]._cmd = args[2]

    super(CmdSetEvent, self).dispatch(
      event_dispatch, *args, **kwargs)

  @staticmethod
  def deserialize(blackboard, *args, **kwargs):
    tokens = args[0] # args is a tuple of 1 list, that list is tokens
    if len(tokens) != 4:
      raise Exception("expected 4 token")
    return (int(tokens[0]), blackboard), (tokens[1], tokens[2], tokens[3]) # tuple

if __name__ == "__main__":
  from common import bcolors, nonempty_queue_exists
  from common import IterateEvent, BlackboardQueueCVED

  import argparse
  from threading import Lock, Condition, Thread

  parser = argparse.ArgumentParser()
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
  gamepad = CommandGamepad()
  gamepad.init(blackboard, "pupper")
  if not gamepad.initialized():
    print("couldn't initialize gamepad")
    sys.exit(1)
  blackboard["gamepad"] = gamepad

  pupper = Pupper()
  pupper.init(blackboard)
  if not pupper.initialized():
    print("couldn't initialize pupper")
    sys.exit(1)
  blackboard["pupper"] = pupper

  ############### dispatches
  gamepad_ed = BlackboardQueueCVED(
    blackboard, "gamepad")
  blackboard["gamepad_thread"] = Thread(
    target=gamepad_ed.run,
    args=(blackboard,
      "gamepad",
      # "done",
      None,
      bcolors.CYAN))

  pupper_ed = BlackboardQueueCVED(
    blackboard, "pupper")
  blackboard["pupper_thread"] = Thread(
    target=pupper_ed.run,
    args=(blackboard,
      "pupper",
      # "done",
      None,
      bcolors.GREEN))

  ############### events
  blackboard["IterateEvent"] = IterateEvent
  blackboard["CmdSetEvent"] = CmdSetEvent

  ############### process init
  blackboard["gamepad_cv"].acquire()
  blackboard["gamepad_queue"].append(
    ["IterateEvent", 1, "gamepad", "gamepad"])
  blackboard["gamepad_cv"].notify(1)
  blackboard["gamepad_cv"].release()

  blackboard["pupper_cv"].acquire()
  blackboard["pupper_queue"].append(
    ["IterateEvent", 1, "pupper", "pupper"])
  blackboard["pupper_cv"].notify(1)
  blackboard["pupper_cv"].release()

  ############### process lifecycle
  blackboard["gamepad_thread"].start()
  blackboard["pupper_thread"].start()

  # really it is wait on a Condition that
  # all consumers notify
  # but predicate on all queues being empty
  blackboard["done"].acquire()
  while nonempty_queue_exists(blackboard,
    [
      # queue names that are admissible
      # if they are nonempty
      "gamepad_queue",
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

  gamepad.cleanup()
  pupper.cleanup()
