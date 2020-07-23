#!/usr/bin/env python3

'''
This script loads a specific leg
And tries to move it to arbitrary angles

USAGE:
./default_pose.py
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

# interaction

import argparse

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

  def do_init(self, *args, **kwargs):
    self.controller.set_pose_to_default(self.state)
    self.hardware_interface.set_actuator_postions(
      self.state.joint_angles)

    if type(args[0]) is not dict:
      raise Exception(
        "expected init with blackboard")

    self._blackboard = args[0]

    if "pupper_command" not in self._blackboard:
      raise Exception(
        "expected init with pupper_command")

  def do_iterate(self, *args, **kwargs):
    self.controller.run(
      self.state,
      self._blackboard["pupper_command"])

  def do_cleanup(self):
    pass

class PupperRunEvent(Event):
  def __init__(self, event_id, *args, **kwargs):
    super(PupperRunEvent, self).__init__(
      event_id, *args, **kwargs)

  def dispatch(self, event_dispatch, *args, **kwargs):
    super(PupperRunEvent, self).dispatch(
      event_dispatch, *args, **kwargs)

    # print("dispatching")
    event_dispatch.blackboard[
      "pupper"].iterate(event_dispatch)
    # time.sleep(0.05)
    # print("dispatch ending")

  def finish(self, event_dispatch, *args, **kwargs):
    # ############### option A:
    # spawn the next event as an explicit
    # **child** thread
    # constructor_args = (self.event_id,
    #   event_dispatch.blackboard)
    # event_dispatch.dispatch(
    #   blackboard["ControllerCheckEvent"](
    #     *constructor_args), ())

    # ############### option B:
    # spawn the next event through the queue
    # and the ED, aka, a **sibling** thread

    event_dispatch.blackboard["pupper_cv"].acquire()
    event_dispatch.blackboard["pupper_queue"].append(
      ["PupperRunEvent", self.event_id])
    event_dispatch.blackboard["pupper_cv"].notify(1)
    event_dispatch.blackboard["pupper_cv"].release()

    # event_dispatch.blackboard["controller_queue"].pop(0)

  @staticmethod
  def deserialize(blackboard, *args, **kwargs):
    tokens = args[0] # args is a tuple of 1 list, that list is tokens
    if len(tokens) != 1:
      raise Exception("expected 1 token")
    return (int(tokens[0]), blackboard), () # tuple

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  # parser.add_argument(
  #   "--leg", type=int, help="leg",
  #   default=0)
  # parser.add_argument(
  #   "--axis", type=int,
  #   help="axis index: 0 abduct, 1 inner hip, 2 outer hip",
  #   default=0)
  # parser.add_argument(
  #   "--delta", type=float,
  #   help="delta in degs to move, clamped",
  #   default=5.0)
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
  pupper = Pupper()
  blackboard["pupper_command"] = Command()
  pupper.init(blackboard)
  if not pupper.initialized():
    print("couldn't initialize pupper")
    sys.exit(1)
  blackboard["pupper"] = pupper

  ############### dispatches
  pupper_ed = BlackboardQueueCVED(
    blackboard, "pupper")
  blackboard["pupper_thread"] = Thread(
    target=pupper_ed.run,
    args=(blackboard,
      "pupper",
      # "done",
      None,
      bcolors.CYAN))

  input_ed = BlackboardQueueCVED(
    blackboard, "input")
  blackboard["input_thread"] = Thread(
    target=input_ed.run,
    args=(blackboard,
      "input",
      # "done",
      None,
      bcolors.GREEN))

  ############### events
  blackboard["PupperRunEvent"] = PupperRunEvent
  # blackboard["InputCheckEvent"] = InputCheckEvent

  ############### process init
  blackboard["pupper_cv"].acquire()
  blackboard["pupper_queue"].append(
    ["PupperRunEvent", 1])
  blackboard["pupper_cv"].notify(1)
  blackboard["pupper_cv"].release()

  blackboard["input_cv"].acquire()
  blackboard["input_queue"].append(
    ["InputCheckEvent", 1])
  blackboard["input_cv"].notify(1)
  blackboard["input_cv"].release()

  ############### process lifecycle
  blackboard["pupper_thread"].start()
  blackboard["input_thread"].start()

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

  pupper.cleanup()
