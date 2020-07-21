#!/usr/bin/env python3

'''
This script loads a specific leg
And tries to move it to arbitrary angles

USAGE:
./leg_shakeout.py --leg 0 --axis 0 --delta 

'''

import pigpio
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import PWMParams, ServoParams

from calibrate_servos import *

import argparse

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument(
    "--leg", type=int, help="leg index",
    default=0)
  parser.add_argument(
    "--axis", type=int,
    help="axis index: 0 abduct, 1 inner hip, 2 outer hip",
    default=0)
  parser.add_argument(
    "--delta", type=float,
    help="delta in degs to move, clamped",
    default=5.0)
  args = parser.parse_args()

  axis = args.axis
  leg_index = args.leg

  hardware_interface = HardwareInterface()

  # Zero out the neutral angle
  hardware_interface.servo_params.neutral_angle_degrees[
    axis, leg_index] = 0

  # Move servo to set_point angle
  hardware_interface.set_actuator_position(
    degrees_to_radians(args.delta),
    axis,
    leg_index,
  )

if __name__ == "__main__":
    main()