from src.Gaits import GaitController
from src.StanceController import StanceController
from src.SwingLegController import SwingController
from src.Utilities import clipped_first_order_filter
from src.State import BehaviorState, State

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat


class Controller:
    """Controller and planner object
    """

    def __init__(
        self,
        config,
        inverse_kinematics):
        self.config = config

        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        # define a state machine
        self.hop_transition_mapping = {
            BehaviorState.REST: BehaviorState.HOP,
            BehaviorState.HOP: BehaviorState.FINISHHOP,
            BehaviorState.FINISHHOP: BehaviorState.REST,
            BehaviorState.TROT: BehaviorState.HOP}

        self.trot_transition_mapping = {
            BehaviorState.REST: BehaviorState.TROT,
            BehaviorState.TROT: BehaviorState.REST,
            BehaviorState.HOP: BehaviorState.TROT,
            BehaviorState.FINISHHOP: BehaviorState.TROT}

        self.activate_transition_mapping = {
            BehaviorState.DEACTIVATED: BehaviorState.REST,
            # BehaviorState.RISING: BehaviorState.RISEN,
            # BehaviorState.RISEN: BehaviorState.REST,
            BehaviorState.REST: BehaviorState.DEACTIVATED}


    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes


    def run(self, state, command):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """


        # 2021-05 if commanding horizontal_velocity
        # put into TROT mode
        # otherwise, put into REST mod
        if np.linalg.norm(command.horizontal_velocity) > 1e-5 and state.behavior_state == BehaviorState.REST:
          print("horizontal_velocity", command.horizontal_velocity)
          state.behavior_state = BehaviorState.TROT
        # else:
        #   state.behavior_state = BehaviorState.REST

        ########## Update operating state based on command ######
        ########## State machines grrrrr :( ######
        if command.activate_event and\
            state.behavior_state in self.activate_transition_mapping:
            state.behavior_state = self.activate_transition_mapping[
                state.behavior_state]
            command.activate_event = not command.activate_event
        elif command.trot_event and\
            state.behavior_state in self.trot_transition_mapping:
            state.behavior_state = self.trot_transition_mapping[
                state.behavior_state]
            command.trot_event = not command.trot_event
            if state.behavior_state == BehaviorState.REST:
              command.horizontal_velocity = np.array([0, 0])
        elif command.hop_event and\
            state.behavior_state in self.hop_transition_mapping:
            state.behavior_state = self.hop_transition_mapping[
                state.behavior_state]
            command.hop_event = not command.hop_event

        # print("state", state.behavior_state)

        # update cmd_foot_locations
        cmd_foot_locations = None

        if state.behavior_state == BehaviorState.TROT:
            # print("controller trotting")

            state.foot_locations, contact_modes = self.step_gait(
                state,
                command,
            )

            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            (roll, pitch, yaw) = quat2euler(state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations
            cmd_foot_locations = rotated_foot_locations

        elif state.behavior_state == BehaviorState.HOP:
            # crouch down
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, -0.09])[:, np.newaxis]
            )
            cmd_foot_locations = state.foot_locations

        elif state.behavior_state == BehaviorState.FINISHHOP:
            # pop up
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, -0.22])[:, np.newaxis]
            )
            cmd_foot_locations = state.foot_locations

        elif state.behavior_state == BehaviorState.REST:
            # print("controller resting")
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )
            # Set the foot locations to the default
            # stance plus the standard height
            # note that height is only relevant
            # for BehaviorState.REST

            # propagate command.height/roll here
            # because Controller is ALWAYS running
            # for stream semantics #cool
            command.height = state.height -\
                self.config.dt * self.config.z_speed * command.height_delta
            command.roll = state.roll +\
                self.config.dt * self.config.roll_speed * command.roll_delta

            # clamping in case comms loss
            if command.height > self.config.max_cmd_height:
                command.height = self.config.max_cmd_height
            elif command.height < self.config.min_cmd_height:
                command.height = self.config.min_cmd_height

            if command.roll > self.config.max_cmd_roll:
                command.roll = self.config.max_cmd_roll
            elif command.roll < self.config.min_cmd_roll:
                command.roll = self.config.min_cmd_roll

            # print("controller height %.3f, roll %.3f" % (
            #     command.height, command.roll))

            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll,
                    command.pitch,
                    self.smoothed_yaw,
                )
                @ state.foot_locations
            )
            cmd_foot_locations = rotated_foot_locations

        # update joint_angles off cmd_foot_locations
        if cmd_foot_locations is not None:
            state.joint_angles = self.inverse_kinematics(
                cmd_foot_locations, self.config
            )

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

    def set_pose_to_default(self, state):
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        state.joint_angles = self.inverse_kinematics(
            state.foot_locations, self.config
        )
