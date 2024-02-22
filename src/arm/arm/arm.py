#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from src.interface.robot_interface import RobotInterface
from src.robot_info.robot_info import RobotInfo

from std_msgs.msg import String
from std_msgs.msg import Float32
import ikpy.chain
import math

# change this import later if mission control uses different msg types
from geometry_msgs.msg import Point
# add msg for true/false (if needed)


class Arm(Node):
    def __init__(self, urdf_file_name, active_links_masks, start_position):
        super().__init__(node_name="arm")

        # robot interface: send data
        # robot info: retrieve data
        self._robot_info = RobotInfo(self)  # fix later
        self._robot_interface = RobotInterface(self)

        # store args data for reset
        self._urdf_file_name = urdf_file_name
        self._active_link_masks = active_links_masks
        self._start_position = start_position

        # set up arm figure data
        # Note: posistion = motor positions (radian)
        #       end_effector_position: (x, y, z)
        self._current_position = start_position
        self._arm_chain = ikpy.chain.Chain.from_urdf_file(
            urdf_file_name, active_links_mask=active_links_masks)
        self._end_effector_result = self._arm_chain.forward_kinematics(
            self._current_position)
        self._end_effector_position = self._end_effector_result[:3, 3]
        self._end_effector_orientation = self._end_effector_result[:3, :3]

        # start angle of alpha, beta, gamma (fix later)
        self._current_angles = [-1.6, 0, 0]

        self._curr_enable_status = False

        # create subscription to mission control
        # Note axes: The y axis points out the front end of the rover,
        # x to the right, and the z axis is up.
        # ***************
        # Control topics: enable/disable arm, reset, pitch, yaw, move xy, move z
        #                 z should be in the form of (trigger left value, trigger right value)
        # **************
        # Note to self: change msg types later
        self._enable_status_subscriber = self.create_subscription(
            bool, "enable_status_from_mission_control", self.update_enable_status, 10)
        self._reset_status_subscriber = self.create_subscription(
            bool, "reset_status_from_mission_control", self.need_reset, 10)
        self._pitch_action_subscriber = self.create_subscription(
            Point, "yaw_pitch_action_from_mission_control", self.perform_yaw_pitch_action, 10)
        self._move_on_xy_subscriber = self.create_subscription(
            Point, "move_on_xy_action_from_mission_control", self.move_to_xy_action, 10)
        self._move_on_z_subscriber = self.create_subscription(
            String, "move_on_z_action_from_mission_control", self.move_to_z_action, 10)

    # ***************
    # Methods to update the arm's states
    # ***************
    def update_enable_status(self, new_enable_status: bool):
        self._curr_enable_status = new_enable_status

    # reset the arm if needed
    def need_reset(self, should_reset: bool):
        if (should_reset):
            self._reset_arm

    # ***************
    # Register movements and process calculations
    # ***************
    def perform_yaw_pitch_action(self, right_thumb_stick_coordinate: Point):
        # if the arm is not enable
        if (not self._curr_enable_status):
            return

        # -----------------

        target_position_orientation = self._calculate_new_position_and_orientation(
            (0, 0), right_thumb_stick_coordinate, (0, 0))

        # reach the correct position (position of motors) first
        # self._current_position = ik_chain.inverse_kinematics(
        #    position, initial_position=self._current_position)

        # then reach the correct orientation
        # reference -
        # https://github.com/Phylliade/ikpy/blob/master/tutorials/Orientation.ipynb
        # see Orientation and Position section
        target_motor_positions = self._arm_chain.inverse_kinematics(
            target_position_orientation[0], target_position_orientation[1], orientation_mode="all", initial_position=self._current_position)

        # send command to robot interface
        self._send_command_to_robot_interface(target_motor_positions)

        # fix later: do I need to check if the movement is successful (?)

        # update current position
        self._current_position = target_motor_positions

    def move_to_xy_action(self, left_thumb_stick_coordinate: Point):
        # if the arm is not enable
        if (not self._curr_enable_status):
            return

        # -----------------

        target_position_orientation = self._calculate_new_position_and_orientation(
            left_thumb_stick_coordinate, (0, 0), (0, 0))

        target_motor_positions = self._arm_chain.inverse_kinematics(
            target_position_orientation[0], target_position_orientation[1], orientation_mode="all", initial_position=self._current_position)

        # send command to robot interface
        self._send_command_to_robot_interface(target_motor_positions)

        # update current position
        self._current_position = target_motor_positions

    def move_to_z_action(self, triggers_position: String):
        # if the arm is not enable
        if (not self._curr_enable_status):
            return

        # -----------------

        target_position_orientation = self._calculate_new_position_and_orientation(
            (0, 0), (0, 0), triggers_position)

        target_motor_positions = self._arm_chain.inverse_kinematics(
            target_position_orientation[0], target_position_orientation[1], orientation_mode="all", initial_position=self._current_position)

        # send command to robot interface
        self._send_command_to_robot_interface(target_motor_positions)

        # update current position
        self._current_position = target_motor_positions

    # ***************
    # Private methods
    # ***************

    def _reset_arm(self):
        self._current_position = self._start_position
        self._arm_chain = ikpy.chain.Chain.from_urdf_file(
            self._urdf_file_name, active_links_mask=self._active_link_masks)
        self._end_effector_result = self._arm_chain.forward_kinematics(
            self._current_position)
        self._end_effector_position = self._end_effector_result[:3, 3]
        self._end_effector_orientation = self._end_effector_result[:3, :3]
        self._current_angles = [-1.6, 0, 0]

    # calculate the orientation

    def _calculate_new_position_and_orientation(self, left_thumb_stick_coor, right_thumb_stick_coor, triggers_position):
        # update current positions & orientation data
        self._calculate_current_end_effector_position_and_orientation()

        # left: move xy, right: pitch yaw, triggers: z
        x_pos = self._end_effector_position[0]
        y_pos = self._end_effector_position[1]
        z_pos = self._end_effector_position[2]

        alpha = self._current_angles[0]
        beta = self._current_angles[1]
        gamma = self._current_angles[2]

        # Accumulate the position values from the controller
        x_pos -= 0.005 * left_thumb_stick_coor[0]
        y_pos += 0.005 * left_thumb_stick_coor[1]
        z_pos += 0.005 * triggers_position[1]  # left trigger
        z_pos -= 0.005 * triggers_position[0]  # right trigger

        target_position = [-x_pos, y_pos, z_pos]

        alpha += 0.05 * self.right_thumb_stick_coor[1]
        beta += 0.05 * self.right_thumb_stick_coor[0]
        gamma = 0

        # build the "rotational matrix" in the absolute referential
        # (extrinsic rotation - relative to an external, fixed coordinate system)
        # see - https://en.wikipedia.org/wiki/Rotation_matrix
        # specifically the general rotation matrix with improper
        # Euler angles alpha(x), beta(y), and gamma(z)
        # https://github.com/Phylliade/ikpy/blob/master/tutorials/Orientation.ipynb
        # specifically Orientation on a full referential
        aa = math.cos(beta)*math.cos(gamma)
        ab = math.sin(alpha)*math.sin(beta)*math.cos(gamma)
        -math.cos(alpha)*math.sin(gamma)
        ac = math.cos(alpha)*math.sin(beta)*math.cos(gamma)
        +math.sin(alpha)*math.sin(gamma)
        ba = math.cos(beta)*math.sin(gamma)
        bb = math.sin(alpha)*math.sin(beta)*math.sin(gamma)
        +math.cos(alpha)*math.cos(gamma)
        bc = math.cos(alpha)*math.sin(beta)*math.sin(gamma)
        -math.sin(alpha)*math.cos(gamma)
        ca = -math.sin(beta)
        cb = math.sin(alpha)*math.cos(beta)
        cc = math.cos(alpha)*math.cos(beta)

        target_orientation = [[aa, ab, ac],
                              [ba, bb, bc],
                              [ca, cb, cc]]

        return (target_position, target_orientation)

    def _calculate_current_end_effector_position_and_orientation(self):
        current_motor_positions = []
        current_motor_positions.append(
            self._robot_info.get_arm_turntable_motor_position())
        current_motor_positions.append(
            self._robot_info.get_arm_shoulder_motor_position())
        current_motor_positions.append(
            self._robot_info.get_arm_elbow_motor_position())
        current_motor_positions.append(
            self._robot_info.get_arm_forearm_motor_position())
        current_motor_positions.append(
            self._robot_info.get_arm_wrist_motor_position())

        # might not need these for the hand
        # current_motor_positions.append(
        #     self._robot_info.get_arm_hand_motor_position())
        # current_motor_positions.append(
        #     self._robot_info.get_arm_fingers_motor_position())

        # -------------

        # update current position
        self._current_position = current_motor_positions
        self._end_effector_result = self._arm_chain.forward_kinematics(
            self._current_position)
        self._end_effector_position = self._end_effector_result[:3, 3]
        self._end_effector_orientation = self._end_effector_result[:3, :3]

    def _send_command_to_robot_interface(self, target_motor_positions):
        self._robot_interface.arm_turntable_motor(target_motor_positions[0])
        self._robot_interface.arm_shoulder_motor(target_motor_positions[1])
        self._robot_interface.arm_elbow_motor(target_motor_positions[2])
        self._robot_interface.arm_forearm_motor(target_motor_positions[3])
        self._robot_interface.arm_wrist_motor(target_motor_positions[4])
        # self._robot_interface.arm_turntable_motor(target_motor_positions[5])
        # self._robot_interface.arm_fingers_motor(target_motor_positions[6])


# fix these args later
def main(args=None):
    rclpy.init(args=args)

    arm_node = Arm("placeholder.urdf", ["placeholder_masks"],
                   ["placeholder_positions"])

    rclpy.spin(arm_node)

    arm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
