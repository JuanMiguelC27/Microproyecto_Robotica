#!/usr/bin/env python3
"""Joint names and Gazebo command topics shared by the Gecko twin."""

from typing import List

WHEEL_JOINTS = (
    'Wheel_FL_Joint',
    'Wheel_FR_Joint',
    'Wheel_RL_Joint',
    'Wheel_RR_Joint',
)
TURBINE_JOINTS = ('Turbine_F_Joint', 'Turbine_R_Joint')
SERVO_JOINTS = ('Turbine_Servo_F_Joint', 'Turbine_Servo_R_Joint')

VELOCITY_JOINTS = WHEEL_JOINTS + TURBINE_JOINTS


def velocity_topic(joint: str) -> str:
    return f'/gecko/gz/{joint}/cmd_vel'


def position_topic(joint: str) -> str:
    return f'/gecko/gz/{joint}/cmd_pos'


def bridge_args() -> List[str]:
    """Return the ROS -> Gazebo parameter_bridge argument for every joint."""
    topics = [velocity_topic(j) for j in VELOCITY_JOINTS]
    topics += [position_topic(j) for j in SERVO_JOINTS]
    return [f'{t}@std_msgs/msg/Float64]ignition.msgs.Double' for t in topics]
