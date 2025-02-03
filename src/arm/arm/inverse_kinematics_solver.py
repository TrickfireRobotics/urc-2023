import os
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import numpy as np

from roboticstoolbox import ERobot

import swift
import time

# Runs on Python 3.9, websockets 12.0

# Current state: Can individually set positions to IK move the robot to
# TODO: 
# test IK more
#   add realtime controls and such to make this easier
# figure out rotations
# add proper dimensions to urdf

# Import robot urdf from the resources folder
current_dir = os.path.dirname(__file__)
urdf_file_path = os.path.join(current_dir, "..", "resource", "arm.urdf")
urdf_file_path = os.path.normpath(urdf_file_path)

# Initialise model
viator = ERobot.URDF(urdf_file_path)
print(viator)

# Remember:
# viator.q is the robot joint configuration for the current position of the hand
# viator.q = ... checks and sets the joint configuration

# Elementary transforms, basically rotation and translations in xyz
ets = viator.ets()

# ######################################################################################################################################################
# Set goal pose
# Tep is basically for storing coords & rotation
# Use spatial math sm to add the xyz & roll pitch yaw relative to the position of the hand (.q), which we got using forward kinematics (fkine) 
posX = 0
posY = 0.1
posZ = 0.2
# Tep = viator.fkine(viator.q) * sm.SE3.Tx(posX) * sm.SE3.Ty(posY) * sm.SE3.Tz(posZ) 
Tep = viator.fkine(viator.q) * sm.SE3(posX, posY, posZ) * sm.SE3.RPY([0, 0, 0], order='xyz') * sm.SE3.Rz(90, unit='deg')
# ######################################################################################################################################################

jointDict = {}
    

print(jointDict)


# Make our solver
solver = rtb.IK_LM()

# Solve the IK problem
solver.solve(ets, Tep)

def printPose():
    print("End-Effector Pose:")
    print(viator.fkine(viator.q))

    print("\Joint Positions in SE(3) Format:")
    print("┌──────┬───────────┬───────────────────────────────────────────────┐")
    print("│ link │   joint   │              SE(3) Pose                      │")
    print("├──────┼───────────┼───────────────────────────────────────────────┤")

    for i, link in enumerate(viator.links):
        joint_name = link.name if link.name else f"Joint {i}"
        pose = viator.fkine(viator.q, end=joint_name)  # FK up to joint i

        pos = pose.t  # Translation (x, y, z)
        rpy = pose.rpy(order='xyz', unit='deg')  # Rotation in roll-pitch-yaw (degrees)
        
        jointDict[joint_name] = (pos,rpy)
        print(f"│ {i:^4} │ {joint_name:^9} │ SE3({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}; "
            f"{rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°) │")

    print("└──────┴───────────┴───────────────────────────────────────────────┘")

printPose()
# Visualise the solver
# visualise_ik(solver, env)

# Past issues were:
# wheel building matplotlib
  # Solved by downgrading python to 3.9
# async running error with websockets (not swift like I thought)
  # Solved by downgrading websockets
# swift display not finding the viator robot urdf
  # Solution: sod the viator robot, use our own urdf (needed to add effort and velocity properties to the joints)

# Package list + versions:
# Package                Version
# ---------------------- -----------
# ansitable              0.11.4
# asttokens              3.0.0
# cfgv                   3.4.0
# colorama               0.4.6
# colored                2.2.4
# comm                   0.2.2
# cycler                 0.12.1
# debugpy                1.8.12
# decorator              5.1.1
# distlib                0.3.9
# exceptiongroup         1.2.2
# executing              2.1.0
# filelock               3.16.1
# fonttools              4.55.3
# identify               2.6.5
# importlib_metadata     8.5.0
# ipykernel              6.29.5
# ipython                8.18.1
# jedi                   0.19.2
# jupyter_client         8.6.3
# jupyter_core           5.7.2
# kiwisolver             1.4.7
# matplotlib             3.5.1
# matplotlib-inline      0.1.7
# nest-asyncio           1.6.0
# nodeenv                1.9.1
# numpy                  1.26.4
# packaging              24.2
# parso                  0.8.4
# pgraph-python          0.6.3
# pillow                 11.1.0
# pip                    24.3.1
# platformdirs           4.3.6
# pre_commit             4.0.1
# progress               1.6
# prompt_toolkit         3.0.48
# psutil                 6.1.1
# pure_eval              0.2.3
# Pygments               2.19.1
# pyparsing              3.2.1
# python-dateutil        2.9.0.post0
# pywin32                308
# PyYAML                 6.0.2
# pyzmq                  26.2.0
# roboticstoolbox-python 1.1.1
# rtb-data               1.0.1
# scipy                  1.13.1
# setuptools             49.2.1
# six                    1.17.0
# spatialgeometry        1.1.0
# spatialmath-python     1.1.13
# stack-data             0.6.3
# swift-sim              1.1.0
# tornado                6.4.2
# traitlets              5.14.3
# typing_extensions      4.12.2
# virtualenv             20.29.1
# wcwidth                0.2.13
# websockets             12.0
# zipp                   3.21.0