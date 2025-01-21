import os
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import numpy as np

from roboticstoolbox import ERobot

import swift

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

# Give one of the joints some velocity to test moving
# viator.qd = [0,0.1,0,0,0]

# Elementary transforms, basically rotation and translations in xyz
ets = viator.ets()

# Make a new Swift environment
env = swift.Swift()
env.launch(realtime = True)

# Add the bot to swift
# viator.plot(viator.q,backend='swift', block=True)
env.add(viator)
  
# Step through the environment simulator 
# for _ in range(100):
#     env.step(0.05)

# ######################################################################################################################################################
# Set goal pose
# Tep is basically for storing coords & rotation
# Use spatial math sm to add the xyz & roll pitch yaw relative to the position of the hand (.q), which we got using forward kinematics (fkine) 
posX = 0
posY = 0.1
posZ = 0.1
# Tep = viator.fkine(viator.q) * sm.SE3.Tx(posX) * sm.SE3.Ty(posY) * sm.SE3.Tz(posZ) 
Tep = viator.fkine(viator.q) * sm.SE3(posX, posY, posZ) * sm.SE3.RPY([0, 0, 0], order='xyz') * sm.SE3.Rz(90, unit='deg')
# ######################################################################################################################################################

# Add axes at the coords we just set
axes = sg.Axes(length=0.1, base=Tep)
env.add(axes)


# Sim controls
arrived = False # arrived at destination flag
dt = 0.05 # time step, default 0.01

# Sim loop
while not arrived:
    # p_servo Returns the end-effector velocity which will cause the robot to approach the desired pose
    # starting pose is .q
    # end pose is Tep
    # gain is how fast we want to get there
    # threshold is tolerance for when we are considered "at" the destination pose
    v, arrived = rtb.p_servo(viator.fkine(viator.q), Tep, gain=1, threshold=0.05)

    # Calculate the jacobean (?)
    J = viator.jacobe(viator.q)

    # Use both to get the desired velocity of the robot
    # Multiply the "psuedo-inverse of the jacobian" times the desired end-effector velocity (???)
    viator.qd = np.linalg.pinv(J) @ v

    # Step through the environment simulator 
    env.step(dt)

# Use ctrl+C in the terminal to kill the sim, don't just close the tab or you'll have to restart the terminal
env.hold()

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