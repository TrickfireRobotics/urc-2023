import os
import roboticstoolbox as rtb
import matplotlib.pyplot
import numpy as np
import math

from matplotlib.widgets import Slider

from roboticstoolbox import ERobot


# Runs on Python 3.9, websockets 12.0

# Past issues were:
# wheel building matplotlib
  # Solved by downgrading python to 3.9
# async running error with websockets (not swift like I thought)
  # Solved by downgrading websockets
# swift display not finding the viator robot urdf
  # Solution: sod the viator robot, use our own urdf (needed to add effort and velocity properties to the joints)

# Current state: it displays, but completely static
# TODO: add the IK processing, sliders and such. Copy that from https://github.com/jhavl/dkt/tree/main, 4 Numerical Inverse Kinematics.ipynb

# Import robot urdf from the resources folder
current_dir = os.path.dirname(__file__)
urdf_file_path = os.path.join(current_dir, "..", "resource", "viator_urdf.urdf")
urdf_file_path = os.path.normpath(urdf_file_path)
viator = ERobot.URDF(urdf_file_path)
print(viator)


fig = matplotlib.pyplot.figure()
viator.plot(viator.q,backend='swift', block=True)

ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Position sliders
ax_xslider = fig.add_axes([0.25, 0.1, 0.65, 0.03])
ax_yslider = fig.add_axes([0.15, 0.25, 0.0225, 0.63])
ax_zslider = fig.add_axes([0.1, 0.25, 0.0225, 0.63])

# position slider initialisations
x_slider = Slider(
  ax=ax_xslider,
  label='x',
  valmin=-0.450,
  valmax=0.450,
  valinit=0,
  valstep=0.001,
)
y_slider = Slider(
  ax=ax_yslider,
  label='y',
  valmin=-0.450,
  valmax=0.450,
  valinit=0,
  orientation="vertical",
  valstep=0.001
)
z_slider = Slider(
  ax=ax_zslider,
  label='z',
  valmin=-0.050,
  valmax=0.450,
  valinit=0.120,
  orientation="vertical",
  valstep=0.001
)

def update(val):
  global prev_coords
  x_slider.eventson = False
  y_slider.eventson = False
  z_slider.eventson = False

  
  armGhosts = 0
  if (len(ax.lines) > (armGhosts * 8)):
    ax.clear()
    
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')

  # Plot the arm to the target position
  target_pos = [x_slider.val, y_slider.val, z_slider.val]

#   left_arm_chain.plot(left_arm_chain.inverse_kinematics(target_pos), ax)

  # Feed back the real positions into the sliders; 
  # equivalent would be feeding back into the rover controls?
#   x_slider.set_val(np.round(real_pos[0, 3],3))
#   y_slider.set_val(np.round(real_pos[1, 3],3))
#   z_slider.set_val(np.round(real_pos[2, 3],3))

  x_slider.eventson = True
  y_slider.eventson = True
  z_slider.eventson = True

  # pt_slider.eventson = True
  # yw_slider.eventson = True
  # rl_slider.eventson = True

  

# On every slider change
x_slider.on_changed(update)
y_slider.on_changed(update)
z_slider.on_changed(update)


# Initial update
# left_arm_chain.plot(left_arm_chain.inverse_kinematics(init_coords), ax)

# real_pos = left_arm_chain.forward_kinematics(left_arm_chain.inverse_kinematics(init_coords))
# print("Computed position vector : %s, target coordinates : %s" % (np.round(real_pos[:3, 3],3), np.round(init_coords,3)))
matplotlib.pyplot.show()



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