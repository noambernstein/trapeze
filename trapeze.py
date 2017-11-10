#!/usr/bin/env python

import sys
import time
import numpy as np
import pybullet as p

from trapeze_utils import *

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version

# p.setGravity(0,0,0)
p.setGravity(0,0,-9.8)

# load flyer and rig
flyerID = p.loadMJCF("flyer.xml")[0]
rigIDs = p.loadMJCF("rig.xml")

# attach fly trap to crane
fly_crane_id=find_link('fly_crane')
fly_trap_id=find_link('fly_trap')
fly_bar_attachment_constraints = attach_closest_point2point(fly_crane_id[0], fly_trap_id[0], distance=0.5)

# find board edge
(boardID, boardLinkID) = find_link('board')
if boardLinkID == -1:
    (board_edge, _) = p.getBasePositionAndOrientation(boardID)
    board_edge = np.array(board_edge)
else:
    Raise("Board is not a base")

# move flyer to edge of board
(flyer_pos,flyer_orient) = p.getBasePositionAndOrientation(flyerID)
p.resetBasePositionAndOrientation(flyerID,board_edge+flyer_pos,
    p.multiplyTransforms([0,0,0],flyer_orient,[0,0,0],p.getQuaternionFromEuler([0,0,180*deg]))[1])

# flyer_hands_attachment_constraints = attach_closest_point2point(fly_trap_id[0], flyerID, distance=0.2)

# get ready for simulation
dt=0.01
p.setTimeStep(dt)
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=15, cameraTargetPosition=[0,0,board_edge[2]])

# exercise if requested
if len(sys.argv) > 1 and sys.argv[1] == "--exercise":
    exercise(flyerID, sys.argv[2:])

# main loop
going=True
holding_on=True
while True:
    # keys
    keys = p.getKeyboardEvents()
    if ord(' ') in keys and keys[ord(' ')] == 3:
        going = not going
    elif ord('b') in keys and keys[ord('b')] == 3:
        if holding_on:
            for constraint in flyer_hands_attachment_constraints:
                p.removeConstraint(constraint)
        else:
            flyer_hands_attachment_constraints = attach_closest_point2point(fly_trap_id[0], flyerID, distance=0.2)
        holding_on = not holding_on

    if going:
        p.stepSimulation()
    time.sleep(dt)

p.disconnect()
