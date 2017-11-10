#!/usr/bin/env python

import sys
import time
import numpy as np
import pybullet as p

from trapeze_utils import *

do_exercise = len(sys.argv) > 1 and sys.argv[1] == "--exercise"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version

# p.setGravity(0,0,0)
p.setGravity(0,0,-9.8)

# load flyer and rig
flyerID = p.loadMJCF("flyer.xml")[0]
rigIDs = p.loadMJCF("rig.xml")

# attach fly trap to crane
fly_crane_id=find_link('fly_crane')
fly_trap_id=find_link('fly_trap')
print "ATTACHING FLY TRAP TO CRANE"
fly_trap_attachment_constraints = attach_closest_point2point(fly_crane_id[0], fly_trap_id[0], distance=0.5)

# find board edge
(boardID, boardLinkID) = find_link('board')
if boardLinkID == -1:
    (board_edge, _) = p.getBasePositionAndOrientation(boardID)
    board_edge = np.array(board_edge)
else:
    Raise("Board is not a base")

offset = [0,0,0]
if do_exercise:
    offset = [1,0,1]

# move flyer to edge of board and face the right way
(flyer_pos,flyer_orient) = p.getBasePositionAndOrientation(flyerID)
p.resetBasePositionAndOrientation(flyerID,flyer_pos + board_edge + offset,
    p.multiplyTransforms([0,0,0],flyer_orient,[0,0,0],p.getQuaternionFromEuler([0,0,180*deg]))[1])

# apply belt hold
belt_hold_constraint = fix_in_space(find_link('torso'), 'point2point')

# get ready for simulation
dt=0.01
p.setTimeStep(dt)
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=15, cameraTargetPosition=[0,0,board_edge[2]])

# exercise if requested
if len(sys.argv) > 1 and sys.argv[1] == "--exercise":
    exercise(flyerID, sys.argv[2:])

# grab on
print "ATTACHING HANDS TO FLY BAR"
flyer_hands_attachment_constraints = attach_closest_point2point(fly_trap_id[0], flyerID, distance=0.2)

# main loop
while True:
    start_time = time.time()
    # keys
    keys = p.getKeyboardEvents()
    # if ord(' ') in keys and keys[ord(' ')] == 3:
        # going = not going
    if ord('h') in keys and keys[ord('h')] == 3:
        p.removeConstraint(belt_hold_constraint)
        p.resetBaseVelocity(flyerID,linearVelocity=[0,0,2],angularVelocity=[0,1,0])
    elif ord('b') in keys and keys[ord('b')] == 3:
        for constraint in flyer_hands_attachment_constraints:
            p.removeConstraint(constraint)
    for i in range(10):
        p.stepSimulation()
    sleep_time = dt*10-(time.time()-start_time)
    if sleep_time > 0.0:
        time.sleep(sleep_time)

p.disconnect()
