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

for j in range(p.getNumJoints(flyerID)):
    p.changeDynamics(flyerID,j,linearDamping=0.01, angularDamping=0.0)
for i in rigIDs:
    for j in range(p.getNumJoints(i)):
        p.changeDynamics(i,j,linearDamping=0.01, angularDamping=0.0)

print(flyerID,rigIDs)

# attach fly trap to crane
fly_crane_id=find_link('fly_crane')
fly_trap_id=find_link('fly_trap')
print("ATTACHING FLY TRAP TO CRANE")
fly_trap_attachment_constraints = attach_closest_point2point(fly_crane_id[0], fly_trap_id[0], distance=0.2)

# find board edge
(boardID, boardLinkID) = find_link('board')
if boardLinkID == -1:
    (board_edge, _) = p.getBasePositionAndOrientation(boardID)
    board_edge = np.array(board_edge)
else:
    raise ValueError("Board is not a base")

offset = [-0.30,0,-0.08]
if do_exercise:
    offset = [1,0,1]

# move flyer to edge of board and face the right way
(flyer_pos,flyer_orient) = p.getBasePositionAndOrientation(flyerID)
p.resetBasePositionAndOrientation(flyerID,flyer_pos + board_edge + offset,
    p.multiplyTransforms([0,0,0],flyer_orient,[0,0,0],p.getQuaternionFromEuler([0,20*deg,180*deg]))[1])

# apply belt hold and hold bar in place
belt_hold_constraint = fix_in_space(find_link('torso'), 'point2point')
bar_serve_hold = fix_in_space(find_link('fly_trap'), 'point2point')

# get ready for simulation
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=5, cameraTargetPosition=[0,0,board_edge[2]])

# exercise if requested
if len(sys.argv) > 1 and sys.argv[1] == "--exercise":
    exercise(flyerID, sys.argv[2:])

(poses, pose_sequences) = parse_poses(flyerID, "poses.xml")
print("KNOWN POSES:")
for key in poses:
    print ('key',key,' ',end='')
    print_pose(poses[key])
print("KNOWN POSE SEQUENCES:")
for key in pose_sequences:
    print ('key',key,' ',end='')
    print_pose_seq(pose_sequences[key])



dt=0.005
p.setRealTimeSimulation(1)
# main loop
print("MAIN LOOP")
sim_state = SimulationState(poses, pose_sequences)

sim_state.do_key('r')

# settle down bar position
t0 = time.time()
while time.time()-t0 < 0.4:
    if time.time() - t0 > 0.2 and bar_serve_hold is not None:
        p.removeConstraint(bar_serve_hold)
        base_serve_hold = None
    p.stepSimulation()


# grab on
print("ATTACHING HANDS TO FLY BAR")
flyer_hands_attachment_constraints = attach_closest_point2point(fly_trap_id[0], flyerID, distance=0.3)

def takeoff_release_hep():
    global belt_hold_constraint, flyer_hands_attachment_constraints
    if belt_hold_constraint is not None: # takeoff
        p.removeConstraint(belt_hold_constraint)
        belt_hold_constraint = None
        p.resetBaseVelocity(flyerID,linearVelocity=[0,0,3],angularVelocity=[0,2,0])
        sim_state.do_key('7')
    else: # release
        for constraint in flyer_hands_attachment_constraints:
            p.removeConstraint(constraint)

sim_state.register_key(key=' ', name='hep', action=takeoff_release_hep)

while True:
    # handle keyboard
    keys = p.getKeyboardEvents()

    for key_ord in keys:
        if (keys[key_ord] & p.KEY_WAS_TRIGGERED) != 0: # key down
            sim_state.do_key(chr(key_ord))

    sim_state.do_pose_seq_stuff()

    p.stepSimulation()
    time.sleep(dt)

p.disconnect()
