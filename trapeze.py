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

print(flyerID,rigIDs)

# attach fly trap to crane
fly_crane_id=find_link('fly_crane')
fly_trap_id=find_link('fly_trap')
print("ATTACHING FLY TRAP TO CRANE")
fly_trap_attachment_constraints = attach_closest_point2point(fly_crane_id[0], fly_trap_id[0], distance=0.5)

# find board edge
(boardID, boardLinkID) = find_link('board')
if boardLinkID == -1:
    (board_edge, _) = p.getBasePositionAndOrientation(boardID)
    board_edge = np.array(board_edge)
else:
    raise ValueError("Board is not a base")

offset = [-0.01,0,-0.05]
if do_exercise:
    offset = [1,0,1]

# move flyer to edge of board and face the right way
(flyer_pos,flyer_orient) = p.getBasePositionAndOrientation(flyerID)
p.resetBasePositionAndOrientation(flyerID,flyer_pos + board_edge + offset,
    p.multiplyTransforms([0,0,0],flyer_orient,[0,0,0],p.getQuaternionFromEuler([0,0,180*deg]))[1])

# apply belt hold
belt_hold_constraint = fix_in_space(find_link('torso'), 'point2point')

# get ready for simulation
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=5, cameraTargetPosition=[0,0,board_edge[2]])

# exercise if requested
if len(sys.argv) > 1 and sys.argv[1] == "--exercise":
    exercise(flyerID, sys.argv[2:])

# grab on
## print("ATTACHING HANDS TO FLY BAR")
flyer_hands_attachment_constraints = attach_closest_point2point(fly_trap_id[0], flyerID, distance=0.2)

def print_pose(pose):
    print("pose {}".format(pose[0]))
    for pose in pose[1]:
        print("body",p.getBodyInfo(pose['bodyIndex']))
        for (joint_id, value, force) in zip(pose['jointIndices'],pose['targetPositions'],pose['forces']):
            print("  joint",p.getJointInfo(pose['bodyIndex'],joint_id)[1],value,force)

def print_pose_seq(pose_sequence):
    print("pose_sequence {}".format(pose_sequence[0]))
    for elem in pose_sequence[1]:
        if isinstance(elem,float):
            print("  wait",elem)
        else:
            print("  key",elem)

(poses, pose_sequences) = parse_poses(flyerID, "poses.xml")
print("KNOWN POSES:")
for key in poses:
    print ('key',key,' ',end='')
    print_pose(poses[key])
print("KNOWN POSE SEQUENCES:")
for key in pose_sequences:
    print ('key',key,' ',end='')
    print_pose_seq(pose_sequences[key])

def do_pose(pose):
    for kwargs in pose[1]:
        p.setJointMotorControlArray(**kwargs)

do_pose(poses['7'])

dt=0.005
p.setRealTimeSimulation(1)
# main loop
print("MAIN LOOP")
current_pose_seq = None
while True:
    # handle keyboard
    keys = p.getKeyboardEvents()

    # space is special
    if ord(' ') in keys and keys[ord(' ')] == 3:
        if belt_hold_constraint is not None:
            p.removeConstraint(belt_hold_constraint)
            belt_hold_constraint = None
            p.resetBaseVelocity(flyerID,linearVelocity=[0,0,2],angularVelocity=[0,1,0])
        else:
            for constraint in flyer_hands_attachment_constraints:
                p.removeConstraint(constraint)

    for key_ord in keys:
        if (keys[key_ord] & p.KEY_WAS_TRIGGERED) != 0: # key down
            key = chr(key_ord)
            if key in poses:
                in_pose_sequence = False
                print_pose(poses[key])
                do_pose(poses[key])
            elif key in pose_sequences:
                pose_seq_start_time = time.time()
                print_pose_seq(pose_sequences[key])
                current_pose_seq = pose_sequences[key][1]
                pose_seq_cur_index = 0

    if current_pose_seq is not None:
        if (time.time() - pose_seq_start_time) >= current_pose_seq[pose_seq_cur_index]:
            print("do_pose with key ",current_pose_seq[pose_seq_cur_index+1])
            do_pose(poses[current_pose_seq[pose_seq_cur_index+1]])
            pose_seq_cur_index += 2
            if pose_seq_cur_index >= len(current_pose_seq):
                current_pose_seq = None

    p.stepSimulation()
    time.sleep(dt)

p.disconnect()
