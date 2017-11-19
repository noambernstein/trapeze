#!/usr/bin/env python

import sys
import argparse

parser = argparse.ArgumentParser(description="flying trapeze simulation")
parser.add_argument('-t','--trick_name',type=str,action='store',default='takeoff_hocks_style_knee_hang', help='name of trick triggered by key "t"')
parser.add_argument('-g','--grip_width',type=float,action='store',default=0.5, help='width of grip in m')
parser.add_argument('-m','--movie',type=str,action='store',default=None, help='make a movie from sequence initiated by key')
parser.add_argument('-l','--movie_length',type=int,action='store',default='5', help='length of movie in seconds')
args = parser.parse_args()

import time
import numpy as np
import pybullet as p
import png

from trapeze_utils import *

do_exercise = len(sys.argv) > 1 and sys.argv[1] == "--exercise"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setPhysicsEngineParameter(numSolverIterations=1000)

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
fly_bar_id=find_link('fly_bar')
print("ATTACHING FLY TRAP TO CRANE")
fly_bar_attachment_constraints = attach_closest_point2point(fly_crane_id[0], fly_bar_id[0], distance=0.2)

# find board edge
(boardID, boardLinkID) = find_link('board')
if boardLinkID == -1:
    (board_edge, _) = p.getBasePositionAndOrientation(boardID)
    board_edge = np.array(board_edge)
else:
    raise ValueError("Board is not a base")

flyer_board_offset = [-0.20,0,-0.08]
if do_exercise:
    flyer_board_offset = [1,0,1]

# move flyer to edge of board and face the right way
(flyer_pos,flyer_orient) = p.getBasePositionAndOrientation(flyerID)
p.resetBasePositionAndOrientation(flyerID,flyer_pos + board_edge + flyer_board_offset,
    p.multiplyTransforms([0,0,0],flyer_orient,[0,0,0],p.getQuaternionFromEuler([0,20*deg,180*deg]))[1])

# apply belt hold and serve bar
belt_hold_constraint = fix_in_space(find_link('torso'), 'point2point')
bar_serve_hold = fix_in_space(find_link('fly_bar'), 'fixed')

# get camera aimed for simulation
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=5, cameraTargetPosition=[0,0,board_edge[2]])

# exercise if requested
if len(sys.argv) > 1 and sys.argv[1] == "--exercise":
    exercise(flyerID, sys.argv[2:])

# read poses (needed for 'ready')
(poses, action_sequences) = parse_poses(flyerID, "poses.xml")
print("KNOWN POSES:")
for (key,name,pose) in poses:
    print_pose(key, name, pose)
print("KNOWN POSE SEQUENCES:")
for (key,name,action_seq) in action_sequences:
    print_action_seq(key, name, action_seq)

# create simulation state from poses
sim_state = SimulationState(poses, action_sequences)

dt=0.01
print ("READY")
p.setTimeStep(dt)
sim_state.do_key('r')
for i in range(int(0.5/dt)):
    p.stepSimulation()

print("GRAB BAR")
# settle down bar position
for i in range(int(0.2/dt)):
    p.stepSimulation()
# release bar from hold and grab with hands
p.removeConstraint(bar_serve_hold)
bar_serve_hold = None
flyer_hands_attachment_constraints = grab_bar_with_hands(find_link('fly_bar'),find_link('l_hand'),find_link('r_hand'), args.grip_width)
# settle down hands
for i in range(int(0.2/dt)):
    p.stepSimulation()


###################################################################################################
## actions that aren't poses
###################################################################################################
def takeoff_release_hep():
    global belt_hold_constraint, flyer_hands_attachment_constraints
    print("  takeoff/release hep")

    if belt_hold_constraint is not None: # takeoff
        p.removeConstraint(belt_hold_constraint)
        belt_hold_constraint = None
        p.resetBaseVelocity(flyerID,linearVelocity=[0,0,3],angularVelocity=[0,1.5,0])
        sim_state.do_key('7')
    else: # release
        for constraint in flyer_hands_attachment_constraints:
            p.removeConstraint(constraint)

def gravity():
    print("  disable gravity")
    p.setGravity(0,0,0)
    p.resetBaseVelocity(flyerID,linearVelocity=[0,0,0],angularVelocity=[0,0,0])

sim_state.register_action(key=' ', name='hep', action=takeoff_release_hep)
sim_state.register_action(key='w', name='gravity', action=gravity)

sim_state.register_action_sequence(key='t', action_sequence_name=args.trick_name)
###################################################################################################

print("MAIN LOOP")
if args.movie is not None:
    print("equilibrating for ",int(1.0/dt)," steps")
    for i in range(0,int(1.0/dt)):
        p.stepSimulation()
    steps_per_frame = int((1.0/30.0)/dt)
    i_frame = 0
    print("doing movie with ",steps_per_frame," steps per frame")
    sim_state.do_key(args.movie)
    for i in range(int(args.movie_length/dt)):
        p.stepSimulation()
        if i%steps_per_frame == 0:
            print("doing frame",i_frame)
            frame = p.getCameraImage(960,540, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            png.fromarray(frame[2],'RGBA').save("frame.{:06d}.png".format(i_frame))
            i_frame += 1

else:
    p.setRealTimeSimulation(1)
    while True:
        # handle keyboard
        keys = p.getKeyboardEvents()

        for key_ord in keys:
            if (keys[key_ord] & p.KEY_WAS_TRIGGERED) != 0: # key down
                sim_state.do_key(chr(key_ord))

        # do pose sequence things
        sim_state.do_action_seq_stuff()

p.disconnect()
