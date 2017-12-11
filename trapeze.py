#!/usr/bin/env python

import sys
import argparse

parser = argparse.ArgumentParser(description="flying trapeze simulation")
parser.add_argument('-t','--trick_name',type=str,action='store',default=None, help='name of trick triggered by key "t"')
parser.add_argument('-g','--grip_width',type=float,action='store',default=0.5, help='width of grip in m')
parser.add_argument('-m','--movie',type=str,action='store',default=None, help='name of movie file to make from action initiated by --trick_name')
parser.add_argument('-l','--movie_length',type=int,action='store',default='5', help='length of movie in seconds')
parser.add_argument('-f','--movie_fps',type=int,action='store',default='30', help='frames per second in movie')
parser.add_argument('-d','--dt',type=float,action='store',default=None, help='time step for non-real-time parts')
parser.add_argument('-F','--fast_render',action='store_true',help='fast by ugly render')
args = parser.parse_args()

import time
import numpy as np
import pybullet as p
if args.movie is not None:
    import subprocess
    from PIL import Image, ImageFont, ImageDraw
    if args.trick_name is None:
        raise ValueError("Need trick name to make a movie from")

from trapeze_utils import *

do_exercise = len(sys.argv) > 1 and sys.argv[1] == "--exercise"
################################################################################
if args.fast_render:
    physicsClient = p.connect(p.DIRECT)
    text_color = (0,0,0)
else:
    physicsClient = p.connect(p.GUI)
    text_color = (255,255,255)
################################################################################

p.setPhysicsEngineParameter(numSolverIterations=5000)

p.setGravity(0,0,-9.8)

# load flyer and rig
flyerID = p.loadMJCF("flyer.xml", flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)[0]
####################################################################################################
torso_mass = p.getDynamicsInfo(flyerID,-1)[0]
tot_mass = torso_mass
for j in range(p.getNumJoints(flyerID)):
    tot_mass += p.getDynamicsInfo(flyerID,j)[0]
print("flyer total mass",tot_mass)
print ("flyer base (torso) mass", torso_mass, int(torso_mass/tot_mass*100))
for j in range(p.getNumJoints(flyerID)):
    link_mass = p.getDynamicsInfo(flyerID,j)[0]
    print ("flyer link # name mass %",j,p.getJointInfo(flyerID,j)[-1].decode(),link_mass, int(link_mass/tot_mass*100))
####################################################################################################
rigIDs = p.loadMJCF("rig.xml")

for j in range(p.getNumJoints(flyerID)):
    p.changeDynamics(flyerID,j,linearDamping=0.01, angularDamping=0.02)
for i in rigIDs:
    for j in range(p.getNumJoints(i)):
        p.changeDynamics(i,j,linearDamping=0.01, angularDamping=0.02)

if args.dt is None:
    args.dt = p.getPhysicsEngineParameters()['fixedTimeStep']
    print("using time step",args.dt)
else:
    p.setTimeStep(args.dt)

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

flyer_board_offset_initial = [0.0,0,0.1]
flyer_board_offset_final = [0.13,0,0.005]
if do_exercise:
    flyer_board_offset_initial = [1,0,1]

# move flyer to edge of board and face the right way
(flyer_pos,flyer_orient) = p.getBasePositionAndOrientation(flyerID)
p.resetBasePositionAndOrientation(flyerID,flyer_pos + board_edge + flyer_board_offset_initial,
    p.multiplyTransforms([0,0,0],flyer_orient,[0,0,0],p.getQuaternionFromEuler([0,20*deg,180*deg]))[1])

# apply initial belt hold and serve bar
belt_hold_constraint = fix_in_space(find_link('torso'), 'point2point')
bar_serve_hold = fix_in_space(find_link('fly_bar'), 'fixed')

# get camera aimed for simulation
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=5, cameraTargetPosition=[0,0,board_edge[2]])
view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0,0,board_edge[2]], distance=5,
    yaw=0, pitch=5, roll=0, upAxisIndex=2)
proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=float(960)/float(540), nearVal=0.1, farVal=100.0)

# exercise if requested
if len(sys.argv) > 1 and sys.argv[1] == "--exercise":
    exercise(flyerID, sys.argv[2:])

# read poses (needed for 'ready')
(poses, action_sequences) = parse_poses(flyerID, "poses.xml")
print("KNOWN POSES:")
for (key,name,pose) in poses:
    print("{} {}".format(key,name))
    print_pose(pose)
print("KNOWN POSE SEQUENCES:")
for (key,name,action_seq) in action_sequences:
    print("{} {}".format(key,name))
    print_action_seq(action_seq)

# create simulation state from poses
sim_state = SimulationState(poses, action_sequences)

print ("READY")
sim_state.do_name('ready')
for i in range(int(0.5/args.dt)):
    p.stepSimulation()
    sim_state.do_pose_stuff(time.time())

# position flyer relative to board
(flyer_pos, _) = p.getBasePositionAndOrientation(flyerID)
(_, l_lower_leg_link) = find_link('l_lower_leg')
(_, r_lower_leg_link) = find_link('r_lower_leg')
(board_base, _) = find_link('board')
l_closest_points = p.getClosestPoints(bodyA=board_base, bodyB=flyerID, linkIndexB=l_lower_leg_link, distance=1.0)
min_z = flyer_pos[2]
for cp in l_closest_points:
    if cp[6][2] < min_z:
        l_foot = np.array(cp[6])
        min_z = l_foot[2]
r_closest_points = p.getClosestPoints(bodyA=board_base, bodyB=flyerID, linkIndexB=r_lower_leg_link, distance=1.0)
min_z = flyer_pos[2]
for cp in r_closest_points:
    if cp[6][2] < min_z:
        r_foot = np.array(cp[6])
        min_z = r_foot[2]
mean_foot = 0.5*(l_foot+r_foot)
dpos = board_edge + flyer_board_offset_final-mean_foot
flyer_pos = np.array(flyer_pos) + dpos

# move belt hold to new position
p.removeConstraint(belt_hold_constraint)
p.resetBasePositionAndOrientation(flyerID,flyer_pos,
    p.multiplyTransforms([0,0,0],flyer_orient,[0,0,0],p.getQuaternionFromEuler([0,20*deg,180*deg]))[1])
belt_hold_constraint = fix_in_space(find_link('torso'), 'fixed')

print("GRAB BAR")
# settle down bar position
for i in range(int(0.2/args.dt)):
    p.stepSimulation()
    sim_state.do_pose_stuff(time.time())
# release bar from hold and grab with hands
p.removeConstraint(bar_serve_hold)
bar_serve_hold = None
flyer_hands_attachment_constraints = grab_bar_with_hands(find_link('fly_bar'),find_link('l_hand'),find_link('r_hand'), args.grip_width)
# settle down hands
for i in range(int(0.2/args.dt)):
    p.stepSimulation()
    sim_state.do_pose_stuff(time.time())

###################################################################################################
## actions that aren't poses
###################################################################################################
def takeoff_release_hep(cur_time=None):
    global belt_hold_constraint, flyer_hands_attachment_constraints
    print("  takeoff/release hep")

    if belt_hold_constraint is not None: # takeoff
        p.removeConstraint(belt_hold_constraint)
        belt_hold_constraint = None
        p.resetBaseVelocity(flyerID,linearVelocity=[0,0,2.0],angularVelocity=[0,1.5,0])
        sim_state.do_name('seven')
    else: # release
        for constraint in flyer_hands_attachment_constraints:
            p.removeConstraint(constraint)

def gravity(cur_time=None):
    print("  disable gravity")
    p.setGravity(0,0,0)
    p.resetBaseVelocity(flyerID,linearVelocity=[0,0,0],angularVelocity=[0,0,0])

sim_state.register_action(key=' ', name='hep', action=takeoff_release_hep)
sim_state.register_action(key='w', name='gravity', action=gravity)

if args.trick_name is not None:
    sim_state.register_action_sequence(key='t', action_sequence_name=args.trick_name)
###################################################################################################

print("MAIN LOOP")
if args.movie is not None:
    print("equilibrating for ",int(0.5/args.dt)," steps")
    for i in range(0,int(0.5/args.dt)):
        p.stepSimulation()
        sim_state.do_pose_stuff(time.time())
    steps_per_frame = int((1.0/args.movie_fps)/args.dt)
    i_frame = 0
    print("doing movie with ",steps_per_frame," steps per frame")
    cur_time = 0.0
    sim_state.do_key('t', cur_time=cur_time)
    cmdstring = ('ffmpeg',
                 '-y',
                 '-s', '%dx%d' % (960,540),
                 '-f','image2pipe',
                 '-vcodec', 'mjpeg',
                 '-i', '-',
                 '-r', '%d' % args.movie_fps,
                 args.movie
             )
                 # '-vcodec', 'mpeg4',
    fo = open("/dev/null","w")
    ffmpeg = subprocess.Popen(cmdstring, stdin=subprocess.PIPE, stdout=fo, stderr=fo, shell=False)
    font = ImageFont.truetype("/Library/Fonts/Arial.ttf",32)
    for i_step in range(int(args.movie_length/args.dt)):
        p.stepSimulation()
        if i_step%steps_per_frame == 0:
            if i_frame%args.movie_fps == 0:
                print("doing frame",i_frame)
            if args.fast_render:
                frame = p.getCameraImage(960*2,540*2, viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                    renderer=p.ER_TINY_RENDERER)
            else:
                frame = p.getCameraImage(960*2,540*2, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            img = Image.fromarray(frame[2], 'RGBA')
            if sim_state.current_pose_name is not None:
                drw = ImageDraw.Draw(img)
                drw.text((960*2/10, 540*2/10),text=sim_state.current_pose_name,fill=text_color,font=font)
            ##### png.fromarray(frame[2],'RGBA').save("frame.{:06d}.png".format(i_frame))
            ffmpeg.stdin.write(img.convert('RGB').tobytes('jpeg','RGB'))
            #####
            i_frame += 1
        cur_time += args.dt
        sim_state.do_pose_stuff(cur_time)
    ffmpeg.stdin.close()

else:
    p.setRealTimeSimulation(1)
    cur_text = None
    cur_text_id = None
    while True:
        # handle keyboard
        keys = p.getKeyboardEvents()

        for key_ord in keys:
            if (keys[key_ord] & p.KEY_WAS_TRIGGERED) != 0: # key down
                sim_state.do_key(chr(key_ord))

        # do pose sequence things
        sim_state.do_pose_stuff(time.time())
        if sim_state.current_pose_name != cur_text:
            if cur_text_id is not None:
                p.removeUserDebugItem(cur_text_id)
            cur_text = sim_state.current_pose_name
            print("adding text",cur_text)
            if cur_text is not None:
                cur_text_id = p.addUserDebugText(cur_text, board_edge+[0,0,2], lifeTime=0)
            else:
                cur_text_id = None

p.disconnect()
