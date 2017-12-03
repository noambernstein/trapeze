import time
import numpy as np
import pybullet as p
import xml.etree.ElementTree as ET

dt=0.01
deg=np.pi/180.0

def find_link(name):
    print("finding link",name)
    for i in range(p.getNumBodies()):
        id = p.getBodyUniqueId(i)
        if p.getBodyInfo(id)[0].decode() == name:
            return (id, -1)
        for i in range(p.getNumJoints(id)):
            if p.getJointInfo(id,i)[-1].decode() == name:
                return (id, i)
    return None

def fix_in_space(obj, type):
    if obj[1] == -1:
        (pos, orient) = p.getBasePositionAndOrientation(obj[0])
    else:
        raise ValueError("Can only fix base for now")

    if type == 'point2point':
        c = p.createConstraint(parentBodyUniqueId=obj[0], parentLinkIndex=obj[1],
            childBodyUniqueId=-1, childLinkIndex=-1,
            jointType=p.JOINT_POINT2POINT, jointAxis=[0,0,1],
            parentFramePosition=[0,0,0], childFramePosition=pos)
    elif type == 'fixed':
        c = p.createConstraint(parentBodyUniqueId=obj[0], parentLinkIndex=obj[1],
            childBodyUniqueId=-1, childLinkIndex=-1,
            jointType=p.JOINT_FIXED, jointAxis=[0,0,1],
            parentFramePosition=[0,0,0], parentFrameOrientation=orient, childFramePosition=pos)
    else:
        raise ValueError("Unknown fix in space type "+type)

    return c

def attach_two_world_points(obj_A_id, obj_A_link_id, obj_B_id, obj_B_link_id, obj_A_pos_world, obj_B_pos_world):
    if obj_A_link_id == -1:
        obj_A_CoM_pos_orient = p.getBasePositionAndOrientation(obj_A_id)
    else:
        s = p.getLinkState(obj_A_id, obj_A_link_id)
        obj_A_CoM_pos_orient = s[0:2]
    if obj_B_link_id == -1:
        obj_B_CoM_pos_orient = p.getBasePositionAndOrientation(obj_B_id)
    else:
        s = p.getLinkState(obj_B_id, obj_B_link_id)
        obj_B_CoM_pos_orient = s[0:2]

    parent_pos =  p.multiplyTransforms([0,0,0], obj_A_CoM_pos_orient[1], obj_A_pos_world-np.array(obj_A_CoM_pos_orient[0]), [0,0,0,1])[0]
    child_pos =  p.multiplyTransforms([0,0,0], obj_B_CoM_pos_orient[1], obj_B_pos_world-np.array(obj_B_CoM_pos_orient[0]), [0,0,0,1])[0]

    return (
        p.createConstraint(
            parentBodyUniqueId=obj_A_id, parentLinkIndex=obj_A_link_id,
            childBodyUniqueId=obj_B_id, childLinkIndex=obj_B_link_id,
            jointType=p.JOINT_POINT2POINT, jointAxis=[0,0,1],
            parentFramePosition=np.array(parent_pos),childFramePosition=np.array(child_pos))
        )


def attach_closest_point2point(obj_A_id, obj_B_id, distance=0.1):
    closest_points = p.getClosestPoints(bodyA=obj_A_id, bodyB=obj_B_id, distance=distance)
    attachment_constraints = []
    for closest_pair in closest_points:
        obj_A_link_id = closest_pair[3]
        obj_B_link_id = closest_pair[4]
        obj_A_pos_world = np.array(closest_pair[5])
        obj_B_pos_world = np.array(closest_pair[6])

        attachment_constraints.append(attach_two_world_points(obj_A_id, obj_A_link_id, obj_B_id, obj_B_link_id,
            obj_A_pos_world, obj_B_pos_world))

    return attachment_constraints

def grab_bar_with_hands(bar_id, l_hand_id, r_hand_id, grip_distance=0.5):
    l_closest_points = p.getClosestPoints(bodyA=bar_id[0], linkIndexA=bar_id[1], bodyB=l_hand_id[0], linkIndexB=l_hand_id[1], distance=0.5)
    r_closest_points = p.getClosestPoints(bodyA=bar_id[0], linkIndexA=bar_id[1], bodyB=r_hand_id[0], linkIndexB=r_hand_id[1], distance=0.5)
    if len(l_closest_points) != 1:
        raise ValueError("Confused by len(l_closest_points) = {}".format(len(l_closest_points)))
    if len(r_closest_points) > 1:
        raise ValueError("Confused by len(r_closest_points) = {}".format(len(r_closest_points)))
    l_closest_points = l_closest_points[0]
    r_closest_points = r_closest_points[0]
    print("l_closest_points",l_closest_points[5],l_closest_points[6])
    print("r_closest_points",r_closest_points[5],r_closest_points[6])
    bar_center = (np.array(l_closest_points[5]) + np.array(r_closest_points[5]))/2.0
    l_bar_vec = np.array(l_closest_points[5]) - bar_center
    r_bar_vec = np.array(r_closest_points[5]) - bar_center
    l_bar_vec /= np.linalg.norm(l_bar_vec)
    r_bar_vec /= np.linalg.norm(r_bar_vec)
    print("bar vec",l_bar_vec,r_bar_vec)
    ###
    l_bar_grip_point = bar_center + l_bar_vec*grip_distance/2.0 + np.array((0.01,0,0))
    r_bar_grip_point = bar_center + r_bar_vec*grip_distance/2.0 + np.array((0.01,0,0))
    print("grip point",l_bar_grip_point, l_closest_points[5])
    ### l_bar_grip_point = l_closest_points[5]
    ### r_bar_grip_point = r_closest_points[5]
    ###
    l_hand_grip_point = np.array(l_closest_points[6]) ### + np.array((-0.01,0,0))
    r_hand_grip_point = np.array(r_closest_points[6]) ### + np.array((-0.01,0,0))
    # print("bar_grip_points",l_bar_grip_point,r_bar_grip_point)
    # print("hand_grip_points",l_hand_grip_point,r_hand_grip_point)
    return((attach_two_world_points(bar_id[0], bar_id[1], l_hand_id[0], l_hand_id[1], l_bar_grip_point, l_hand_grip_point),
            attach_two_world_points(bar_id[0], bar_id[1], r_hand_id[0], r_hand_id[1], r_bar_grip_point, r_hand_grip_point)))



def exercise(flyerID, name_list):
    print(p.getPhysicsEngineParameters())
    p.setGravity(0,0,0)

    (flyer_pos,flyer_orient) = p.getBasePositionAndOrientation(flyerID)
    p.resetBasePositionAndOrientation(flyerID,np.array(flyer_pos)+[0,0,0.1],flyer_orient)

    for jointID in range(p.getNumJoints(flyerID)):
        joint_info = p.getJointInfo(flyerID, jointID)
        print(joint_info)
    print("")

    print("index name type pindex vindex flags damping friction lolim uplim maxf maxv linkname")

    joint_dict = {}
    for jointID in range(p.getNumJoints(flyerID)):
        joint_name = p.getJointInfo(flyerID, jointID)[1]
        joint_dict[joint_name] = jointID

    if len(name_list) > 0:
        joint_list = [joint_dict[joint_name] for joint_name in name_list]
    else:
        joint_list = range(p.getNumJoints(flyerID))

    for joint in joint_list:
        joint_info = p.getJointInfo(flyerID, joint)
        print(joint_info)
        i_frame = 0
        dir=1
        while True:
            # process keys
            keys = p.getKeyboardEvents()
            if ord('n') in keys and keys[ord('n')] == 3:
                break

            # modify motor control
            if i_frame%int(1.0/dt) == 0:
                p.setJointMotorControl2(flyerID,joint,p.VELOCITY_CONTROL,targetVelocity=6*dir,force=100)
                dir *= -1
            i_frame += 1

            # wait
            p.stepSimulation()
            time.sleep(dt)

    p.setGravity(0,0,-9.8)


def parse_poses(flyerID, file):
    joint_ids = {}
    for joint_id in range(p.getNumJoints(flyerID)):
        info = p.getJointInfo(flyerID,joint_id)
        joint_ids[info[1].decode()] = joint_id

    poses_info = ET.parse(file).getroot()
    poses = []
    action_sequences = []
    pose_default = { 'body_index' : -1, 'joint_indices' : [], 'target_positions' : [], 'speeds' : [], 'crit_angles' : [], 'max_forces' : [] }
    for child in poses_info:
        # print(child.tag)
        if child.tag == 'defaults':
            for joint in child:
                if joint.tag != 'joint':
                    raise ValueError("Unknown tag inside defaults "+joint.tag)
                name = joint.attrib['name']
                id = joint_ids[name]
                value = float(joint.attrib['value'])
                vel = float(joint.attrib['speed'])
                crit_angle = float(joint.attrib['crit_angle'])
                force = float(joint.attrib['force'])
                pose_default['body_index'] = flyerID
                pose_default['joint_indices'].append(id)
                pose_default['target_positions'].append(value*deg)
                pose_default['speeds'].append(vel*deg)
                pose_default['crit_angles'].append(crit_angle*deg)
                pose_default['max_forces'].append(force)

            pose_default['target_positions'] = np.array(pose_default['target_positions'])
            pose_default['speeds'] = np.array(pose_default['speeds'])
            pose_default['crit_angles'] = np.array(pose_default['crit_angles'])
            pose_default['max_forces'] = np.array(pose_default['max_forces'])
        elif child.tag == 'pose':
            try:
                pose_name=child.attrib['name']
            except KeyError:
                pose_name=''
            try:
                pose_key=child.attrib['key']
            except KeyError:
                pose_key=''
            pose = {}
            for key in pose_default:
                try:
                    pose[key] = pose_default[key].copy()
                except:
                    pose[key] = pose_default[key]

            pose['name'] = pose_name

            for joint in child:
                # print("parsing joint in pose",joint.tag,joint.attrib)
                if joint.tag != 'joint':
                    raise ValueError("Unknown tag inside pose "+joint.tag)
                joint_name=joint.attrib['name']
                joint_id=joint_ids[joint_name]
                arrays_index = pose['joint_indices'].index(joint_id)
                try:
                    pose['target_positions'][arrays_index] = float(joint.attrib['value'])*deg
                except:
                    pass
                try:
                    pose['speeds'][arrays_index] = float(joint.attrib['speed'])*deg
                except:
                    pass
                try:
                    pose['crit_angles'][arrays_index] = float(joint.attrib['crit_angle'])*deg
                except:
                    pass
                try:
                    pose['max_forces'][arrays_index] = float(joint.attrib['force'])
                except:
                    pass

            poses.append((pose_key, pose_name, pose))
        elif child.tag == 'action_sequence':

            try:
                action_seq_name=child.attrib['name']
            except KeyError:
                action_seq_name=''
            try:
                action_seq_key=child.attrib['key']
            except KeyError:
                action_seq_key=''

            action_seq = [0.0]
            cumul_wait = 0.0
            for elem in child:
                # print("in child",elem.tag)
                if elem.tag == 'pose':
                    if 'name' in elem.attrib:
                        action_seq.append(('name',elem.attrib['name']))
                        if 'key' in elem.attrib:
                            raise ValueError('key and attrib in same elem of action_sequence')
                    else:
                        raise ValueError('No name specified in pose in action_sequence {}'.format(action_seq_name))
                elif elem.tag == 'wait':
                    cumul_wait += float(elem.attrib['time'])
                    action_seq.append(cumul_wait)
                else:
                    raise ValueError("Unknown tag within action_sequence "+elem.tag)
            action_sequences.append((action_seq_key, action_seq_name, action_seq))

        else:
            raise ValueError("Unknown top level tag "+child.tag)

    return (poses, action_sequences)

def print_pose(pose):
    print("body",p.getBodyInfo(pose['body_index']))
    for (joint_id, value, speed, crit_angle, force) in zip(pose['joint_indices'],pose['target_positions'],pose['speeds'],pose['crit_angles'],pose['max_forces']):
        print("  joint",p.getJointInfo(pose['body_index'],joint_id)[1],"pos",value/deg,"speed",speed/deg,"crit angle",crit_angle/deg,"force",force)

def print_action_seq(action_sequence):
    for elem in action_sequence:
        if isinstance(elem,float):
            print("  wait",elem)
        else:
            print("  ",elem)

class SimulationState:

    def __init__(self, poses, action_sequences):
        self.current_pose = None
        self.current_action_seq = None
        self.action_seq_start_time=-1
        self.action_seq_cur_index=-1
        self.in_action_sequence = False
        self.poses = poses
        self.action_sequences = action_sequences
        self.actions_by_name = {}
        self.actions_by_key = {}
        for (key, name, pose) in poses:
            self.register_action(key, name, self.start_pose, pose_name=name, pose=pose)
        for (key, name, action_seq) in action_sequences:
            for i in range(0,len(action_seq),2):
                if action_seq[i+1][0] == 'key':
                    action_seq[i+1] = (self.do_key, action_seq[i+1][1])
                elif action_seq[i+1][0] == 'name':
                    action_seq[i+1] = (self.do_name, action_seq[i+1][1])
                else:
                    raise ValueError("unknown action type "+action_seq[i+1][0])
            self.register_action(key, name, self.start_action_seq, action_name=name, action_sequence=action_seq)

    def register_action_sequence(self, key, action_sequence_name):
        self.register_action(key, '', self.start_action_seq, **self.actions_by_name[action_sequence_name][1])

    def register_action(self, key, name, action, **kwargs):
        if len(key) > 0:
            self.actions_by_key[key] = (action, kwargs)
        if len(name) > 0:
            self.actions_by_name[name] = (action, kwargs)

    def start_pose(self, pose_name,pose, cur_time=None):
        print("  start pose",pose_name)
        self.current_pose = pose

    def start_action_seq(self,action_name,action_sequence, cur_time = None):
        print("  start action sequence",action_name)
        if cur_time is None:
            self.action_seq_start_time = time.time()
        else:
            self.action_seq_start_time = cur_time
        self.current_action_seq = action_sequence
        self.action_seq_cur_index = 0

    def do_name(self,name):
        print("do name ",name)
        (action_func, action_kwargs) = self.actions_by_name[name]
        action_func(**action_kwargs)

    def do_key(self,key,cur_time=None):
        try:
            (action_func, action_kwargs) = self.actions_by_key[key]
            print("do key '{}'".format(key))
            action_func(**action_kwargs, cur_time=cur_time)
        except KeyError:
            pass

    def do_pose_stuff(self, cur_time=None):
        if self.current_action_seq is not None:
            if cur_time is None:
                cur_time = time.time()
            if (cur_time - self.action_seq_start_time) >= self.current_action_seq[self.action_seq_cur_index]:
                action = self.current_action_seq[self.action_seq_cur_index+1][0]
                action_arg = self.current_action_seq[self.action_seq_cur_index+1][1]
                action(action_arg)
                self.action_seq_cur_index += 2
                if self.action_seq_cur_index >= len(self.current_action_seq):
                    self.current_action_seq = None
        if self.current_pose is not None:
            body_id = self.current_pose['body_index']
            joint_ids = self.current_pose['joint_indices']
            joint_states = p.getJointStates(body_id, joint_ids)
            joint_cur_positions = [j[0] for j in joint_states]
            delta_pos = self.current_pose['target_positions']-joint_cur_positions
            delta_pos_sign = np.sign(delta_pos)
            joint_velocities = delta_pos_sign*self.current_pose['speeds']
            crit_angles = self.current_pose['crit_angles']
            close_joints = (np.abs(delta_pos) < crit_angles)
            joint_velocities[close_joints] *= delta_pos_sign[close_joints]*delta_pos[close_joints]/crit_angles[close_joints]
            #####
            # if self.current_pose['name'] == 'flat':
                # print("POSE")
                # for i in range(len(joint_ids)):
                    # print(i,p.getJointInfo(body_id,joint_ids[i])[1],"cur pos",joint_cur_positions[i]/deg,"target pos",self.current_pose['target_positions'][i]/deg,
                        # "vel",joint_velocities[i]/deg)
            #####
            p.setJointMotorControlArray(body_id, jointIndices=joint_ids, controlMode=p.VELOCITY_CONTROL,
                targetVelocities=joint_velocities, forces=self.current_pose['max_forces'])
