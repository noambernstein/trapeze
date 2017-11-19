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
            parentFramePosition=[0,0,0], childFramePosition=pos)
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
    default_pose = { 'bodyIndex' : -1, 'jointIndices' : [], 'controlMode' : p.POSITION_CONTROL, 'targetPositions' : [], 'forces' : [] }
    for child in poses_info:
        # print(child.tag)
        if child.tag == 'defaults':
            for joint in child:
                if joint.tag != 'joint':
                    raise ValueError("Unknown tag inside defaults "+joint.tag)
                name = joint.attrib['name']
                id = joint_ids[name]
                value = float(joint.attrib['value'])
                force = float(joint.attrib['force'])
                default_pose['bodyIndex'] = flyerID
                default_pose['jointIndices'].append(id)
                default_pose['targetPositions'].append(value*deg)
                default_pose['forces'].append(force)
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
            for key in default_pose:
                try:
                    pose[key] = default_pose[key].copy()
                except:
                    pose[key] = default_pose[key]

            for joint in child:
                # print("parsing joint in pose",joint.tag,joint.attrib)
                if joint.tag != 'joint':
                    raise ValueError("Unknown tag inside pose "+joint.tag)
                joint_name=joint.attrib['name']
                joint_id=joint_ids[joint_name]
                arrays_index = pose['jointIndices'].index(joint_id)
                try:
                    pose['targetPositions'][arrays_index] = float(joint.attrib['value'])*deg
                except:
                    pass
                try:
                    pose['forces'][arrays_index] = float(joint.attrib['force'])
                except:
                    pass

            # return an array of poses so it's possible to have multiple types of control, but only position implemented for now
            poses.append((pose_key, pose_name, [pose]))
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
                    if 'key' in elem.attrib:
                        action_seq.append(('key',elem.attrib['key']))
                        if 'name' in elem.attrib:
                            raise ValueError('key and attrib in same elem of action_sequence')
                    if 'name' in elem.attrib:
                        action_seq.append(('name',elem.attrib['name']))
                        if 'key' in elem.attrib:
                            raise ValueError('key and attrib in same elem of action_sequence')
                elif elem.tag == 'wait':
                    cumul_wait += float(elem.attrib['time'])
                    action_seq.append(cumul_wait)
                else:
                    raise ValueError("Unknown tag within action_sequence "+elem.tag)
            action_sequences.append((action_seq_key, action_seq_name, action_seq))

        else:
            raise ValueError("Unknown top level tag "+child.tag)

    return (poses, action_sequences)

def print_pose(key, name, pose_a):
    print("key {} pose {}".format(key, name))
    for pose in pose_a:
        print("body",p.getBodyInfo(pose['bodyIndex']))
        for (joint_id, value, force) in zip(pose['jointIndices'],pose['targetPositions'],pose['forces']):
            print("  joint",p.getJointInfo(pose['bodyIndex'],joint_id)[1],value,force)

def print_action_seq(key, name, action_sequence):
    print("key {} action_sequence {}".format(key, name))
    for elem in action_sequence:
        if isinstance(elem,float):
            print("  wait",elem)
        else:
            print("  ",elem)

class SimulationState:

    def __init__(self, poses, action_sequences):
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

    def start_pose(self, pose_name,pose):
        print("  start pose",pose_name)
        for motorcontrol_kwargs in pose:
            p.setJointMotorControlArray(**motorcontrol_kwargs)

    def start_action_seq(self,action_name,action_sequence):
        print("  start action sequence",action_name)
        self.action_seq_start_time = time.time()
        self.current_action_seq = action_sequence
        self.action_seq_cur_index = 0

    def do_name(self,name):
        print("do name ",name)
        (action_func, action_kwargs) = self.actions_by_name[name]
        action_func(**action_kwargs)

    def do_key(self,key):
        try:
            (action_func, action_kwargs) = self.actions_by_key[key]
            print("do key '{}'".format(key))
            action_func(**action_kwargs)
        except KeyError:
            pass

    def do_action_seq_stuff(self):
        if self.current_action_seq is not None:
            if (time.time() - self.action_seq_start_time) >= self.current_action_seq[self.action_seq_cur_index]:
                action = self.current_action_seq[self.action_seq_cur_index+1][0]
                action_arg = self.current_action_seq[self.action_seq_cur_index+1][1]
                action(action_arg)
                self.action_seq_cur_index += 2
                if self.action_seq_cur_index >= len(self.current_action_seq):
                    self.current_action_seq = None
