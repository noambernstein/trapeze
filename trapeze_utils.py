import time
import numpy as np
import pybullet as p
import xml.etree.ElementTree as ET

dt=0.01
deg=np.pi/180.0

def find_link(name):
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
        c = [p.createConstraint(parentBodyUniqueId=obj[0], parentLinkIndex=obj[1],
                childBodyUniqueId=-1, childLinkIndex=-1,
                jointType=p.JOINT_POINT2POINT, jointAxis=[0,0,1],
                parentFramePosition=[0,0,0], childFramePosition=pos),
             p.createConstraint(parentBodyUniqueId=obj[0], parentLinkIndex=obj[1],
                childBodyUniqueId=-1, childLinkIndex=-1,
                jointType=p.JOINT_POINT2POINT, jointAxis=[0,0,1],
                parentFramePosition=[0.1,0,0], childFramePosition=np.array(pos)-[0.1,0,0]),
             p.createConstraint(parentBodyUniqueId=obj[0], parentLinkIndex=obj[1],
                childBodyUniqueId=-1, childLinkIndex=-1,
                jointType=p.JOINT_POINT2POINT, jointAxis=[0,0,1],
                parentFramePosition=[0,0.1,0], childFramePosition=np.array(pos)-[0,0.1,0])
            ]
    else:
        raise ValueError("Unknown fix in space type "+type)

    return c

def attach_closest_point2point(obj_A_id, obj_B_id, distance=0.1):
    closest_points = p.getClosestPoints(bodyA=obj_A_id, bodyB=obj_B_id, distance=distance)
    attachment_constraints = []
    for closest_pair in closest_points:
        obj_A_link_id = closest_pair[3]
        obj_B_link_id = closest_pair[4]
        obj_A_pos_world = np.array(closest_pair[5])
        obj_B_pos_world = np.array(closest_pair[6])

        if obj_A_link_id == -1:
            obj_A_CoM_pos_orient = p.getBasePositionAndOrientation(obj_A_id)
        else:
            s = p.getLinkState(obj_A_id, obj_A_link_id)
            obj_A_CoM_pos_orient = s[0:2]
        if obj_B_link_id == -1:
            obj_B_CoM_pos_orient = p.getBasePositionBndOrientation(obj_B_id)
        else:
            s = p.getLinkState(obj_B_id, obj_B_link_id)
            obj_B_CoM_pos_orient = s[0:2]

        parent_pos =  p.multiplyTransforms([0,0,0], obj_A_CoM_pos_orient[1], obj_A_pos_world-np.array(obj_A_CoM_pos_orient[0]), [0,0,0,1])[0]
        child_pos =  p.multiplyTransforms([0,0,0], obj_B_CoM_pos_orient[1], obj_B_pos_world-np.array(obj_B_CoM_pos_orient[0]), [0,0,0,1])[0]

        attachment_constraints.append(
            p.createConstraint(
                parentBodyUniqueId=obj_A_id, parentLinkIndex=obj_A_link_id,
                childBodyUniqueId=obj_B_id, childLinkIndex=obj_B_link_id,
                jointType=p.JOINT_POINT2POINT, jointAxis=[0,0,1],
                parentFramePosition=np.array(parent_pos),childFramePosition=np.array(child_pos))
            )
    return attachment_constraints


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
    poses = {}
    pose_sequences = {}
    default_pose = { 'bodyIndex' : -1, 'jointIndices' : [], 'controlMode' : p.POSITION_CONTROL, 'targetPositions' : [], 'forces' : [] }
    for child in poses_info:
        print(child.tag)
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
            # print("got default_pose",default_pose)
        elif child.tag == 'pose':
            pose_name=child.attrib['name']
            pose_key=child.attrib['key']
            pose = {}
            for key in default_pose:
                try:
                    pose[key] = default_pose[key].copy()
                except:
                    pose[key] = default_pose[key]

            for joint in child:
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

            # print("got pose",key, pose_name)
            # return an array of poses so it's possible to have multiple types of control, but only position implemented for now
            poses[pose_key] = (pose_name, [pose])
        elif child.tag == 'pose_sequence':
            pose_seq_name=child.attrib['name']
            pose_seq_key=child.attrib['key']
            pose_seq = [0.0]
            cumul_wait = 0.0
            for elem in child:
                print("in child",elem.tag)
                if elem.tag == 'pose':
                    pose_seq.append(elem.attrib['key'])
                elif elem.tag == 'wait':
                    cumul_wait += float(elem.attrib['time'])
                    pose_seq.append(cumul_wait)
                else:
                    raise ValueError("Unknown tag within pose_sequence "+elem.tag)
            pose_sequences[pose_seq_key] = (pose_seq_name, pose_seq)

        else:
            raise ValueError("Unknown top level tag "+child.tag)

    return (poses, pose_sequences)
