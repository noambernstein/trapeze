import time
import numpy as np
import pybullet as p

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
        Raise("Can only fix base for now")

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
        raise("Unknown fix in space type "+type)

    return c

def attach_closest_point2point(obj_A_id, obj_B_id, distance=0.1):
    closest_points = p.getClosestPoints(bodyA=obj_A_id, bodyB=obj_B_id, distance=distance)
    # print "got closest",obj_A_id, obj_B_id, len(closest_points)
    attachment_constraints = []
    for closest_pair in closest_points:
        obj_A_link_id = closest_pair[3]
        obj_B_link_id = closest_pair[4]
        obj_A_pos_world = np.array(closest_pair[5])
        obj_B_pos_world = np.array(closest_pair[6])

        # print "obj A ", obj_A_id, obj_A_link_id
        # print "obj B ", obj_B_id, obj_B_link_id
        # print "obj_A_pos_world", obj_A_pos_world
        # print "obj_B_pos_world", obj_B_pos_world

        if obj_A_link_id == -1:
            obj_A_CoM_pos_orient = p.getBasePositionAndOrientation(obj_A_id)
        else:
            s = p.getLinkState(obj_A_id, obj_A_link_id)
            # print "obj A",s
            # print p.getJointInfo(obj_A_id, obj_A_link_id)
            obj_A_CoM_pos_orient = s[0:2]
        if obj_B_link_id == -1:
            obj_B_CoM_pos_orient = p.getBasePositionBndOrientation(obj_B_id)
        else:
            s = p.getLinkState(obj_B_id, obj_B_link_id)
            # print p.getJointInfo(obj_B_id, obj_B_link_id)
            # print "obj B",s
            obj_B_CoM_pos_orient = s[0:2]

        # print "obj_A_CoM pos,orient", obj_A_CoM_pos_orient
        # print "obj_A euler", p.getEulerFromQuaternion(obj_A_CoM_pos_orient[1])
        # print "obj_B_CoM pos,orient", obj_B_CoM_pos_orient
        # print "obj_B euler", p.getEulerFromQuaternion(obj_B_CoM_pos_orient[1])

        parent_pos =  p.multiplyTransforms([0,0,0], obj_A_CoM_pos_orient[1], obj_A_pos_world-np.array(obj_A_CoM_pos_orient[0]), [0,0,0,1])[0]
        child_pos =  p.multiplyTransforms([0,0,0], obj_B_CoM_pos_orient[1], obj_B_pos_world-np.array(obj_B_CoM_pos_orient[0]), [0,0,0,1])[0]

        # print "raw parent pos",obj_A_pos_world-np.array(obj_A_CoM_pos_orient[0])
        # print "parent_pos", parent_pos
        # print "raw child pos",obj_B_pos_world-np.array(obj_B_CoM_pos_orient[0])
        # print "child_pos", child_pos

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
