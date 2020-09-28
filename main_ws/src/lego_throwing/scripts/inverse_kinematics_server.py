#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import ikfastpy
from lego_throwing.srv import FindBestInverseKinematicsSolution, FindBestInverseKinematicsSolutionResponse
import rospy
from tf.transformations import quaternion_matrix
from kinematics import *


# Initialize kinematics for UR5 robot arm
ur5_kin = ikfastpy.PyKinematics()
n_joints = ur5_kin.getDOF()


def handle_inverse_kinematics(req):
    ee_pose = req.pose
    joint_angles = req.jointAngles


    transformation_matrix = quaternion_matrix([ee_pose.orientation.x,
                                               ee_pose.orientation.y,
                                               ee_pose.orientation.z,
                                               ee_pose.orientation.w])

    transformation_matrix[0][3] = ee_pose.position.x
    transformation_matrix[1][3] = ee_pose.position.y
    transformation_matrix[2][3] = ee_pose.position.z

    transformation_matrix = np.delete(transformation_matrix, 3, 0)

    print("ee_pose:\n")
    print(ee_pose)
    print("\n")
    print("joint_angles:\n")
    print(joint_angles)
    print("\n")
    print("trans_matrix:\n")
    print(transformation_matrix)
    print("\n")

    joint_configs = ur5_kin.inverse(transformation_matrix.reshape(-1).tolist())




    n_solutions = int(len(joint_configs)/n_joints)


    joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)

    print("joint_configs:\n")
    print(joint_configs)

    motion = []
    for index in range(len(joint_configs)):
        motion.append(np.abs(joint_configs[index] - (joint_angles)))

    #print(motion)
    print("\n\n")

    test = inv_kin(ee_pose, joint_angles)

    print("test_joint_configs:\n")
    print(test)

    return FindBestInverseKinematicsSolutionResponse(test)



    maximumVal = []
    for index in range(len(motion)):
        maximumVal.append(max(motion[index]))

    indexForBestSol = np.where(maximumVal == min(maximumVal))

    print(indexForBestSol)


    bestSolution = joint_configs[indexForBestSol][3]

    print(bestSolution)





    #bestSolution = [1.0, 3.4, 7.7, 2.2, 2.2, 9.8]




    return FindBestInverseKinematicsSolutionResponse(bestSolution)

def inverse_kinematics_server():
    rospy.init_node('inverse_kinematics_server')
    s = rospy.Service('inverse_kinematics', FindBestInverseKinematicsSolution, handle_inverse_kinematics)
    print("Hello Universe")
    rospy.spin()

if __name__ == "__main__":
    inverse_kinematics_server()


















