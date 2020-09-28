#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <math.h>

#include "lego_throwing/FindBestInverseKinematicsSolution.h"
#include <cstdlib>







double pi = M_PI;






std::vector<double> inverse(geometry_msgs::Pose pose) {
    std::vector<double> joint_values;

    std::size_t attempts = 10;
    double timeout = 1.0;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");

    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();



    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, attempts, timeout);

    if (found_ik) {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    } else {
        ROS_INFO("Did not find IK solution");
    }

    return joint_values;
}





void setPose(geometry_msgs::Pose pose) {
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    std::vector<double> joint_goal = inverse(pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setJointValueTarget(joint_goal);
    move_group.setNumPlanningAttempts(10);
    move_group.plan(my_plan);
    move_group.execute(my_plan);
}







void jointTarget() {
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::vector<double> joints{0.58715242, -2.06680655, -1.59263468, 0.44127813, 1.18455327, 1.57040942};

    move_group.setJointValueTarget(joints);

    move_group.setNumPlanningAttempts(10);

    move_group.plan(my_plan);

    move_group.execute(my_plan);
}





std::vector<double> getJointVelocities(std::vector<double> endEffectorVelocities, std::vector<double> jointAngles) {

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    kinematic_state->setJointGroupPositions(joint_model_group, jointAngles);

    Eigen::MatrixXd jacobian;
    jacobian = kinematic_state->getJacobian(joint_model_group);

    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    Eigen::MatrixXd inverseJacobian(6, 6);
    inverseJacobian = jacobian.inverse();
    ROS_INFO_STREAM("inverseJacobian: \n" << inverseJacobian << "\n");
    
    Eigen::MatrixXd ide = inverseJacobian*jacobian;
    ROS_INFO_STREAM("ide: \n" << inverseJacobian*jacobian << "\n");




    Eigen::MatrixXd endEffectorVelocitiesMatrix(6, 1);
    endEffectorVelocitiesMatrix << endEffectorVelocities[0], endEffectorVelocities[1], endEffectorVelocities[2], endEffectorVelocities[3], endEffectorVelocities[4], endEffectorVelocities[5];
    ROS_INFO_STREAM("bumbum: \n" << endEffectorVelocitiesMatrix << "\n");




    Eigen::MatrixXd jointVelocitiesMatrix = inverseJacobian*endEffectorVelocitiesMatrix;
    ROS_INFO_STREAM("bumbum: \n" << jointVelocitiesMatrix << "\n");

    std::vector<double> jointVelocities;
    jointVelocities.push_back(jointVelocitiesMatrix(0, 0));
    jointVelocities.push_back(jointVelocitiesMatrix(1, 0));
    jointVelocities.push_back(jointVelocitiesMatrix(2, 0));
    jointVelocities.push_back(jointVelocitiesMatrix(3, 0));
    jointVelocities.push_back(jointVelocitiesMatrix(4, 0));
    jointVelocities.push_back(jointVelocitiesMatrix(5, 0));

    return jointVelocities;
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "add_two_ints_client");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    //jointTarget();

    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;

    geometry_msgs::Pose target_pose = start_pose;
    target_pose.position.z += 0.15;
    target_pose.position.y -= 0.2;

    ROS_INFO("start pose:");
    ROS_INFO("  pos x: %f", start_pose.position.x);
    ROS_INFO("  pos y: %f", start_pose.position.y);
    ROS_INFO("  pos z: %f", start_pose.position.z);
    ROS_INFO("  quar x: %f", start_pose.orientation.x);
    ROS_INFO("  quar y: %f", start_pose.orientation.y);
    ROS_INFO("  quar z: %f", start_pose.orientation.z);
    ROS_INFO("  quar w: %f", start_pose.orientation.w);
    ROS_INFO("");
    ROS_INFO("target pose:");
    ROS_INFO("  pos x: %f", target_pose.position.x);
    ROS_INFO("  pos y: %f", target_pose.position.y);
    ROS_INFO("  pos z: %f", target_pose.position.z);
    ROS_INFO("  quar x: %f", target_pose.orientation.x);
    ROS_INFO("  quar y: %f", target_pose.orientation.y);
    ROS_INFO("  quar z: %f", target_pose.orientation.z);
    ROS_INFO("  quar w: %f", target_pose.orientation.w);
    ROS_INFO("");



    //setPose(start_pose);








    geometry_msgs::Pose test_pose;
    test_pose.position.x = 0.5;
    test_pose.position.y = 0.45;
    test_pose.position.z = 0.5;
    test_pose.orientation.w = 0.4;
    test_pose.orientation.x = 0.5;
    test_pose.orientation.y = -0.6922559;
    test_pose.orientation.z = 0.0206427;
    move_group.setPoseTarget(test_pose);
    moveit::planning_interface::MoveGroupInterface::Plan test_plan;
    move_group.plan(test_plan);
    move_group.move();

    std::vector<double> currentJointValues = move_group.getCurrentJointValues();
    ROS_INFO("joint: %f", currentJointValues[0]);
    ROS_INFO("joint: %f", currentJointValues[1]);
    ROS_INFO("joint: %f", currentJointValues[2]);
    ROS_INFO("joint: %f", currentJointValues[3]);
    ROS_INFO("joint: %f", currentJointValues[4]);
    ROS_INFO("joint: %f", currentJointValues[5]);



    ros::init(argc, argv, "inverse_kinematics_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<lego_throwing::FindBestInverseKinematicsSolution>("inverse_kinematics");
    lego_throwing::FindBestInverseKinematicsSolution srv;

    srv.request.pose = test_pose;
    srv.request.jointAngles = currentJointValues;

    if (client.call(srv)) {
        std::vector<double> bestSolution = srv.response.bestSolution;
        ROS_INFO("Sum: %f", bestSolution[1]);



        moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
        move_group2.setJointValueTarget(bestSolution);
        move_group2.setNumPlanningAttempts(10);
        move_group2.plan(my_plan2);
        move_group2.execute(my_plan2);

    } else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }








    //std::vector<double> test = getJointVelocities({0.2, 0.2, 0.3, 0.0, 0.0, 0.0}, {1.3, 1.7, 0.2, 0.5, 0.6, 0.8});



    std::vector<double> joint_start;
    joint_start.push_back(0.1);
    joint_start.push_back(-pi/2);
    joint_start.push_back(pi/2);
    joint_start.push_back(0.0);
    joint_start.push_back(0.0);
    joint_start.push_back(0.0);

    std::vector<double> joint_start_vel;
    joint_start_vel.push_back(0.0);
    joint_start_vel.push_back(0.0);
    joint_start_vel.push_back(0.0);
    joint_start_vel.push_back(0.0);
    joint_start_vel.push_back(0.0);
    joint_start_vel.push_back(0.0);



    std::vector<double> joint_goal;
    joint_goal.push_back(pi);
    joint_goal.push_back(-pi);
    joint_goal.push_back(pi/2);
    joint_goal.push_back(0.0);
    joint_goal.push_back(0.0);
    joint_goal.push_back(0.0);

    std::vector<double> joint_goal_vel;
    joint_goal_vel.push_back(0.0);
    joint_goal_vel.push_back(0.0);
    joint_goal_vel.push_back(0.0);
    joint_goal_vel.push_back(0.0);
    joint_goal_vel.push_back(0.0);
    joint_goal_vel.push_back(0.0);





    double travelTime = 5.0;


    double a0j1 = joint_start[0];
    ROS_INFO("the start is this: %f", a0j1);
    double a0j2 = joint_start[1];
    double a0j3 = joint_start[2];
    double a0j4 = joint_start[3];
    double a0j5 = joint_start[4];
    double a0j6 = joint_start[5];

    double a1j1 = joint_start_vel[0];
    ROS_INFO("a1j1 is this: %f", a1j1);
    double a1j2 = joint_start_vel[1];
    double a1j3 = joint_start_vel[2];
    double a1j4 = joint_start_vel[3];
    double a1j5 = joint_start_vel[4];
    double a1j6 = joint_start_vel[5];

    double a2j1 = (3/pow(travelTime, 2)) * (joint_goal[0] - joint_start[0]) - (2/travelTime) * joint_start_vel[0] - (1/travelTime) * joint_goal_vel[0];
    ROS_INFO("a2j1 is this: %f", a2j1);
    double a2j2 = (3/pow(travelTime, 2)) * (joint_goal[1] - joint_start[1]) - (2/travelTime) * joint_start_vel[1] - (1/travelTime) * joint_goal_vel[1];
    double a2j3 = (3/pow(travelTime, 2)) * (joint_goal[2] - joint_start[2]) - (2/travelTime) * joint_start_vel[2] - (1/travelTime) * joint_goal_vel[2];
    double a2j4 = (3/pow(travelTime, 2)) * (joint_goal[3] - joint_start[3]) - (2/travelTime) * joint_start_vel[3] - (1/travelTime) * joint_goal_vel[3];
    double a2j5 = (3/pow(travelTime, 2)) * (joint_goal[4] - joint_start[4]) - (2/travelTime) * joint_start_vel[4] - (1/travelTime) * joint_goal_vel[4];
    double a2j6 = (3/pow(travelTime, 2)) * (joint_goal[5] - joint_start[5]) - (2/travelTime) * joint_start_vel[5] - (1/travelTime) * joint_goal_vel[5];

    double a3j1 = -(2/pow(travelTime, 3)) * (joint_goal[0] - joint_start[0]) + (1/pow(travelTime, 2)) * (joint_goal_vel[0] - joint_start_vel[0]);
    ROS_INFO("a3j1 is this: %f", a3j1);
    double a3j2 = -(2/pow(travelTime, 3)) * (joint_goal[1] - joint_start[1]) + (1/pow(travelTime, 2)) * (joint_goal_vel[1] - joint_start_vel[1]);
    double a3j3 = -(2/pow(travelTime, 3)) * (joint_goal[2] - joint_start[2]) + (1/pow(travelTime, 2)) * (joint_goal_vel[2] - joint_start_vel[2]);
    double a3j4 = -(2/pow(travelTime, 3)) * (joint_goal[3] - joint_start[3]) + (1/pow(travelTime, 2)) * (joint_goal_vel[3] - joint_start_vel[3]);
    double a3j5 = -(2/pow(travelTime, 3)) * (joint_goal[4] - joint_start[4]) + (1/pow(travelTime, 2)) * (joint_goal_vel[4] - joint_start_vel[4]);
    double a3j6 = -(2/pow(travelTime, 3)) * (joint_goal[5] - joint_start[5]) + (1/pow(travelTime, 2)) * (joint_goal_vel[5] - joint_start_vel[5]);



    int steps = 10;





    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;

    
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("fraction: %f", fraction);



    moveit_msgs::RobotTrajectory trajectory2;

    //trajectory2.joint_trajectory.header = trajectory.joint_trajectory.header;
    trajectory2.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
    //trajectory2.joint_trajectory.points = trajectory.joint_trajectory.points;


    for (int i = 0; i <= steps; i++) {
        ROS_INFO("size: %i", trajectory2.joint_trajectory.points.size());
        trajectory_msgs::JointTrajectoryPoint point;

        double time = (travelTime/steps)*i;
        ROS_INFO("the time is: %f", time);

        std::vector<double> pos;
        pos.push_back(a0j1 + a1j1*time + a2j1*pow(time, 2) + a3j1*pow(time, 3));
        pos.push_back(a0j2 + a1j2*time + a2j2*pow(time, 2) + a3j2*pow(time, 3));
        pos.push_back(a0j3 + a1j3*time + a2j3*pow(time, 2) + a3j3*pow(time, 3));
        pos.push_back(a0j4 + a1j4*time + a2j4*pow(time, 2) + a3j4*pow(time, 3));
        pos.push_back(a0j5 + a1j5*time + a2j5*pow(time, 2) + a3j5*pow(time, 3));
        pos.push_back(a0j6 + a1j6*time + a2j6*pow(time, 2) + a3j6*pow(time, 3));
        ROS_INFO("pos: %f, %f", time, (a0j1 + a1j1*time + a2j1*pow(time, 2) + a3j1*pow(time, 3)));


        std::vector<double> vel;
        vel.push_back(a1j1 + 2*a2j1*time + 3*a3j1*pow(time, 2));
        vel.push_back(a1j2 + 2*a2j2*time + 3*a3j2*pow(time, 2));
        vel.push_back(a1j3 + 2*a2j3*time + 3*a3j3*pow(time, 2));
        vel.push_back(a1j4 + 2*a2j4*time + 3*a3j4*pow(time, 2));
        vel.push_back(a1j5 + 2*a2j5*time + 3*a3j5*pow(time, 2));
        vel.push_back(a1j6 + 2*a2j6*time + 3*a3j6*pow(time, 2));

        std::vector<double> acc;
        acc.push_back(2*a2j1 + 6*a3j1*time);
        acc.push_back(2*a2j1 + 6*a3j1*time);
        acc.push_back(2*a2j1 + 6*a3j1*time);
        acc.push_back(2*a2j1 + 6*a3j1*time);
        acc.push_back(2*a2j1 + 6*a3j1*time);
        acc.push_back(2*a2j1 + 6*a3j1*time);
        



        point.positions = pos;
        point.velocities = vel;
        point.accelerations = acc;

        ros::Duration seconds(time);
        point.time_from_start = seconds;

        //point.positions = trajectory.joint_trajectory.points[i].positions;
        //point.velocities = trajectory.joint_trajectory.points[i].velocities;
        //point.accelerations = trajectory.joint_trajectory.points[i].accelerations;
        //point.effort = trajectory.joint_trajectory.points[i].effort;
        //point.time_from_start = trajectory.joint_trajectory.points[i].time_from_start;

        trajectory2.joint_trajectory.points.push_back(point);
    }




    //move_group.execute(trajectory2);


    ROS_INFO("done john");

    return 1;
}