#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <math.h>








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


    std::vector<double> joints{0.1, -pi/2, pi/2, 0.0, 0.0, 0.0};

    move_group.setJointValueTarget(joints);

    move_group.setNumPlanningAttempts(10);

    move_group.plan(my_plan);

    move_group.execute(my_plan);
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    jointTarget();

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




    std::vector<double> joint_start;
    joint_start.push_back(0.1);
    joint_start.push_back(-pi/2);
    joint_start.push_back(pi/2);
    joint_start.push_back(0.0);
    joint_start.push_back(0.0);
    joint_start.push_back(0.0);


    std::vector<double> joint_goal;
    joint_goal.push_back(pi);
    joint_goal.push_back(-pi);
    joint_goal.push_back(pi/2);
    joint_goal.push_back(0.0);
    joint_goal.push_back(0.0);
    joint_goal.push_back(0.0);



    double travelTime = 10.0;


    double a0j1 = joint_start[0];
    ROS_INFO("the start is this: %f", a0j1);
    double a0j2 = joint_start[1];
    double a0j3 = joint_start[2];
    double a0j4 = joint_start[3];
    double a0j5 = joint_start[4];
    double a0j6 = joint_start[5];

    double a1j1 = 0;
    ROS_INFO("a1j1 is this: %f", a1j1);
    double a1j2 = 0;
    double a1j3 = 0;
    double a1j4 = 0;
    double a1j5 = 0;
    double a1j6 = 0;

    double a2j1 = (3/pow(travelTime, 2))*(joint_goal[0] - joint_start[0]);
    ROS_INFO("a2j1 is this: %f", a2j1);
    double a2j2 = (3/pow(travelTime, 2))*(joint_goal[1] - joint_start[1]);
    double a2j3 = (3/pow(travelTime, 2))*(joint_goal[2] - joint_start[2]);
    double a2j4 = (3/pow(travelTime, 2))*(joint_goal[3] - joint_start[3]);
    double a2j5 = (3/pow(travelTime, 2))*(joint_goal[4] - joint_start[4]);
    double a2j6 = (3/pow(travelTime, 2))*(joint_goal[5] - joint_start[5]);

    double a3j1 = -(2/pow(travelTime, 3))*(joint_goal[0] - joint_start[0]);
    ROS_INFO("a3j1 is this: %f", a3j1);
    double a3j2 = -(2/pow(travelTime, 3))*(joint_goal[1] - joint_start[1]);
    double a3j3 = -(2/pow(travelTime, 3))*(joint_goal[2] - joint_start[2]);
    double a3j4 = -(2/pow(travelTime, 3))*(joint_goal[3] - joint_start[3]);
    double a3j5 = -(2/pow(travelTime, 3))*(joint_goal[4] - joint_start[4]);
    double a3j6 = -(2/pow(travelTime, 3))*(joint_goal[5] - joint_start[5]);



    int n = 10;





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


    for (int i = 0; i <= n; i++) {
        ROS_INFO("size: %i", trajectory2.joint_trajectory.points.size());
        trajectory_msgs::JointTrajectoryPoint point;

        double time = (travelTime/n)*i;
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
        

/*
        std::vector<double> vel;
        vel.push_back(0.0);
        vel.push_back(0.0);
        vel.push_back(0.0);
        vel.push_back(0.0);
        vel.push_back(0.0);
        vel.push_back(0.0);

        std::vector<double> acc;
        acc.push_back(0.0);
        acc.push_back(0.0);
        acc.push_back(0.0);
        acc.push_back(0.0);
        acc.push_back(0.0);
        acc.push_back(0.0);
*/


        point.positions = pos;
        point.velocities = vel;
        point.accelerations = acc;

        //point.positions = trajectory.joint_trajectory.points[i].positions;
        //point.velocities = trajectory.joint_trajectory.points[i].velocities;
        //point.accelerations = trajectory.joint_trajectory.points[i].accelerations;
        //point.effort = trajectory.joint_trajectory.points[i].effort;
        point.time_from_start = trajectory.joint_trajectory.points[i].time_from_start;

        trajectory2.joint_trajectory.points.push_back(point);
    }




    move_group.execute(trajectory2);

    ROS_INFO("done john");

    return 1;

    // get joint position, velocity and acceleration for n intervals
}