#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <pluginlib/class_loader.h>
#include <cmath>
#include <math.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_interface/planning_interface.h>

#include <boost/scoped_ptr.hpp>

double offset = 0.109492;




void move() {
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::Pose target_pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.5;
    target_pose.position.y = -0.3;
    target_pose.position.z = 0.7;
    move_group.setPoseTarget(target_pose);

    move_group.setNumPlanningAttempts(10);

    move_group.plan(my_plan);

    //move_group.execute(my_plan);

    move_group.move();

    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.5;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.7;
    move_group.setPoseTarget(target_pose);

    //move_group.setNumPlanningAttempts(10);

    move_group.plan(my_plan);

    //move_group.execute(my_plan);

    move_group.move();


    move_group.setPositionTarget(0.5, -0.1, 0.7);

    move_group.plan(my_plan);

    //move_group.execute(my_plan);

    move_group.move();

}


void cartesianPath() {
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
    //waypoints.push_back(start_pose);


    geometry_msgs::Pose target_pose = start_pose;

    target_pose.position.x += 0.5;
    target_pose.position.z += 0.3;
    waypoints.push_back(target_pose);

    target_pose.position.x += 0.2;
    target_pose.position.z += 0.2;
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("fraction: %f", fraction);

    for (int j = 0; j < trajectory.joint_trajectory.points.size(); j++) {
        for (int i = 0; i < trajectory.joint_trajectory.points[j].positions.size(); i++) {
            ROS_INFO("positions: %f", trajectory.joint_trajectory.points[j].positions[i]);
        }
        for (int i = 0; i < trajectory.joint_trajectory.points[j].velocities.size(); i++) {
            ROS_INFO("velocities: %f", trajectory.joint_trajectory.points[j].velocities[i]);
        }
        for (int i = 0; i < trajectory.joint_trajectory.points[j].accelerations.size(); i++) {
            ROS_INFO("accelerations: %f", trajectory.joint_trajectory.points[j].accelerations[i]);
        }
        ROS_INFO("");
    }

    move_group.execute(trajectory);
}


void addObject() {
    ros::NodeHandle node_handle;

    //moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    //visual_tools.deleteAllMarkers();

    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::AttachedCollisionObject collision_object;
    collision_object.link_name = "base_link";
    collision_object.object.header.frame_id = "base_link";
    collision_object.object.id = "lh";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.15;
    pose.position.y = 0.4;
    pose.position.z = -0.5;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 1.0;

    collision_object.object.primitives.push_back(primitive);
    collision_object.object.primitive_poses.push_back(pose);
    collision_object.object.operation = collision_object.object.ADD;

    collision_object.touch_links = std::vector<std::string>{"base_link"};

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object.object);
    planning_scene_interface.addCollisionObjects(collision_objects);
}


void jointTarget(double jointOneAngle) {
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    double pi = M_PI;

    std::vector<double> joints{jointOneAngle, -(pi/7)*6, -(pi/7), pi, -pi/2, 0.0};

    move_group.setJointValueTarget(joints);

    move_group.setNumPlanningAttempts(10);

    move_group.plan(my_plan);

    move_group.execute(my_plan);

    //move_group.move();


    geometry_msgs::PoseStamped pose = move_group.getCurrentPose();
    ROS_INFO("x: %f", pose.pose.position.x);
    ROS_INFO("y: %f", pose.pose.position.y);
    ROS_INFO("z: %f", pose.pose.position.z);
}


double getJointOneAngle(double x, double y) {
    double distToPoint = sqrt(pow(x, 2) + pow(y, 2));
    double theta1 = asin(offset/distToPoint);
    double theta2 = atan2(y, x);
    return theta2 - theta1;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    jointTarget(0);

    double test = getJointOneAngle(2.0, offset);
    ROS_INFO("angle: %f", test*57.2957795);

    jointTarget(test);

    //jointTarget(getJointOneAngle(2.0, 2.0));

    //jointTarget(0.0);
    //addObject();

    //move();
    cartesianPath();
}