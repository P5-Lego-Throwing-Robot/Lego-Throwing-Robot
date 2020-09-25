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
    target_pose.position.z += 0.5;
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.05;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("fraction: %f", fraction);

    double positions[trajectory.joint_trajectory.points.size()][6];


    moveit_msgs::RobotTrajectory trajectory2;

    std::vector<trajectory_msgs::JointTrajectoryPoint> test;


    ROS_INFO("1111");
    for (int j = 0; j < trajectory.joint_trajectory.points.size(); j++) {
        ROS_INFO("2222");

        trajectory_msgs::JointTrajectoryPoint point;

        ROS_INFO("3333");
        
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> acc;

        ROS_INFO("4444");




        for (int a = 0; a < trajectory.joint_trajectory.points[j].positions.size(); a++) {
            ROS_INFO("aaaa");
            //trajectory.joint_trajectory.points[j].positions[a] = 0.0;
            pos.push_back(trajectory.joint_trajectory.points[j].positions[a]);
            ROS_INFO("bbbb");
            ROS_INFO("fffffff: %f", pos[a]);
            ROS_INFO("cccc");
        }
        for (int b = 0; b < trajectory.joint_trajectory.points[j].velocities.size(); b++) {
            //trajectory.joint_trajectory.points[j].velocities[b] = 0.0;
            ROS_INFO("dddd");
            vel.push_back(trajectory.joint_trajectory.points[j].velocities[b]);
            ROS_INFO("eeee");
        }
        for (int c = 0; c < trajectory.joint_trajectory.points[j].accelerations.size(); c++) {
            //trajectory.joint_trajectory.points[j].accelerations[c] = 0.0;
            ROS_INFO("ffff");
            acc.push_back(trajectory.joint_trajectory.points[j].accelerations[c]);
            ROS_INFO("gggg");
        }

        ROS_INFO("5555");

        point.positions = pos;
        point.velocities = vel;
        point.accelerations = acc;

        ROS_INFO("6666");

        test.push_back(point);

        ROS_INFO("7777");

        ROS_INFO("seq: %i", trajectory.joint_trajectory.header.seq);
        ROS_INFO("sec: %i", trajectory.joint_trajectory.header.stamp.sec);
        ROS_INFO("nsec: %i", trajectory.joint_trajectory.header.stamp.nsec);
        ROS_INFO("frame_id: %s", trajectory.joint_trajectory.header.frame_id.c_str());
        for (int i = 0; i < trajectory.joint_trajectory.points[j].velocities.size(); i++) {
            //ROS_INFO("effort: %f", trajectory.joint_trajectory.points[j].accelerations[i]);
        }
        ROS_INFO(" ");
    }






    ROS_INFO("8888");

    trajectory2.joint_trajectory.points = test;

    std_msgs::Header header;
    header.frame_id = "world";
    trajectory2.joint_trajectory.header = header;

    trajectory2.joint_trajectory.header.seq = trajectory.joint_trajectory.header.seq;
    trajectory2.joint_trajectory.header.stamp = trajectory.joint_trajectory.header.stamp;

    trajectory2.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;

    ROS_INFO("joint name: %s", trajectory2.joint_trajectory.joint_names[1].c_str());

    trajectory2.joint_trajectory = trajectory.joint_trajectory;

    ROS_INFO("9999");

    ROS_INFO("fractsdfsdfsdion: %f", trajectory2.joint_trajectory.points[3].positions[1]);

    move_group.execute(trajectory2);
}


void testTraj() {
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
    //waypoints.push_back(start_pose);


    geometry_msgs::Pose target_pose = start_pose;

    target_pose.position.x += 0.5;
    target_pose.position.z += 0.5;
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.05;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("fraction: %f", fraction);

    moveit_msgs::RobotTrajectory trajectory2;

    //trajectory2.joint_trajectory.header = trajectory.joint_trajectory.header;
    trajectory2.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
    //trajectory2.joint_trajectory.points = trajectory.joint_trajectory.points;

    for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++) {
        ROS_INFO("size: %i", trajectory2.joint_trajectory.points.size());
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = trajectory.joint_trajectory.points[i].positions;
        point.velocities = trajectory.joint_trajectory.points[i].velocities;
        point.accelerations = trajectory.joint_trajectory.points[i].accelerations;
        point.effort = trajectory.joint_trajectory.points[i].effort;
        point.time_from_start = trajectory.joint_trajectory.points[i].time_from_start;
        trajectory2.joint_trajectory.points.push_back(point);
    }

    move_group.execute(trajectory2);
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
    testTraj();
}