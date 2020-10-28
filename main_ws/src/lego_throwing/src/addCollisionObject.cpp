// Made by Bastian and Mathias

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



void addCollisionObject() {
    ros::NodeHandle node_handle;
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
        sleep_t.sleep();
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::AttachedCollisionObject collision_object;
    collision_object.link_name = "base_link";
    collision_object.object.header.frame_id = "base_link";
    collision_object.object.id = "box";

    double boxHeight = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.8;
    primitive.dimensions[1] = 1.1;
    primitive.dimensions[2] = boxHeight;

    geometry_msgs::Pose pose;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = -0.3826834;
    pose.orientation.w = 0.9238795;
    pose.position.x = -0.12;
    pose.position.y = -0.35;
    pose.position.z = -boxHeight/2;

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


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    addCollisionObject();
}