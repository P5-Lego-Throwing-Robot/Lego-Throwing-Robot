#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>

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


void modelState() {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    ROS_INFO("Model Group Name: %s", joint_model_group->getName().c_str());
    std::cout << "fuck";

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    const std::vector<std::string>& joint_namesx = joint_model_group->getJointModelNames();

    ROS_INFO("joint_names: %s", joint_names[0].c_str());
    ROS_INFO("joint_namesx: %s", joint_namesx[0].c_str());


    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
}


void planningScene() {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    collision_request.group_name = "hand";
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("manipulator");
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO_STREAM("Test 4: Current state is " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    std::string end_effector_name = joint_model_group->getLinkModelNames().back();

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.orientation.w = 1.0;
    desired_pose.pose.position.x = 0.3;
    desired_pose.pose.position.y = -0.185;
    desired_pose.pose.position.z = 0.5;
    desired_pose.header.frame_id = "base_link";
    moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

    copied_state.setToRandomPositions();
    copied_state.update();
    bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
    ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));
}



void PSROSAPI() {
    ros::NodeHandle node_handle;

    //moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    //visual_tools.deleteAllMarkers();

    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "wrist_3_link";
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "wrist_3_link";
    /* The id of the object */
    attached_object.object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    attached_object.object.operation = attached_object.object.ADD;

    attached_object.touch_links = std::vector<std::string>{ "wrist_3_link", "base_link", "base_link" };


    ROS_INFO("Adding the object into the world at the location of the hand.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
}


void addToBase() {
    ros::NodeHandle node_handle;

    //moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    //visual_tools.deleteAllMarkers();

    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    moveit_msgs::AttachedCollisionObject object;
    object.link_name = "base_link";
    object.object.header.frame_id = "base_link";
    object.object.id = "lh";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.5;

    object.object.primitives.push_back(primitive);
    object.object.primitive_poses.push_back(pose);
    object.object.operation = object.object.ADD;

    object.touch_links = std::vector<std::string>{"base_link"};

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
}


void goToGoal() {
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();


    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.2;
    target_pose1.position.y = 0.3;
    target_pose1.position.z = 0.3;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    /* Uncomment below line when working with a real robot */
    move_group.move();
}


void move() {
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.6;
    move_group.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //modelState();
    ROS_INFO("");
    ROS_INFO("");
    ROS_INFO("");
    //planningScene();
    //PSROSAPI();
    //addToBase();
    //goToGoal();
    move();




    ROS_INFO("Hello");

    return 0;
}