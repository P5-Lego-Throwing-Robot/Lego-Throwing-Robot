#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <math.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
    

const std::string PLANNING_GROUP = "manipulator";
const double PI = M_PI;

double throwingVelocity = 2.0; // m/s
double throwingAngle = PI/4; // radians
double offset = 0.109492;
/*
std::vector<double> joint_position_start{0.0, -PI/2, (PI*3)/4, 0.0, PI/2, 0.0};
std::vector<double> joint_position_throw{0.0, -PI/3, PI/5, 0.0, PI/2, 0.0};
std::vector<double> joint_position_end{0.0, -PI/5, -PI/5, 0.0, PI/2, 0.0};
*/
/*
std::vector<double> joint_position_start{0.0, -(PI*7)/8, -(PI*1)/8, 0.0, PI/2, 0.0};
std::vector<double> joint_position_throw{0.0, -(PI*5)/8, -(PI*1)/8, 0.0, PI/2, 0.0};
std::vector<double> joint_position_end{0.0, -(PI*5)/8, -(PI*1)/8, 0.0, PI/2, 0.0};
*/

std::vector<double> joint_position_start{0.0, -(PI*2)/8, (PI*6)/8, 0.0, PI/2, 0.0};
std::vector<double> joint_position_throw{0.0, -(PI*3)/8, (PI*4)/8, -(PI*1)/8, PI/2, 0.0};
std::vector<double> joint_position_end{0.0, -(PI*3)/8, (PI*4)/8, -(PI*2)/8, PI/2, 0.0};

double getJointOneAngle(double x, double y) {
    double distToPoint = sqrt(pow(x, 2) + pow(y, 2));
    double theta1 = asin(offset/distToPoint);
    double theta2 = atan2(y, x);
    return theta2 - theta1;
}


double get_throwing_velocity(double x0, double y0, double x1, double y1, double theta) {
    double h = y1 - y0;
    double l = x1 - x0;
    double g = -9.82;
    return -(sqrt(2) * l * sqrt(g * cos(theta) * (h * cos(theta) - l * sin(theta)))) / (2 * (h * pow(cos(theta), 2) - l * cos(theta) * sin(theta)));
}


void goToJointPosition(std::vector<double> joint_goal) {
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setJointValueTarget(joint_goal);
    move_group.setNumPlanningAttempts(10);
    move_group.plan(plan);
    move_group.execute(plan);
}


std::vector<double> vectorizeThrowingVelocity(double velocity, double angle) {
    std::vector<double> result{
        cos(angle)*velocity,
        0.0,
        sin(angle)*velocity
    };
    return result;
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


moveit_msgs::RobotTrajectory addToATrajectory(moveit_msgs::RobotTrajectory trajectory, std::vector<double> thetaStart, std::vector<double> thetaEnd, std::vector<double> thetaDotStart, std::vector<double> thetaDotEnd, double travelTime, int steps, bool isFirstStep, double startTime) {
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    std::vector<std::string> jointNames = move_group.getJointNames();

    for (int i = 0; i < jointNames.size(); i++) {
        ROS_INFO("%s", jointNames[i].c_str());
    }
    ROS_INFO("joint size: %i", jointNames.size());

    trajectory.joint_trajectory.joint_names = jointNames;

    double a0j1 = thetaStart[0];
    double a0j2 = thetaStart[1];
    double a0j3 = thetaStart[2];
    ROS_INFO("a0j3 is this: %f", a0j3);
    double a0j4 = thetaStart[3];
    double a0j5 = thetaStart[4];
    double a0j6 = thetaStart[5];


    double a1j1 = thetaDotStart[0];
    double a1j2 = thetaDotStart[1];
    double a1j3 = thetaDotStart[2];
    ROS_INFO("a1j3 is this: %f", a1j3);
    double a1j4 = thetaDotStart[3];
    double a1j5 = thetaDotStart[4];
    double a1j6 = thetaDotStart[5];


    double a2j1 = (3/pow(travelTime, 2)) * (thetaEnd[0] - thetaStart[0]) - (2/travelTime) * thetaDotStart[0] - (1/travelTime) * thetaDotEnd[0];
    double a2j2 = (3/pow(travelTime, 2)) * (thetaEnd[1] - thetaStart[1]) - (2/travelTime) * thetaDotStart[1] - (1/travelTime) * thetaDotEnd[1];
    double a2j3 = (3/pow(travelTime, 2)) * (thetaEnd[2] - thetaStart[2]) - (2/travelTime) * thetaDotStart[2] - (1/travelTime) * thetaDotEnd[2];
    ROS_INFO("a2j3 is this: %f", a2j3);
    double a2j4 = (3/pow(travelTime, 2)) * (thetaEnd[3] - thetaStart[3]) - (2/travelTime) * thetaDotStart[3] - (1/travelTime) * thetaDotEnd[3];
    double a2j5 = (3/pow(travelTime, 2)) * (thetaEnd[4] - thetaStart[4]) - (2/travelTime) * thetaDotStart[4] - (1/travelTime) * thetaDotEnd[4];
    double a2j6 = (3/pow(travelTime, 2)) * (thetaEnd[5] - thetaStart[5]) - (2/travelTime) * thetaDotStart[5] - (1/travelTime) * thetaDotEnd[5];


    double a3j1 = -(2/pow(travelTime, 3)) * (thetaEnd[0] - thetaStart[0]) + (1/pow(travelTime, 2)) * (thetaDotEnd[0] + thetaDotStart[0]);
    double a3j2 = -(2/pow(travelTime, 3)) * (thetaEnd[1] - thetaStart[1]) + (1/pow(travelTime, 2)) * (thetaDotEnd[1] + thetaDotStart[1]);
    double a3j3 = -(2/pow(travelTime, 3)) * (thetaEnd[2] - thetaStart[2]) + (1/pow(travelTime, 2)) * (thetaDotEnd[2] + thetaDotStart[2]);
    ROS_INFO("a3j3 is this: %f", a3j3);
    double a3j4 = -(2/pow(travelTime, 3)) * (thetaEnd[3] - thetaStart[3]) + (1/pow(travelTime, 2)) * (thetaDotEnd[3] + thetaDotStart[3]);
    double a3j5 = -(2/pow(travelTime, 3)) * (thetaEnd[4] - thetaStart[4]) + (1/pow(travelTime, 2)) * (thetaDotEnd[4] + thetaDotStart[4]);
    double a3j6 = -(2/pow(travelTime, 3)) * (thetaEnd[5] - thetaStart[5]) + (1/pow(travelTime, 2)) * (thetaDotEnd[5] + thetaDotStart[5]);


    ROS_INFO("a0j2: %f", a0j2);
    ROS_INFO("a1j2: %f", a1j2);
    ROS_INFO("a2j2: %f", a2j2);
    ROS_INFO("a3j2: %f", a3j2);

    int k = 0;
    if (!isFirstStep) {
        k = 1;
    }

    for (int i = k; i <= steps; i++) {
        ROS_INFO("size: %i", trajectory.joint_trajectory.points.size());
        trajectory_msgs::JointTrajectoryPoint point;

        double time = (travelTime/steps)*i;

        std::vector<double> pos;
        pos.push_back(a0j1 + a1j1*time + a2j1*pow(time, 2) + a3j1*pow(time, 3));
        pos.push_back(a0j2 + a1j2*time + a2j2*pow(time, 2) + a3j2*pow(time, 3));
        pos.push_back(a0j3 + a1j3*time + a2j3*pow(time, 2) + a3j3*pow(time, 3));
        pos.push_back(a0j4 + a1j4*time + a2j4*pow(time, 2) + a3j4*pow(time, 3));
        pos.push_back(a0j5 + a1j5*time + a2j5*pow(time, 2) + a3j5*pow(time, 3));
        pos.push_back(a0j6 + a1j6*time + a2j6*pow(time, 2) + a3j6*pow(time, 3));

        std::vector<double> vel;
        vel.push_back(a1j1 + 2*a2j1*time + 3*a3j1*pow(time, 2));
        vel.push_back(a1j2 + 2*a2j2*time + 3*a3j2*pow(time, 2));
        vel.push_back(a1j3 + 2*a2j3*time + 3*a3j3*pow(time, 2));
        vel.push_back(a1j4 + 2*a2j4*time + 3*a3j4*pow(time, 2));
        vel.push_back(a1j5 + 2*a2j5*time + 3*a3j5*pow(time, 2));
        vel.push_back(a1j6 + 2*a2j6*time + 3*a3j6*pow(time, 2));

        std::vector<double> acc;
        acc.push_back(2*a2j1 + 6*a3j1*time);
        acc.push_back(2*a2j2 + 6*a3j2*time);
        acc.push_back(2*a2j3 + 6*a3j3*time);
        acc.push_back(2*a2j4 + 6*a3j4*time);
        acc.push_back(2*a2j5 + 6*a3j5*time);
        acc.push_back(2*a2j6 + 6*a3j6*time);
        



        point.positions = pos;
        point.velocities = vel;
        point.accelerations = acc;

        ROS_INFO("Time at this step is: %f", time + startTime);
        ros::Duration seconds(time + startTime);
        point.time_from_start = seconds;

        //point.positions = trajectory.joint_trajectory.points[i].positions;
        //point.velocities = trajectory.joint_trajectory.points[i].velocities;
        //point.accelerations = trajectory.joint_trajectory.points[i].accelerations;
        //point.effort = trajectory.joint_trajectory.points[i].effort;
        //point.time_from_start = trajectory.joint_trajectory.points[i].time_from_start;

        trajectory.joint_trajectory.points.push_back(point);
    }

    return trajectory;
}


moveit_msgs::RobotTrajectory scaleTrajectory(moveit_msgs::RobotTrajectory trajectory, double velocity) {
    for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++) {
        for (int j = 0; j < 6; j++) {
            trajectory.joint_trajectory.points[i].positions[j].
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "newton_throw");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Hello Newton");
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    goToJointPosition(joint_position_start);

    std::vector<double> throwingVelocityVector = vectorizeThrowingVelocity(throwingVelocity, throwingAngle);
    throwingVelocityVector.push_back(0.0);
    throwingVelocityVector.push_back(-4.33*throwingVelocity);
    throwingVelocityVector.push_back(0.0);
    std::vector<double> jointVelocitiesVector = getJointVelocities(throwingVelocityVector, joint_position_throw);



    moveit_msgs::RobotTrajectory trajectory;

    double time = 1.3/throwingVelocity;
    std::vector<double> ZeroVelocityVector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    trajectory = addToATrajectory(trajectory, joint_position_start, joint_position_throw, ZeroVelocityVector, jointVelocitiesVector, time, 50, true, 0.0);
    trajectory = addToATrajectory(trajectory, joint_position_throw, joint_position_end, jointVelocitiesVector, ZeroVelocityVector, 3.0/throwingVelocity, 50, false, time);




    ROS_INFO("x: %f", throwingVelocityVector[0]);
    ROS_INFO("y: %f", throwingVelocityVector[1]);
    ROS_INFO("z: %f", throwingVelocityVector[2]);

    ROS_INFO("J1: %f", jointVelocitiesVector[0]);
    ROS_INFO("J2: %f", jointVelocitiesVector[1]);
    ROS_INFO("J3: %f", jointVelocitiesVector[2]);
    ROS_INFO("J4: %f", jointVelocitiesVector[3]);
    ROS_INFO("J5: %f", jointVelocitiesVector[4]);
    ROS_INFO("J6: %f", jointVelocitiesVector[5]);
    /*
    goToJointPosition(joint_position_start);
    goToJointPosition(joint_position_throw);
    goToJointPosition(joint_position_end);
    goToJointPosition(joint_position_start);
    */

    move_group.execute(trajectory);

    double test_vel = get_throwing_velocity(0.0, 0.0, 2.0, 0.0, PI/4);
    ROS_INFO("test_vel: %f", test_vel);
}