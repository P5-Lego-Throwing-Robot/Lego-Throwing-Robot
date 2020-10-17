
#include "ros/ros.h"
#include "std_msgs/String.h"


#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>


 double calculateCurrentDistance(std::vector<geometry_msgs::PoseStamped> poseStampVec);

 
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "EndEffector_Velocity");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
     
    geometry_msgs::PoseStamped eePose;
    
   
    ros::Publisher EE_Vel = n.advertise<geometry_msgs::PoseStamped>("EE_Vel", 1000);
   
    ros::Rate loop_rate(10);

    std::vector<geometry_msgs::PoseStamped> poseStampVec;
     


    
    while (ros::ok())
    {
      eePose = group.getCurrentPose("ee_link");

      poseStampVec.push_back(eePose);


      if(poseStampVec.size() < 2){

        continue;
      }


      double velocity = calculateCurrentDistance(poseStampVec);



      EE_Vel.publish(eePose);

      poseStampVec.erase(poseStampVec.begin());
      
      loop_rate.sleep();  
    }

    return 0;
    }





    double calculateCurrentDistance(std::vector<geometry_msgs::PoseStamped> poseStampVec){

      double xSqr, ySqr, zSqr;

      // Calculate distance
      xSqr = (poseStampVec[0].pose.position.x - poseStampVec[1].pose.position.x) * (poseStampVec[0].pose.position.x - poseStampVec[1].pose.position.x);
      ySqr = (poseStampVec[0].pose.position.y - poseStampVec[1].pose.position.y) * (poseStampVec[0].pose.position.y - poseStampVec[1].pose.position.y);
      zSqr = (poseStampVec[0].pose.position.z - poseStampVec[1].pose.position.z) * (poseStampVec[0].pose.position.z - poseStampVec[1].pose.position.z);

      double mySqr = xSqr + ySqr + zSqr;

      double myDistance = sqrt(mySqr);

      double time_stamp = poseStampVec[1].header.stamp.toSec() - poseStampVec[0].header.stamp.toSec();

      double CurrentVel = myDistance / time_stamp;

      ROS_INFO("dist, time-stamp, val: %f, %f, %f", myDistance, time_stamp, CurrentVel);

      return CurrentVel;
    }
