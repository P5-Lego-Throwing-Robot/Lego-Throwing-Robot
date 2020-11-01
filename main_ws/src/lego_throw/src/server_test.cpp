//
// Created by magnus on 11/1/20.
//

#include <ros/ros.h>
#include <lego_throw/camera.h>


bool add(lego_throw::camera::Request  &req,
          lego_throw::camera::Response &res) {

    float answer = req.x + req.y;
    int status = 1;
    res.status = 1;
    ROS_INFO("request: x=%ld, y=%ld", (long int) req.x, (long int) req.y);
    ROS_INFO("sending back response: [%ld]", (long int) res.status);
    return true;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("camera", add);
    ROS_INFO("READY FOR CLIENT");
    ros::spin();


    return 0;
}