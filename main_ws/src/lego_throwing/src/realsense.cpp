//
// Created by magnus on 10/26/20.
//

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <cv.hpp>
#include <zbar.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

struct Object : public cv::_InputArray {
    std::string type;
    std::string data;
    std::vector<cv::Point> location;
    cv::Point center;
};

struct Frame {
    rs2::frameset frameset;
    rs2::frame colorFrame;
    rs2::frame depthFrame;
    uint height{};
    uint width{};
    cv::Mat matImage;
    uint32_t count = 0;
};

ros::Publisher yeetLocationPublisher;

// Get a frame from realsense
void retrieveFrame(const rs2::pipeline &pipe, Frame *frame) {
    frame->frameset = pipe.wait_for_frames();
    frame->colorFrame = frame->frameset.get_color_frame();
    frame->depthFrame = frame->frameset.get_depth_frame();
    frame->width = frame->colorFrame.as<rs2::video_frame>().get_width();
    frame->height = frame->colorFrame.as<rs2::video_frame>().get_height();
    frame->matImage = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC3, (void *) frame->colorFrame.get_data(),
                              cv::Mat::AUTO_STEP);

    frame->count++;
}

// Find and decode barcodes and QR codes
void decode(cv::Mat &im, std::vector<Object> &decodedObjects) {

    // Create zbar scanner
    zbar::ImageScanner scanner;

    // Configure scanner
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Convert image to grayscale
    cv::Mat imGray;
    cvtColor(im, imGray, CV_BGR2GRAY);

    // Wrap image data in a zbar image
    zbar::Image image(im.cols, im.rows, "Y800", (uchar *) imGray.data, im.cols * im.rows);

    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);

    // Print results
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        Object obj;
        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();

        // Print type and data

        // Obtain location
        for (int i = 0; i < symbol->get_location_size(); i++) {
            obj.location.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
        }

        // calculate center coords
        int sumX = 0, sumY = 0;
        for (auto &i : obj.location) {
            sumX += i.x;
            sumY += i.y;
        }
        obj.center.x = sumX / obj.location.size();
        obj.center.y = sumY / obj.location.size();

        // Check if QR code has already been scanned, if yes then update location and center, if not then update vector with new QR code
        for (auto &decodedObject : decodedObjects) {
            if (decodedObject.data == obj.data) {
                decodedObject.location = obj.location;
                decodedObject.center = obj.center;
                return;
            }
        }
        decodedObjects.push_back(obj);
    }
}


void doHomography(const std::vector<Object> objects, cv::Mat colorImage) {

    cv::Mat homographyImage;
    int counter = 0;


    // Four corners of the plane in of the real world
    std::vector<cv::Point2f> pts_src(4);

    /*
    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "00") {
            pts_src.at(counter) = cv::Point2f(30, 35);
            counter++;
        }
        if (objects[i].data == "01") {
            pts_src.at(counter) = cv::Point2f(30, 185);
            counter++;
        }
        if (objects[i].data == "02") {
            pts_src.at(counter) = cv::Point2f(265, 35);
            counter++;
        }
        if (objects[i].data == "03") {
            pts_src.at(counter) = cv::Point2f(265, 185);
            counter++;
        }
    }
*/

    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "00") {
            pts_src.at(counter) = cv::Point2f(15, 11);
            counter++;
        }
        if (objects[i].data == "01") {
            pts_src.at(counter) = cv::Point2f(87, 11);
            counter++;
        }
        if (objects[i].data == "02") {
            pts_src.at(counter) = cv::Point2f(15, 81);
            counter++;
        }
        if (objects[i].data == "03") {
            pts_src.at(counter) = cv::Point2f(87, 81);
            counter++;
        }
    }

    if (counter != 4)
        return;

// Output image
    cv::Mat hImage;

    std::vector<cv::Point2f> QrCamCoordinates(4);
    counter = 0;
    for (int i = 0; i < objects.size(); i++) {
        // std::cout << "Center for: " << objects[i].data << " " << objects[i].center << std::endl;
        if (objects[i].data != "Yeeeet") {
            QrCamCoordinates[counter] = objects[i].center;
            counter++;
        }
    }
//calculate Homography
    cv::Mat hMatrix = findHomography(QrCamCoordinates, pts_src);

//Warp source image to destination based on homography
    warpPerspective(colorImage, hImage, hMatrix, colorImage.size());
    cv::namedWindow("homography", cv::WINDOW_FULLSCREEN);
    cv::resizeWindow("homography", 1000, 1000);
    cv::imshow("homography", hImage);


    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "Yeeeet") {
            printf("Yeeet coordinates: %d, %d\n", objects[i].center.x, objects[i].center.y);

            std::vector<cv::Point2f> yeetPoints(2);
            yeetPoints[0] = objects[i].center;

            perspectiveTransform(yeetPoints, yeetPoints, hMatrix);
            printf("Yeeet prime coordinates: %f, %f\n", yeetPoints[0].x, yeetPoints[0].y);

            geometry_msgs::Vector3 yeetMsg;
            yeetMsg.x = static_cast<float>(((yeetPoints[0].x - 42))/ 100);
            yeetMsg.y = static_cast<float>(((yeetPoints[0].y + 99))/ 100);
            yeetMsg.z = 0.045;
            yeetLocationPublisher.publish(yeetMsg);
            ros::shutdown();
        }
    }

}

int main(int argc, char *argv[]) {
    // -- REALSENSE SETUP --
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();
    // define frame handle
    Frame frame;
    // QR contents

    // --- ROS STUFF
    ros::init(argc, argv, "realsenseVision");
    ros::NodeHandle node_handle;
    yeetLocationPublisher = node_handle.advertise<geometry_msgs::Vector3>("box_position", 1000);

    while (ros::ok()) {
        std::vector<Object> decodedObjects;

        retrieveFrame(pipe, &frame);
        // Find the QR codes
        decode(frame.matImage, decodedObjects);

        if (!decodedObjects.empty())

            if (cv::waitKey(25) == 27) {
                for (const auto &qr : decodedObjects)
                    printf("QR code data: %s\n", qr.data.c_str());
                break;
            }

        cvtColor(frame.matImage, frame.matImage, CV_BGR2RGB);
        cv::imshow("Image", frame.matImage);

        if (decodedObjects.size() > 3)
            doHomography(decodedObjects, frame.matImage);
        else {
            printf("Decoded objects: %lu\n", decodedObjects.size());

        }
    }
    return 0;
}
