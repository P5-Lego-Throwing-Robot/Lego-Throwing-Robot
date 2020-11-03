
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <cv.hpp>
#include <zbar.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include "lego_throw/camera.h"


struct Object : public cv::_InputArray {
    std::string data;
    std::vector<cv::Point2f> location;
    cv::Point2f center;
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

// Add names for QR codes here if you add more QR codes to scene
const std::vector<std::string> qrCustomNames = {
        "ID_1",
        "ID_2",
        "ID_3",
        "ID_4",
        "ID_5"
};
std::vector<Object> customQRDetected; // Initially we can only detect 12 custom objects or else we crash

// service client
ros::ServiceClient client;


// Get a frame from realsense
void retrieveFrame(const rs2::pipeline &pipe, Frame *frame) {
    frame->frameset = pipe.wait_for_frames();
    frame->colorFrame = frame->frameset.get_color_frame();
    //frame->depthFrame = frame->frameset.get_depth_frame(); // We do not need depth frame for anything.. yet
    frame->width = frame->colorFrame.as<rs2::video_frame>().get_width();
    frame->height = frame->colorFrame.as<rs2::video_frame>().get_height();
    frame->matImage = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC3, (void *) frame->colorFrame.get_data(),
                            node_handle  cv::Mat::AUTO_STEP);

    // Increment frame number
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
        obj.data = symbol->get_data();

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

        decodedObjects.push_back(obj);
    }
}


void doHomography(const std::vector<Object> objects, cv::Mat colorImage) {
    // Four corners of the plane in of the real world is added to each QR code
    std::vector<cv::Point2f> cornersForPlane(4);
    // Put QR positions into a new vector of cv::Point2f, this new type is needed for findHomography(...);
    // Corner QR codes should be filtered from custom qr codes which is why we loop through them.
    std::vector<cv::Point2f> QrCamCoordinates(4);
    int amountQRCornersFound = 0;

    /*
    //ROBOT TABLE POINTS
    for (int i = 0; i < objects.size(); ++i) {
        switch (objects[i].data) {
            case "00":
                cornersForPlane[amountQRCornersFound] = cv::Point2f(0, 0);
                QrCamCoordinates[amountQRCornersFound] = objects[i].center;
                amountQRCornersFound++;
                break;

            case "01":
                cornersForPlane[amountQRCornersFound] = cv::Point2f(71, 0);
                QrCamCoordinates[amountQRCornersFound] = objects[i].center;
                amountQRCornersFound++;
                break;
            case "02":
                cornersForPlane[amountQRCornersFound] = cv::Point2f(0, 74);
                QrCamCoordinates[amountQRCornersFound] = objects[i].center;
                amountQRCornersFound++;
                break;
            case "03":
                cornersForPlane[amountQRCornersFound] = cv::Point2f(74, 71);
                QrCamCoordinates[amountQRCornersFound] = objects[i].center;
                amountQRCornersFound++;
                break;
            default:


        }

    } */

    // A4 PAPER TEST POINTS IN mm
    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "00") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(0, 0);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "01") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(145, 0);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "02") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(0, 235);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "03") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(145, 235);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
    }

    // If we didn't find 4 QR corners then stop executing and return to main loop
    if (amountQRCornersFound != 4)
        return;

    //calculate Homography matrix from 4 corners position with offsets
    cv::Mat hMatrix = findHomography(QrCamCoordinates, cornersForPlane);

    // Output image
    cv::Mat hImage;
    //Warp source image to destination based on homography
    warpPerspective(colorImage, hImage, hMatrix, cv::Size(500, 500));
    //cv::namedWindow("homography", cv::WINDOW_FULLSCREEN);
    //cv::imshow("homography", hImage);


    // Check if packaging QR codes have been found
    for (int i = 0; i < objects.size(); ++i) {
        std::vector<std::string>::const_iterator it;
        it = std::find(std::begin(qrCustomNames), std::end(qrCustomNames), objects[i].data);

        if (it != std::end(qrCustomNames)) {
            // Check if the packaging QR code already exists
            for (int j = 0; j < customQRDetected.size(); ++j) {
                if (objects[i].data == customQRDetected[j].data) // It exists
                    return;
            }
            // If we reach this far we have a new object push this into a vector. This vector is global
            customQRDetected.push_back(objects[i]);

            // Prepare data for perspectiveTransform
            std::vector<cv::Point2f> transposedPoint {objects[i].center};
            // Multiply point with homography matrix to transform to plane. Funtion input is cv::InputArray or a std::vector
            perspectiveTransform(transposedPoint, transposedPoint, hMatrix);

            // Measured offset from robot
            float xRobotOffset = 28; //cm
            float yRobotOffset = 73; // cm

            auto xWithOffsetInMeters = static_cast<float>(((transposedPoint[0].x - xRobotOffset)) / 100);
            auto yWithOffsetInMeters = static_cast<float>(((transposedPoint[0].y + yRobotOffset)) / 100);

            lego_throw::camera camSrv;

            camSrv.request.x = xWithOffsetInMeters;
            camSrv.request.y = yWithOffsetInMeters;
            camSrv.request.z = 0.045;
            camSrv.request.data = *it;

            printf("%s: x =  %f, y = %f\n", objects[i].data.c_str(), camSrv.request.x, camSrv.request.y);

            if (client.call(camSrv)) printf("Response status: %i\n", static_cast<int>(camSrv.response.status));


        }
    }

}

int main(int argc, char *argv[]) {
    // --- ROS STUFF
    ros::init(argc, argv, "realsenseVision");
    ros::NodeHandle nodeHandle;
    client = nodeHandle.serviceClient<lego_throw::camera>("camera");
    //client.waitForExistence();

    // -- REALSENSE SETUP --
    rs2::pipeline pipe;                     // Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe.start();                           // Start streaming with default recommended configuration
    Frame frame;                            // Create frame handle

    printf("Start filming the scene\n");
    while (ros::ok()){
        std::vector<Object> decodedObjects;

        retrieveFrame(pipe, &frame);                               // Retrieve a set of frames from Realsense camera
        decode(frame.matImage, decodedObjects);              // Find the QR codes and store them in vector
        cvtColor(frame.matImage, frame.matImage, CV_BGR2RGB); // Convert image to RGB to display correct channels to cv::imshow
        cv::imshow("Image", frame.matImage);

        if (decodedObjects.size() > 3)                              // Dont bother checking for corners unless we have 4 or more corners
            doHomography(decodedObjects, frame.matImage);           // Check for 4 corners and a throwing target

        if (cv::waitKey(25) == 27) break;
    }
    return 0;
}

