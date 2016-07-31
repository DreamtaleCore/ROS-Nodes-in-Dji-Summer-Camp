/** @file  Find the color apperent object
 * @author DreamTale
 * @date   Jul 30, 2016
  */

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
// Include ros and data transport headers
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
// Include image transport & bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// +------------+--------+--------+
// | color sort | posi_x | posi_y |
// +------------+--------+--------+


ros::Publisher pubMarkerPosi;

void callBackDollTrack(const sensor_msgs::ImageConstPtr& msg)
{
    //Matrix to store each frame of the webcam feed
    Mat src;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    src = cvPtr->image;


    std_msgs::Float32MultiArray sendData;
    sendData.data.push_back();
    pubMarkerPosi.publish(sendData);

    imshow(windowName, src);
    waitKey(10);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "find_doll");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImg;

    pubMarkerPosi = nh.advertise<std_msgs::Float32MultiArray>("/uav_vision/findMarker", 1000);

    subImg = it.subscribe("/uav_cam/image", 5, callBackDollTrack);

    ros::spin();

    return 0;
}
