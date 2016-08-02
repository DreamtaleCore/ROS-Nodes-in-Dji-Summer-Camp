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
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
// Include image transport & bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "squareDetect.h"

ros::Publisher pubDollPosi;

// +------------+--------+--------+
// | color sort | posi_x | posi_y |
// +------------+--------+--------+

RotatedRect detectSquare(Mat img)
{
    //split the channel
    vector<Mat> img_split;
    Mat img_single;
    split(img,img_split);
    img_single = img_split[2].clone();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    float maxContourArea = 0;
    int maxContourArea_index;

    Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(img_single,img_single,MORPH_CLOSE, element);
    threshold(img_single, img_single, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);// Use the OTSU algorithm
    imshow("img_single",img_single);
    findContours(img_single, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    // Find the max area of contours
    for (int i=0;i<contours.size();i++)
    {
        int contourSize = contourArea(contours[i]);
        if(contourSize > maxContourArea)
        {
            maxContourArea = contourSize;
            maxContourArea_index = i;
        }
    }
    RotatedRect box = minAreaRect(contours[maxContourArea_index]);
    return box;
}

Mat detectDoll(Mat img, RotatedRect box)
{
    // Set ROI
    Mat mask(img.rows,img.cols,CV_8UC3,Scalar::all(0));
    Point2f vertex[4];
    box.points(vertex);
    vector<vector<Point> > maskContours;
    vector<Point> ptr;
    for(int i=0;i<4;i++)
    {
        ptr.push_back(Point(vertex[i].x,vertex[i].y));
    }
    maskContours.push_back(ptr);
    drawContours(mask,maskContours,0,Scalar::all(255),-1);
    Mat imgROI;
    img.copyTo(imgROI,mask);

    // convert to HSV
    Mat img_hsv_blue, img_hsv_red1,img_hsv_red2,img_hsv_yellow;
    cvtColor(imgROI, img_hsv_blue, CV_BGR2HSV);
    Mat img_threshold_blue, img_threshold_red, img_threshold_red1, img_threshold_red2, img_threshold_yellow;

    Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_OPEN,element,Point(-1,-1),3);
    morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_CLOSE,element,Point(-1,-1),3);

    img_hsv_red1   = img_hsv_blue.clone();
    img_hsv_red2   = img_hsv_blue.clone();
    img_hsv_yellow = img_hsv_blue.clone();

    Mat blue_low(Scalar(60,43,46));
    Mat blue_higher(Scalar(140,255,255));

    Mat red1_low(Scalar(0,43,46));
    Mat red1_higher(Scalar(3,255,255));

    Mat red2_low(Scalar(170,43,46));
    Mat red2_higher(Scalar(180,255,255));

    Mat yellow_low(Scalar(20,43,46));
    Mat yellow_higher(Scalar(34,255,255));

    inRange(img_hsv_blue,   blue_low,   blue_higher,   img_threshold_blue);
    inRange(img_hsv_red1,   red1_low,   red1_higher,   img_threshold_red1);
    inRange(img_hsv_red2,   red2_low,   red2_higher,   img_threshold_red2);
    inRange(img_hsv_yellow, yellow_low, yellow_higher, img_threshold_yellow);
    img_threshold_red = img_threshold_red1 | img_threshold_red2;

    imshow("123",img_threshold_red1);

    vector<vector<Point> > contours_blue, contours_red, contours_yellow;
    vector<Vec4i> hierarchy_blue, hierarchy_red, hierarchy_yellow;
    findContours(img_threshold_blue,   contours_blue,hierarchy_blue,     CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    findContours(img_threshold_red,    contours_red,hierarchy_red,       CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    findContours(img_threshold_yellow, contours_yellow,hierarchy_yellow, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    // Find the max area of contours
    for (int i=0;i<contours_blue.size();i++){
        int contourSize = contourArea(contours_blue[i]);
        if(contourSize>6000){
            Rect box = boundingRect(contours_blue[i]);
            rectangle(img,box,Scalar(255,0,0),5);
        }
    }

    for (int i=0;i<contours_red.size();i++){
        int contourSize = contourArea(contours_red[i]);
        if(contourSize>10000){
            Rect box = boundingRect(contours_red[i]);
            rectangle(img,box,Scalar(0,0,255),5);
        }
    }

    for (int i=0;i<contours_yellow.size();i++){
        int contourSize = contourArea(contours_yellow[i]);
        if(contourSize>10000){
            Rect box = boundingRect(contours_yellow[i]);
            rectangle(img,box,Scalar(0,255,255),5);
        }
    }

    imshow("dst",img);
    return img;
}


void callBackDollTrack(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat src = cvPtr->image;

    imshow("Original Image", src);

    RotatedRect box = detectSquare(src);
    Point2f vertex[4];
    box.points(vertex);
    for(int i=0;i<4;i++)
    {
        line(src,vertex[i],vertex[(i+1)%4],Scalar(255,0,0),5);
    }
    //rectangle(img_src,box,Scalar(0,255,0),5);

    Mat imgROI = detectDoll(src, box);
    //time_start = getTickCount() - time_start;
    //cout << "run time:" << time_start / getTickFrequency() << "s" << endl;
    imshow("1",src);

    waitKey(10);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "find_doll");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImg;

    pubDollPosi = nh.advertise<std_msgs::Int16MultiArray>("/uav_vision/findDoll", 1000);

    subImg = it.subscribe("/uav_cam/image", 5, callBackDollTrack);

    ros::spin();

    return 0;
}
