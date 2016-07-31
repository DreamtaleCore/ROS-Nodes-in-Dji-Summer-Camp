/* @Camera processing nodes
 * @Note: * .xml file to modify the calibration of the location and name
 * @Author: DreamTale
 * @Date: 2014-11-14
 */

#define CAMERA_NUM 0

#include<sstream>
#include<iostream>
#include<stdlib.h>

//include ros libraries
#include<ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"

//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/legacy/compat.hpp>
#include "opencv2/ml/ml.hpp"
#include "opencv2/objdetect/objdetect.hpp"

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/* this file has the coordinate with opencv photo:
 *  (0,0)+------------------------>u
 *       |
 *       |
 *       |
 *       \/v
 *  the number of distence between the circle and UAV
 *   is depend on the hight of uav
 */

using namespace cv;
using namespace std;

//String CalibrationCascadeName = "/home/odroid/ws/src/read_cam/camera.xml";
int i = 0;
double scale = 1.4;

Mat view, rview;
Mat Image, ImageGRAY, mid;

int imgWidth, imgHeight;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_camera_reader");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("uav_cam/image", 3);

    cv_bridge::CvImage outMsg;

    ros::Rate loopRate(30);

    VideoCapture cap(CAMERA_NUM);
    cap.set(CV_CAP_PROP_FRAME_WIDTH ,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT ,480);
    cap.set(CV_CAP_PROP_EXPOSURE ,30);

    Size imageSize;

    if( n.ok() )
    {
        cap>>view;
        imageSize = view.size();

        while (nh.ok())
        {
            cap>>view;
            Image = view;

            imshow("Undistored View", Image);

            outMsg.image = Image;
            outMsg.header.stamp = ros::Time::now();
            outMsg.encoding = "bgr8";

            pub.publish(outMsg.toImageMsg());
            ros::spinOnce();
            loopRate.sleep();

            if(waitKey(1) == 27)
                break;
        }
    }

    return 0;
}
