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

#include "Object.h"

//########-----------Blue-----------------------+
//H_MIN 	33 	42	42	52	78	78	59	50 	50	|
//H_MAX 	103	118	118	126 126	141	163	151 123	|
//S_MIN 	45	66	66	23 	72	62	51	46	55	|
//S_MAX 	171	204	204	199	204	204	204	204	204	|
//V_MIN 	41	5	5	161	7	0	43	9	37	|
//V_MAX 	256	256	256	231	256	256	256	256	256	|
//########-----------red--------+---------------+
//H_MIN 	0	0	0	0 	0	|
//H_MAX 	0	3	1	1	1	|
//S_MIN 	22	54	50	48	44	|
//S_MAX 	168	143	180	162	167	|
//V_MIN 	0	15	9	39	52	|
//V_MAX 	256	256	256	256	256	|
//########-----------yellow-----+---------------+
//H_MIN 	11	14	16	21	22 	24	23	11	21 	|
//H_MAX 	92	34	51	53	53	61	61	89	40	|
//S_MIN 	102	72	85	87	82	105	69	69	95	|
//S_MAX 	140	107	132	133	114	129	81	92	121	|
//V_MIN 	24	64	114	70	9	9	26	9	9	|
//V_MAX 	251	256	256	256	256	256	256	256	256	|
//----------------------------------------------+

// +------------+--------+--------+
// | color sort | posi_x | posi_y |
// +------------+--------+--------+

//initial min and max HSV filter values.
//these will be changed using trackbars
vector<int> hsvRed, hsvYellow, hsvBlue;
vector<vector<int> > colorSelc;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 20;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "Thresholded Image";

//The following for canny edge detec
Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
string window_name = "Edge Map";

ros::Publisher pubMarkerPosi;

void hsvParamSetup(int mode)
{
    // Method 1: use the max frequency number
    if(mode == 1)
    {
        hsvBlue.push_back(50);
        hsvBlue.push_back(126);
        hsvBlue.push_back(55);
        hsvBlue.push_back(204);
        hsvBlue.push_back(43);
        hsvBlue.push_back(256);

        hsvYellow.push_back(0);
        hsvYellow.push_back(1);
        hsvYellow.push_back(48);
        hsvYellow.push_back(167);
        hsvYellow.push_back(15);
        hsvYellow.push_back(256);

        hsvRed.push_back(16);
        hsvRed.push_back(61);
        hsvRed.push_back(82);
        hsvRed.push_back(121);
        hsvRed.push_back(9);
        hsvRed.push_back(256);
    }
    // Method 2: use the average frequency number
    else
    {
        hsvBlue.push_back(54);
        hsvBlue.push_back(126);
        hsvBlue.push_back(45);
        hsvBlue.push_back(195);
        hsvBlue.push_back(41);
        hsvBlue.push_back(256);

        hsvYellow.push_back(0);
        hsvYellow.push_back(1);
        hsvYellow.push_back(38);
        hsvYellow.push_back(162);
        hsvYellow.push_back(26);
        hsvYellow.push_back(256);

        hsvRed.push_back(18);
        hsvRed.push_back(63);
        hsvRed.push_back(85);
        hsvRed.push_back(121);
        hsvRed.push_back(24);
        hsvRed.push_back(256);
    }
    colorSelc.push_back(hsvBlue);
    colorSelc.push_back(hsvRed);
    colorSelc.push_back(hsvYellow);
}

void drawObject(vector<Object> theObjects,Mat &frame, int colorSort = 0)
{

    Scalar color;
    string colorInfo;
    if (colorSort == 0)             {color = Scalar(255, 0, 0); colorInfo = "Red"   ;}
    else if (colorSort == 1)        {color = Scalar(0, 255, 0); colorInfo = "Yellow";}
    else                            {color = Scalar(0, 0, 255); colorInfo = "Blue"  ;}
    for(int i =0; i < theObjects.size(); i++)
    {
        std_msgs::Int16MultiArray sendData;
        sendData.data.push_back(colorSort);
        sendData.data.push_back(theObjects[i].getXPos());
        sendData.data.push_back(theObjects[i].getYPos());

        pubMarkerPosi.publish(sendData);

        cout << colorInfo << " object at [" << theObjects.at(i).getXPos()
             << ", " << theObjects.at(i).getYPos() << "]" << endl;
        cv::circle(frame,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),10, color);
    }
}

void morphOps(Mat &thresh)
{
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);

    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}

void trackFilteredObject(Mat threshold, Mat &cameraFeed, int colorMode = 0)
{
    vector <Object> objects;
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    bool objectFound = false;
    if (hierarchy.size() > 0)
    {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects < MAX_NUM_OBJECTS)
        {
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {
                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA)
                {
                    Object object;

                    object.setXPos(moment.m10/area);
                    object.setYPos(moment.m01/area);

                    objects.push_back(object);

                    objectFound = true;

                }
                else objectFound = false;
            }
            //let user know you found an object
            if(objectFound ==true)
            {
                //draw object location on screen
                drawObject(objects,cameraFeed, colorMode);
            }
        }
        else
            cout << "TOO MUCH NOISE! ADJUST FILTER" << endl;
    }
}

void callBackDollTrack(const sensor_msgs::ImageConstPtr& msg)
{
    //Matrix to store each frame of the webcam feed
    Mat cameraFeed;
    Mat threshold;
    Mat HSV;

    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    src = cvPtr->image;

    cameraFeed = src;

    //convert frame from BGR to HSV colorspace
    cvtColor(src,HSV,CV_BGR2HSV);

    //if in calibration mode, we track objects based on the HSV slider values.
    cvtColor(cameraFeed,HSV,CV_BGR2HSV);

    for(int i = 0; i < 3; i++)
    {
        inRange(HSV,
                Scalar(colorSelc[i][0],colorSelc[i][2],colorSelc[i][4]),
                Scalar(colorSelc[i][1],colorSelc[i][3],colorSelc[i][5]),
                threshold);
        morphOps(threshold);
        imshow(windowName1,threshold);

        //the folowing for canny edge detec
        /// Create a matrix of the same type and size as src (for dst)
        dst.create( src.size(), src.type() );
        /// Convert the image to grayscale
        cvtColor( src, src_gray, CV_BGR2GRAY );

        trackFilteredObject(threshold, cameraFeed, i);

    }

    imshow(windowName, cameraFeed);
    waitKey(10);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "find_doll");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImg;

    namedWindow(windowName, 1);

    // use the max frequency
    hsvParamSetup(0);

    pubMarkerPosi = nh.advertise<std_msgs::Int16MultiArray>("/uav_vision/findDoll", 1000);

    subImg = it.subscribe("/uav_cam/image", 5, callBackDollTrack);

    ros::spin();

    return 0;
}
