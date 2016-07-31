/** @file UAV core logic node
  * @author DreamTale
  * @date Jul 30, 2016
  */
#include "generalHeader.h"
#include "recvInfo.h"

#include <opencv2/opencv.hpp>

ros::Publisher pubServo;
ros::Publisher pubOnboard;

ros::Subscriber subGuidance;
ros::Subscriber subGroundCar;
ros::Subscriber subUavVision;

void callBackGroundCar(const std_msgs::String::ConstPtr& data)
{

}

void callBackGuidance(const std_msgs::Float64MultiArray::ConstPtr& data)
{

}

void callBackUavVision(const std_msgs::Float64MultiArray::ConstPtr& data)
{

}

void uavSetup()
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uavCoreLogic");
    ros::NodeHandle nhPub, nhSub;

    pubServo   = nhPub.advertise<std_msgs::String>("uavCoreLogicServo",   1000);
    pubOnboard = nhPub.advertise<std_msgs::String>("uavCoreLogicOnboard", 1000);

    subGroundCar = nhSub.subscribe("uavGroundCar", 1000, callBackGroundCar);
    subUavVision = nhSub.subscribe("uavUavVision", 1000, callBackUavVision);
    subGuidance  = nhSub.subscribe("uavGuidance",  1000, callBackGuidance );

    while (nhPub.ok() && nhSub.ok())
    {
        ros::spinOnce();

//        stringstream ss;
//        int ang1, ang2;
//        cout << "Input:  ";
//        cin >> ang1 >> ang2;
//        ss << ang1 << "," << ang2;
//        std_msgs::String outStr;
//        outStr.data = ss.str();
//        pubServo.publish(outStr);
//        if(ang1 == -1)
//            break;

    }

    return 0;
}
