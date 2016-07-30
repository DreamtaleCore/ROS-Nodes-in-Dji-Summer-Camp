#include "commCtrl.h"
#include "../include/stdHeaders.h"
#include "ros/ros.h"

//==============================================

// the content of the data packet

// +------------+----------+----------------+------------+
// | packetHead | itemName | objectCategory | packetTail |
// +------------+----------+----------------+------------+

char dev[13] = "/dev/ttyUSB0";
ros::Publisher dataPub;
std_msgs::String pubData;

void chatterCallback(const std_msgs::String::ConstPtr& msgs)
{
    string cmd_data = (char*)msgs->data.c_str();

    ROS_INFO("I'll write [%s]\n", cmd_data.c_str());

    commWrite(cmd_data, dev);

    cmd_data = "";
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serialWrite");
    ros::NodeHandle nh;

    dataPub = nh.advertise<std_msgs::String>("serialWrite", 1000);

    ros::Subscriber sub = nh.subscribe("CvtCommand", 1000, chatterCallback);

    ros::spin();

    return 0;
}
