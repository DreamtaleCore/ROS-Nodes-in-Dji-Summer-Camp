#include "../../include/stdHeaders.h"
#include "ros/ros.h"

char dev[13] = "/dev/ttyUSB0";
ros::Publisher dataPub;
std_msgs::String pubData;

//void chatterCallback(const std_msgs::String::ConstPtr& msgs)
//{
//    string cmd_data = (char*)msgs->data.c_str();
//    cmd_data += '\0';
//    ROS_INFO("I get the data: [%s]\n", cmd_data.c_str());

//    usleep(5000);
//}

void chatterCallback(const std_msgs::Int16MultiArray::ConstPtr &msgs)
{
    ROS_INFO("I get the data: [%d, %d, %d]\n", msgs->data[0], msgs->data[1], msgs->data[2]);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "echoNode");
    ros::NodeHandle nh;

    dataPub = nh.advertise<std_msgs::String>("echoNode", 1000);

    //ros::Subscriber sub = nh.subscribe("serialRead", 1000, chatterCallback);
    ros::Subscriber sub = nh.subscribe("/uav_vision/findDoll", 1000, chatterCallback);

    ros::spin();

    return 0;
}
