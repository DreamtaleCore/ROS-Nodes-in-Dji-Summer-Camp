//include ros libraries
#include <ros/ros.h>
#include "../../include/stdHeaders.h"
#include "../../include/sysHeaders.h"
//#include "stdHeaders.h"

using namespace std;

ros::Publisher publisher;
std_msgs::String pubData;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CvtCommand");
    ros::NodeHandle n;

    // Binding the da
    publisher = n.advertise<std_msgs::String>("CvtCommand", 1000);

    int num = 2;
    while(1)
    {
        stringstream ss;
        ss << "CvtCommand: " << num;
        pubData.data = ss.str();
        publisher.publish(pubData);
        num += 2;

        cout << "Here: " << ss.str() << endl;

        usleep(500000);

    }

    return 0;
}
