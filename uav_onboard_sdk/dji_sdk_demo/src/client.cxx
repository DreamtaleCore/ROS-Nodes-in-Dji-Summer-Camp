#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <vector>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace std;
using namespace DJI::onboardSDK;

// The sort of uav control signals
enum uavCtrlSignal
{
    uavTakeOff = 0,
    uavLanding,
    uavHangSlient,
    uavGoFlightHeigh,
    uavGoMissionHeigh,
    uavMovVelocity,
    uavForceMannul,
    uavGetCtrlAbility,
    uavControlSum
};

// Define a global DIJ drone
DJIDrone* drone;
const float missioHeight = 1.700;
const float flightHeight = 0.800;

// void coreLogicFunc(vector<int> &_cmd)
void callBackMarkerCtrl(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    int cmd = msg->data[0];
    double correntX, correntY, correntZ;
    switch (cmd) {
    case uavGetCtrlAbility:
        drone->request_sdk_permission_control();
        break;

    case uavMovVelocity:
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 (float)msg->data[1] / 100,                 // x
                                 (float)msg->data[1] / 100,                 // y
                                 (float)msg->data[1] / 100,                 // z
                                 (float)msg->data[1] / 100);                // yaw
        usleep(20000);
        break;

    case uavTakeOff:
        drone->takeoff();   // fly to 1.2m

        break;
    case uavGoFlightHeigh:



        break;
    case uavGoMissionHeigh:

        break;
    case uavLanding:

        break;
    case uavForceMannul:
        // If wanna switch to mannual control, release the control ability
        drone->release_sdk_permission_control();

        break;

    case uavHangSlient:     // Default status is hang slient
    default:
        break;
    }
}

//void callBackMarkerCtrl(const std_msgs::Float32MultiArray::ConstPtr& msg)
//{
//    vector<int> cmd;
//    if(msg->data[3] > 50.0)
//    {
//        cmd.push_back(uavMovLeft);
//        coreLogicFunc(cmd);
//    }
//    if(msg->data[3] < -50.0)
//    {
//        cmd.push_back(uavMovRight);
//        coreLogicFunc(cmd);
//    }
//    if(msg->data[4] > 50.0)
//    {
//        cmd.push_back(uavMovBack);
//        coreLogicFunc(cmd);
//    }
//    if(msg->data[4] < -50.0)
//    {
//        cmd.push_back(uavMovFront);
//        coreLogicFunc(cmd);
//    }
//}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "onboard_sdk");
    ros::NodeHandle nh;

    drone = new DJIDrone(nh);

    //ros::Subscriber subMarkTrack;
    ros::Subscriber subCoreLogic;

    //subMarkTrack = nh.subscribe("/uav_vision/findMarker", 1000, callBackMarkerCtrl);
    subCoreLogic = nh.subscribe("/uav_ctrl/onboard", 1000, callBackMarkerCtrl);

    ros::spin();

    return 0;
}
