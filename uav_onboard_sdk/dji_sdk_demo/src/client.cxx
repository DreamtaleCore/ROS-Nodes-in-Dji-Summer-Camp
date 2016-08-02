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
void callBackLogicalCtrl(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    int cmd = msg->data[0];
    cout << "I got the cmd: " << cmd << endl;
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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "onboard_sdk");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    drone = new DJIDrone(nh);

    //ros::Subscriber subMarkTrack;
    ros::Subscriber subCoreLogic;

    //subMarkTrack = nh.subscribe("/uav_vision/findMarker", 100, callBackMarkerCtrl);
    subCoreLogic = nh.subscribe("/uav_ctrl/onboard", 100, callBackLogicalCtrl);

    while(nh.ok())
    {
        ros::spinOnce();
        loopRate.sleep();

    }

    return 0;
}
