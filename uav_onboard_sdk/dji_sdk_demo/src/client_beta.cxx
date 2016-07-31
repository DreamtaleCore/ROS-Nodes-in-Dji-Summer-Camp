#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
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
    uavMovFront,
    uavMovBack,
    uavMovLeft,
    uavMovRight,
    uavTurnLeft,
    uavTurnRight,
    uavForceMannul,
    uavGetCtrlAbility
};

// Define a global DIJ drone
DJIDrone* drone;
const float missioHeight = 1.700;
const float flightHeight = 0.800;

void coreLogicCallback(const std_msgs::UInt8MultiArray::ConstPtr& _cmd)
{
    int cmd = _cmd[0];
    float correntX, correntY, correntZ;
    switch (cmd) {
    case uavGetCtrlAbility:
        drone->request_sdk_permission_control();
        break;

    case uavMovFront:
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 3, 3, 0, 0 );
        usleep(20000);
        break;

    case uavMovBack:
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 -3, -3, 0, 0);
        usleep(20000);
        break;

    case uavMovLeft:
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 3, -3, 0, 0);
        usleep(20000);

        break;
    case uavMovRight:
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 -3, 3, 0, 0);
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
    case uavTurnLeft:       // Disabled
    case uavTurnRight:
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

    drone = new DJIDrone(nh);

    // Get the control ability first
    drone->check_version();       // Check the onboard sdk verison
    drone->request_sdk_permission_control(); // Get the permission

    ros::Subscriber sub;

    sub = nh.subscribe("core_logic", 1000, coreLogicCallback);

    ros::spin();

    return 0;
}
