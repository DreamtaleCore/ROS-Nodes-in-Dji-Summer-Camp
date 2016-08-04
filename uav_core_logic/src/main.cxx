/** @file UAV core logic node
  * @author DreamTale
  * @date Jul 30, 2016
  */
#include "generalHeader.h"
#include "recvInfo.h"

#include <opencv2/opencv.hpp>
//#include <uav_vision/DetectInfo.h>

double KP_X = 0.1;
double KP_Y = 0.1;
double KP_Z = 0.1;
double KP_W = 0.1;  // yaw
double KI_X = 0.01;
double KI_Y = 0.01;
double KI_Z = 0.01;
double KI_W = 0.01; // yaw
double KD_X = 0.01;
double KD_Y = 0.01;
double KD_Z = 0.01;
double KD_W = 0.01; // yaw

ros::Publisher pubServo;
ros::Publisher pubOnboard;

enum uavStatusInfo
{
    uavStWait       = 0x001,
    uavStTakeOff    = 0x002,
    uavStReady      = 0x004,
    uavStFlight     = 0x008,
    uavStGripDoll   = 0x010,
    uavStThrowDoll  = 0x020,
    uavStHover      = 0x040
};

// The sort of uav control signals
enum uavCtrlSignal
{
    uavCsTakeOff = 0,
    uavCsLanding,
    uavCsHangSlient,
    uavCsGoFlightHeigh,
    uavCsGoMissionHeigh,
    uavCsMovVelocity,
    uavCsForceMannul,
    uavCsGetCtrlAbility,
    uavCsControlSum
};

volatile int uavStatus = uavStWait;
// +-------+-------+-------+-------+-------+-------+---------+-------+-------+
// | vel_x | vel_y | vel_z | pos_x | pos_y | pos_z | pos_yaw | ult_z | ult_r |
// +-------+-------+-------+-------+-------+-------+---------+-------+-------+
volatile double droneVelPosInfo[7] = {0.0};

void callBackGroundBuffCar(const std_msgs::String::ConstPtr& msg)
{

}

void callBackGuidanceVoInfo(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i = 0; i < msg->data.size(); i++)
    {
        droneVelPosInfo[i] = msg->data[i];

    }
}

void callBackUavVisionDoll(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    // Step1: Wait the drone get ready and mission is grip dolls
    if(uavStatus == (uavStGripDoll | uavStReady) ||
            uavStatus == (uavStTakeOff | uavStReady))
    {
        // Step2 : Track the doll so make the doll at center of view
        int colorSort = msg->data[0];
        int posiX = msg->data[1];
        int posiY = msg->data[2];
        // Method 1: all dolls heaped by color, we grip dolls by the order of color
        // A strong assum: The dolls DO NOT disturbed by drone's wind
        // Step 2.1.1 Have found the aim color
        // Socket like: |r_x|r_y|y_x|y_y|b_x|b_y|
        for(int i = 0; i < 3; i++)
        {
            // The selected color is verfied
            if(msg->data[i] > 0 && msg->data[2 * i - 1] > 0)
            {
                // Already know the position diff between drone and doll
                // Adjust drone by PD
                std_msgs::Float32MultiArray dollCmd;
                dollCmd.data.push_back(uavCsMovVelocity);
//                dollCmd.data.push_back(x_cmd);
//                dollCmd.data.push_back(y_cmd0);
                pubOnboard.publish(dollCmd);



            }
        }
    }
}

void callBackUavVisionMarker(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // Insure the drone is not busy and is flying
    if(uavStatus == (uavStReady | uavStFlight))
    {
        double dimX = msg->data[3];
        double dimY = msg->data[4];

        // Adjust the drone direction by PD control
        // call the function
        uavStatus = uavStFlight;
        // Waiting for cmd realized (low maybe)
        usleep(1000);
        uavStatus = uavStReady | uavStFlight;

        // VVVVVV Below the code is not safeVVVVVVV
        //Method 1: Use P-D to control the odrone
        if(droneVelPosInfo[0] > 0.1 || droneVelPosInfo[0] < - 0.1)
        {
            for(int i = 0; i < 3; i++)
            {
                cout << droneVelPosInfo[i] << endl;
            }
            cout << "*********************" << endl << endl;
        }
        //Control_X = KP_X * (dimX) + KD_X * (-Guidance_velocity);
        //Con
        // Assume We get the velocity
        double ctrlX = KP_X * (dimX) + KD_X * (- droneVelPosInfo[0]);
        double ctrlY = KP_Y * (dimY) + KD_Y * (- droneVelPosInfo[1]);

        std_msgs::Int32MultiArray pdCtrlMsg;

        pdCtrlMsg.data.push_back(uavCsMovVelocity);
        pdCtrlMsg.data.push_back((int)(ctrlX * 100));
        pdCtrlMsg.data.push_back((int)(ctrlY * 100));
        pdCtrlMsg.data.push_back(0);
        pdCtrlMsg.data.push_back(0);
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        pubOnboard.publish(pdCtrlMsg);
    }
}

void callBackOnboard(const std_msgs::String::ConstPtr& msg)
{
    string strInfo;
    strInfo = "Onboard Callback:" + msg->data;
    ROS_INFO(strInfo.c_str());
}

void droneSetup()
{
    // Step 1: Get the permission of UAV control
    std_msgs::Int32MultiArray initCmd;
    initCmd.data.push_back(uavCsGetCtrlAbility);
    int sendTimes = 400;
    uavStatus = uavStWait;
    while(sendTimes--)
    {
        usleep(1000);
        pubOnboard.publish(initCmd);
    }

    // Step 2: the UAV take off
    uavStatus = uavStTakeOff;
    initCmd.data.clear();
    initCmd.data.push_back(uavCsTakeOff);
    sendTimes = 30;
    while(sendTimes--)
    {
        usleep(1000);
        pubOnboard.publish(initCmd);
    }
    sleep(7);           // Wait the drone setup ok

    // Step 3: the UAV is ready, Let's take missions
    uavStatus = uavStTakeOff | uavStReady;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uavCoreLogic");
    ros::NodeHandle nhPub, nhSub;
    ros::Rate loopRate(100);

    ros::param::get("~KP_X", KP_X);
    ros::param::get("~KP_Y", KP_Y);
    ros::param::get("~KP_Z", KP_Z);
    ros::param::get("~KP_W", KP_W);
    ros::param::get("~KI_X", KI_X);
    ros::param::get("~KI_Y", KI_Y);
    ros::param::get("~KI_Z", KI_Z);
    ros::param::get("~KI_W", KI_W);
    ros::param::get("~KD_X", KD_X);
    ros::param::get("~KD_Y", KD_Y);
    ros::param::get("~KD_Z", KD_Z);
    ros::param::get("~KD_W", KD_W);

    cout << KP_X << endl
         << KP_Y << endl
         << KD_X << endl
         << KD_Y << endl;

    ros::Subscriber subGroundCar;
    ros::Subscriber subUavGuidanceVoInfo;
    ros::Subscriber subUavVisionDoll;
    ros::Subscriber subUavVisionMarker;
    ros::Subscriber subUavOnboard;

    pubServo   = nhPub.advertise<std_msgs::String>("/uav_ctrl/servo",   100);
    pubOnboard = nhPub.advertise<std_msgs::Int32MultiArray>("/uav_ctrl/onboard", 100);  

    // Be ready for drone take missions
    droneSetup();

    subGroundCar         = nhSub.subscribe("/uav_parter/buffCar",      100, callBackGroundBuffCar  );
    subUavGuidanceVoInfo = nhSub.subscribe("/uav_guider/velPosByVo",   100, callBackGuidanceVoInfo );
    subUavOnboard        = nhSub.subscribe("/uav_motion/onboard",      100, callBackOnboard        );
    subUavVisionDoll     = nhSub.subscribe("/uav_vision/detectDoll",   100, callBackUavVisionDoll  );
    subUavVisionMarker   = nhSub.subscribe("/uav_vision/detectMarker", 100, callBackUavVisionMarker);

    while (nhPub.ok() && nhSub.ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
