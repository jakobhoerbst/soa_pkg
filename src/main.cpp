/*
    Maze Solving Algorithm:
    Main Program
    Version 1
    Autor: Hoerbst
    Contributor: Gmeiner

    sources: 
    Quarternion - Euler
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    0 ... DEAD END
    1 ... right 
    2 ... down
    3 ... left 
    4 ... up     

    jakob@ubuntu:~$ roslaunch turtlebot3_bringup turtlebot3_model.launch 
 */

#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "gazebo_msgs/ModelStates.h"
#include <vector>
#include <chrono>
    
// own header                                                   
#include "operations.hpp"
#include "DFS.hpp"
#include "navigation.hpp"
#include "dataRecording.hpp"

using namespace std;

float scanResult[360];          /*!< LiDAR scanning values  */
bool initialLidar = false;      /*!< set to true on first LiDAR feedback*/

float poseGT[3] = {0,0,0};      /*!< Ground Truth Pose - Initial: 0,0,0  */
bool initialGT = false;         /*!< set to true on first GT feedback */

float poseOdom[3] = {0,0,0};    /*!< Odometrie (odom) Pose - Initial: 0,0,0  */
bool initialOdom = false;       /*!< et to true on first odom feedback */

bool positionReached = true;    /*!< Feedback from Navigation  */
float nextPosition[2] = {0,0};  /*!< Next Position from DFS to navigation  */

long currentSeconds = 0;        /*!< Current time in Seconds for time measurement  */
long initialSeconds = 0;        /*!< Initial Seconds for time measurement  */ 


/*! \brief Callback for odometry subscriber
 *  Writing odometry feedback to parameters:
 *  @param poseOdom[3] getting position and orientation from odometry feeback 
 *  @param initialOdom true if initial feedback is received
 */
void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odometry)
{
    poseOdom[0] = odometry -> pose.pose.position.x; 
    poseOdom[1] = odometry -> pose.pose.position.y;

    Quaternion orientationQ; 
    orientationQ.x = odometry -> pose.pose.orientation.x; 
    orientationQ.y = odometry -> pose.pose.orientation.y; 
    orientationQ.z = odometry -> pose.pose.orientation.z; 
    orientationQ.w = odometry -> pose.pose.orientation.w;

    EulerAngles orientationE; 
    orientationE = ToEulerAngles(orientationQ); 
    poseOdom[2] = (orientationE.yaw*180.0)/PI;

    initialOdom = true; 
}

/*! \brief Callback for ground truth subscriber
 *  Writing ground truth feedback to parameters:
 *  @param poseGT[3] getting position and orientation from GT feeback 
 *  @param initialGT true if initial feedback is received
 */
void callbackGT(const gazebo_msgs::ModelStates::ConstPtr& GT)
{
    poseGT[0] = GT->pose[2].position.x;
    poseGT[1] = GT->pose[2].position.y;

    Quaternion orientationQ; 
    orientationQ.x = GT->pose[2].orientation.x; 
    orientationQ.y = GT->pose[2].orientation.y; 
    orientationQ.z = GT->pose[2].orientation.z; 
    orientationQ.w = GT->pose[2].orientation.w;

    EulerAngles orientationE; 
    orientationE = ToEulerAngles(orientationQ); 
    poseGT[2] = (orientationE.yaw*180.0)/PI;

    initialGT = true;
}

/*! \brief Callback for LiDAR subscriber
 *  Writing LiDAR feedback to parameters:
 *  @param scanResult[360] LiDAR feedback (0°-359°)
 *  @param currentSeconds time stamp from topic (for simulated time measurement)
 */
void callbackLiDAR(const sensor_msgs::LaserScan::ConstPtr& LiDAR)
{
    initialLidar = true; 
   // inRange = 0; 
  
    for(int i = 0; i < 360; i++)
        scanResult[359-i] = (LiDAR->ranges[i]);

    currentSeconds = LiDAR->header.stamp.sec; 
}

/*! \brief MAIN
 * Setting up the subscriber and publisher. 
 * Creating objects for the different classes
 * Waiting for initial feedback from the topic
 * Loop for main code of the DFS algorithm and finishes when the alogrithm found exit
 */
int main(int argc, char **argv )
{
    string argument = argv[1];

    cout << " - PROJECT 2 -" << endl << endl;

    if(argument == "GT")
        cout << "using: ground truth " << endl;
    else if(argument == "odom") 
        cout << "using: odom" << endl; 
    else
    {
        cout << "ERROR with input argument" << endl; 
        return 0;     
    }

    // ROS
    ros::init(argc, argv, "DFSnode");
    ros::NodeHandle nh("~"); 
    
    ros::Subscriber subscriberLiDAR;
    ros::Subscriber subscriberGT;
    ros::Subscriber subscriberOdometry; 

    ros::Publisher drivePub;

    subscriberLiDAR     = nh.subscribe("/scan", 100, callbackLiDAR);
    subscriberGT        = nh.subscribe("/gazebo/model_states", 100, callbackGT);
    subscriberOdometry  = nh.subscribe("/odom", 100, callbackOdometry);

    drivePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    // classes for navigation and DFS algorithm
    navigationClass navigation(drivePub);
    DFSClass DFS(scanResult, nextPosition);
    dataRecording CSV(argument);
    
    CSV.start();

    // set pointer to GT or odom
    if(argument == "GT")
    {
        DFS.currentPose = poseGT;
        navigation.currentPose = poseGT; 
    }
    else if(argument == "odom")
    {
        DFS.currentPose = poseOdom;
        navigation.currentPose = poseOdom; 
    }

    // startup and waiting for initial sensor data
    while(ros::ok())
    {
        if(initialLidar && initialGT && initialOdom)
        {
            cout << "STARTUP complete" << endl; 
            initialSeconds = currentSeconds;

            break;             
        }

        ros::spinOnce();
    }
    auto start = std::chrono::high_resolution_clock::now(); 

    // loop solving the maze
    while(ros::ok())
    {
        // if new position is reached calculate the next movement
        if(positionReached){
            positionReached = false;
            if(DFS.handleNode()){
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> durationReal = finish - start;
                CSV.writeSuccess(DFS.getNodeNumber(), (currentSeconds-initialSeconds), durationReal);
                break;
            }
        } 
  
        // navigate to next movement
        if(navigation.moveTo(nextPosition))
            positionReached = true; 
        ros::spinOnce();
    }  
    //system("~jakob/SOA_ws/src/soa_pkg/kill.bash");

return 0; 
}
