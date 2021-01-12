/**
 * 
 * \mainpage
 * MMR3 SOAR Project 2 - Maze Solving Algorithm
 * 
 * @section Description
 * This is a Code documentation to the Paper "Maze Solving Algorithm" by Gmeiner Marc and Hoerbst Jakob. \n
 * All necessary instructions to run the code are listed here. \n
 * Author: Hoerbst Jakob \n
 * Contributor: Gmeiner Marc
 * 
 * @section System-Recommendations
 * This code is tested with two different systems: \n
 * 1. System:
 * - Ubuntu 21.04 LTS
 * - ROS Noetic
 * - 8192 MB RAM
 * 2. System:
 * - OS
 * - ROS Version
 * - RAM
 * 
 * 
 * @section Installation
 * 1. Go to your catkin workspace and download the repository  
 * `$ cd /PathTo/catkin_ws/src`  
 * `$ git clone https://github.com/jakobhoerbst/soa_pkg/`
 * 
 * 2. Install Package via catkin_make  
 * `$ cd catkin_ws` or `$ cd ..`  
 * `$ catkin_make`
 * 
 * @section Starting
 * With `testingSequence.sh` (instead of `start.sh`) the program will be executed 100 times (`start.sh` = 1 Run). \n
 * In line 32-33 it must be selected if odometry or ground truth is used for position gathering. \n
 * Ground Truth is pre-selected. \n
 * 
 * 
 * `$ cd /PathTo/catkin_ws/soa_pkg`  
 * `$ bash start.sh`
 * 
 * @section Legend
 * 
 * Meaning of the numbers in the console: \n
 * 0 ... DEAD END \n
 * 1 ... right \n
 * 2 ... down \n
 * 3 ... left \n
 * 4 ... up 
 * 
 */


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
bool initialLidar = false;      /*!< ...  */
float poseGT[3] = {0,0,0};      /*!< Ground Truth Position - Initial: 0,0,0  */
bool initialGT = false;         /*!< ...  */
float poseOdom[3] = {0,0,0};    /*!< Odometrie (odom) Position - Initial: 0,0,0  */
bool initialOdom = false;       /*!< ...  */
bool positionReached = true;    /*!< Feedback from Navigation  */
float nextPosition[2] = {0,0};  /*!< Next Position from DFS to navigation  */

long currentSeconds = 0;        /*!< Current time in Seconds for time measurement  */
long initialSeconds = 0;        /*!< Initial Seconds for time measurement  */ 


/*! \brief Odometry Ros Subscriber
 *
 *  Detailed description starts here.
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

/*! \brief Ground Truth Ros Subscriber
 *
 *  Detailed description starts here.
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

/*! \brief LiDAR Ros Subscriber
 *
 *  Detailed description starts here.
 */
void callbackLiDAR(const sensor_msgs::LaserScan::ConstPtr& LiDAR)
{
    initialLidar = true; 
   // inRange = 0; 
  
    for(int i = 0; i < 360; i++)
        scanResult[359-i] = (LiDAR->ranges[i]);

    currentSeconds = LiDAR->header.stamp.sec; 
}

/*! \brief Project Main
 *
 *  Parameter 1: odom/GT
 *  Choice between "odom" as odometry based approach or "GT" as an approach unsing ground truth
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
