/*

next steps: 
_ ground truth nicht mehr verwenden
_ namespace weg

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
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int8.h"
#include "gazebo_msgs/ModelStates.h"
#include <vector>
    
// own header                                                   
#include "operations.hpp"
#include "DFS.hpp"
#include "navigation.hpp"

using namespace std;
/*
struct odom_callback{
    double posX; 
    double posY; 
    double linX;
    double angZ; 
};
odom_callback odom; 
*/

// LiDAR 
float scanResult[360]; 
bool initialLidar = false;

// ground truth position 
float poseGT[3] = {0,0,0};
bool initialGT = false; 

bool positionReached = true;

// next Position from DFS to navigation
float nextPosition[2] = {0,0};

// status from navigation node
/*
int nav_status = 0; 
bool positionReceived = false; 

bool sendPosition = false; 
bool navigationReady = false; 

// odom
bool initialOdom = false; 
*/



////////////////////////////////       SUB      ////////////////////////////////
////////////////////////////////       SUB      ////////////////////////////////
////////////////////////////////       SUB      ////////////////////////////////
/*
////////////////////////////////callbackOdometry////////////////////////////////
void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odometry)
{

    odom.posX = odometry -> pose.pose.position.x; 
    odom.posY = odometry -> pose.pose.position.y;
    odom.linX = odometry -> twist.twist.linear.x; 
    odom.angZ = odometry -> twist.twist.angular.z; 

    initialOdom = true; 

}
*/
////////////////////////////      callbackGT        ////////////////////////////
void callbackGT(const gazebo_msgs::ModelStates::ConstPtr& GT){

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


////////////////////////////////  callbackLiDAR ////////////////////////////////
void callbackLiDAR(const sensor_msgs::LaserScan::ConstPtr& LiDAR)
{

    initialLidar = true; 
   // inRange = 0; 
  
    for(int i = 0; i < 360; i++)
        scanResult[359-i] = (LiDAR->ranges[i]);

}
/*
////////////////////////////////  callbackIMU   ////////////////////////////////
void callbackIMU(const sensor_msgs::Imu::ConstPtr& IMU)
{
    
    Quaternion q; 
    EulerAngles e; 

    q.w = IMU->orientation.w;
    q.x = IMU->orientation.x;
    q.y = IMU->orientation.y;
    q.z = IMU->orientation.z;

    e = ToEulerAngles(q);
    orientationDeg = (e.yaw*180.0)/PI;

}
*/

////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
int main(int argc, char **argv ) {

    cout << "- PROJECT 2 -" << endl << endl;

    //////////////// ROS ////////////////
    ros::init(argc, argv, "DFSnode");
    ros::NodeHandle nh("~"); 
    
    ros::Subscriber subscriberLiDAR;
    ros::Subscriber subscriberGT;
    //ros::Subscriber subscriberOdometry; 
    //ros::Subscriber subscriberIMU;

    ros::Publisher drivePub;

    subscriberLiDAR     = nh.subscribe("/scan", 100, callbackLiDAR);
    subscriberGT        = nh.subscribe("/gazebo/model_states", 100, callbackGT);
    //subscriberOdometry  = nh.subscribe("/odom", 100, callbackOdometry);
    //subscriberIMU       = nh.subscribe("/imu", 100, callbackIMU);

    drivePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);


    // class for moving the robot (included in navigation.hpp)
    navigationClass navigation(drivePub, poseGT);
    // class for DFS algorithm (included in DFS.hpp)
    DFSClass DFS(poseGT, scanResult, nextPosition); 
    

    // starup and waiting for initial sensor data
    while(ros::ok())
    {
        if(initialLidar && initialGT){
            cout << "STARTUP complete" << endl; 
            break;             
        }
        ros::spinOnce();
    }

    // loop solving the maze
    while(ros::ok())
    {

        if(positionReached){
            positionReached = false;
            if(DFS.handleNode())
                break;
        } 
  
        if(navigation.moveTo(nextPosition))
            positionReached = true; 

        ros::spinOnce();
        
    }  

return 0; 
}
