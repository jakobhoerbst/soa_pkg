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
#include "std_msgs/Int8.h"
#include "gazebo_msgs/ModelStates.h"
#include <vector>
    
// own header                                                   
#include "operations.hpp"
#include "DFS.hpp"
#include "navigation.hpp"

using namespace std;

// LiDAR 
float scanResult[360]; 
bool initialLidar = false;

// ground truth position 
float poseGT[3] = {0,0,0};
bool initialGT = false; 

// odom position
float poseOdom[3] = {0,0,0};
bool initialOdom = false; 

// feedback from navigation
bool positionReached = true;

// next Position from DFS to navigation
float nextPosition[2] = {0,0};

////////////////////////////////       SUB      ////////////////////////////////
////////////////////////////////       SUB      ////////////////////////////////
////////////////////////////////       SUB      ////////////////////////////////

////////////////////////////////callbackOdometry////////////////////////////////
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

////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
int main(int argc, char **argv ) {
    
    string argument = argv[1];

    cout << " - PROJECT 2 -" << endl << endl;

    if(argument == "GT")
        cout << "using: ground truth " << endl;
    else if(argument == "odom") 
        cout << "using: odom" << endl; 
    else{
        cout << "ERROR with input argument" << endl; 
        return 0;     
    }

    //////////////// ROS ////////////////
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

    // set pointer to GT or odom
    if(argument == "GT"){
        DFS.currentPose = poseGT;
        navigation.currentPose = poseGT; 
    }
    else if(argument == "odom"){
        DFS.currentPose = poseOdom;
        navigation.currentPose = poseOdom; 
    }

    // starup and waiting for initial sensor data
    while(ros::ok())
    {
        if(initialLidar && initialGT && initialOdom){
            cout << "STARTUP complete" << endl; 
            break;             
        }
        ros::spinOnce();
    }

    // loop solving the maze
    while(ros::ok())
    {
        // if new position is reached calculate the next movement
        if(positionReached){
            positionReached = false;
            if(DFS.handleNode())
                break;
        } 
  
        // navigate to next movement
        if(navigation.moveTo(nextPosition))
            positionReached = true; 

        ros::spinOnce();
        
    }  

return 0; 
}
