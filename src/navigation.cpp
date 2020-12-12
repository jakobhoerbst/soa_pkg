/*
this node ist for navigating the robot
nav_status: 
-1 ... error 
 1 ... in motion 
 2 ... reached goal 
*/

#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int8.h"
#include "gazebo_msgs/ModelStates.h"
#include <math.h>               // for atan2

#include "operations.hpp"
#include "pid.hpp"               
                                       
using namespace std;

long lauf = 0; 

const float toleranceDistance = 0.1;  // jakob: 0.05
const float toleranceAngle = 2; 
const float toleranceAngleLinVel = 30; 

const float angularVel = 0.5;
const float linearVel = 0.3;

ros::Publisher drivePub;
ros::Publisher navStatusPub; 

//desired Position
float desiredPose[3] = {0,0,0}; 

// ground truth position 
float poseGT[3] = {0,0,0};

// current pose from robot
float currentPose[3] = {0.625, 0.625, 0};

// for switch case
int navigationState = 0; 

//new goal on subscriber
bool newGoalReceived = false; 

// navigation status for topic 
std_msgs::Int8 nav_status;

struct odom_callback{
    double posX; 
    double posY; 
    double angZ; 
};
odom_callback odom; 

/*
struct nodestruct{
    double x; 
    double y; 
    double dir[4]; //r, d, l, u
    int move; 
};
*/

/*
////////////////////////////////  callbackOdometry  ////////////////////////////////
void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odometry)
{
    odom.posX = odometry -> pose.pose.position.x; 
    odom.posY = odometry -> pose.pose.position.y;
    odom.linX = odometry -> twist.twist.linear.x; 
    odom.angZ = odometry -> twist.twist.angular.z; 
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

}

////////////////////////////////  callbackIMU   ////////////////////////////////
void callbackIMU(const sensor_msgs::Imu::ConstPtr& IMU)
{
/*    
    Quaternion q; 
    EulerAngles e; 
    q.w = IMU->orientation.w;
    q.x = IMU->orientation.x;
    q.y = IMU->orientation.y;
    q.z = IMU->orientation.z;
    e = ToEulerAngles(q);
    orientationDeg = (e.yaw*180.0)/PI;
    //angleTo360(orientationDeg);
    
    //cout << "orientation: " << (e.yaw*180)/PI << endl; 
*/
}

////////////////////////////////callbackPosition////////////////////////////////
void callbackPosition(const geometry_msgs::Pose::ConstPtr& pose)
{
    
    desiredPose[0] = pose->position.x; 
    desiredPose[1] = pose->position.y; 
    
    cout << "_________________________________" << endl; 
    cout << "NEW GOAL\tx: " << desiredPose[0] << "\ty:" << desiredPose[1] << endl; 

    newGoalReceived = true; 

}

////////////////////////////////      ROTATE    ////////////////////////////////
bool motion(ros::Publisher &drive, float desPose[3], float curPose[3], float linVel){

    float difPose[3] = {0,0,0}; 

    // calculate absolute angle to goal     
    for(int i = 0; i < 2; i++){    
        difPose[i] = desiredPose[i] - curPose[i];
        if(abs(difPose[i]) <  toleranceDistance) 
            difPose[i] = 0;          
    }
    float absAngleToGoal; 
    absAngleToGoal =  atan2(difPose[1], difPose[0]) * 180 / PI;         

    // calculate distance to goal     
    float distToGoal = sqrt(pow(difPose[0], 2) + pow(difPose[1], 2));    

    // calculate relative angle to goal
    // relative oder absolute?   
    //cout << "angle diff: " << absAngleToGoal - curPose[2] << endl; 

    // decide wether turn right or left
    int dir = 0; 
    if(absAngleToGoal - curPose[2] > 0) 
        dir = 1;
    else
        dir = -1; 

    // publish on cmd_vel
    geometry_msgs::Twist driveVal;
    if(abs(absAngleToGoal - curPose[2]) > toleranceAngle){
        driveVal.angular.z = dir*angularVel;
        driveVal.linear.x = mapValue(abs(absAngleToGoal - curPose[2]), 0, toleranceAngleLinVel, linVel, 0);
        //cout << "speed: " << driveVal.linear.x << endl; 
        drive.publish(driveVal);
    }
    else if(distToGoal > toleranceDistance && linVel != 0){
        driveVal.angular.z = 0; 
        driveVal.linear.x = mapValue(abs(absAngleToGoal - curPose[2]), 0, toleranceAngleLinVel, linVel, 0);
        drive.publish(driveVal);
    }
    else{
        driveVal.angular.z = 0;
        drive.publish(driveVal);
        return 1; 
    }

    return 0;  
}

////////////////////////////////      DRIVE     ////////////////////////////////
bool drive(ros::Publisher &drive, float desPose[3], float curPose[3]){

    float difPose[3] = {0,0,0}; 


    // calculate distance to goal     
    for(int i = 0; i < 2; i++){    
        difPose[i] = desPose[i] - curPose[i];
        if(abs(difPose[i]) <  toleranceDistance) 
            difPose[i] = 0;          
    } 
    float distToGoal = sqrt(pow(difPose[0], 2) + pow(difPose[1], 2));
    //cout << "distToGoal: " << distToGoal << endl; 
        // publish on cmd_vel
    geometry_msgs::Twist driveVal;
    if(distToGoal > toleranceDistance){
        motion(drive, desPose, curPose, linearVel);

    }
    else{
        driveVal.angular.z = 0;
        driveVal.linear.x = 0;
        drive.publish(driveVal);
        return 1; 
    }
    
    return 0;  
/*
    

    // Values to be adapted
    // V1: (0.1, 100, -100, 0.1, 0.01, 0.5)
    pidTuner pidDrive(0.1, 100, -100, 0.1, 0.01, 0.5);

    for (int i = 0; i < 3; i++)
    {
        float dStartPos = curPose[i], dEndPos = desPose[i];
        // fuer eine Regeldifferenz von 10 ist iteration von 50 guter Wert mit V1
        // Annahme: Posendifferenz < 10
        for (int j = 0; j < 50; j++) 
        {
            float dIncrement = pidDrive.dCalculate( dStartPos, dEndPos);
            dEndPos += dIncrement;

            // publish on cmd_vel
            geometry_msgs::Twist driveVal;
            if(dIncrement > toleranceDistance){
                driveVal.linear.x = linearVel;
                drive.publish(driveVal);
                rotate(drive, desPose, curPose, linearVel);
            }
            else{
                driveVal.angular.z = 0;
                driveVal.linear.x = 0;
                drive.publish(driveVal);
                return 1; 
            }
        }
    }

    
    
    return 0;  
*/
}


////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
int main(int argc, char **argv ) {

    cout << "- NAVIGATION -" << endl << endl;

    //////////////// ROS ////////////////
    ros::init(argc, argv, "navigationNode");
    ros::NodeHandle nh("~"); 

    ros::Subscriber subscriberIMU;
    ros::Subscriber subscriberPosition; 
    ros::Subscriber subscriberGT;   

    subscriberIMU       = nh.subscribe("/imu", 100, callbackIMU);
    subscriberPosition  = nh.subscribe("/nextPosition", 100, callbackPosition);  
    subscriberGT        = nh.subscribe("/gazebo/model_states", 100, callbackGT); //GT ... ground truth  

    drivePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    navStatusPub = nh.advertise<std_msgs::Int8>("/nav_status", 100);
/*
    cout << "current pose[0]: " << currentPose[0] << "  PoseGT[0]: " << poseGT[0] << endl;
    cout << "current pose[1]: " << currentPose[1] << "  PoseGT[1]: " << poseGT[1] << endl;
    cout << "current pose[2]: " << currentPose[2] << "  PoseGT[2]: " << poseGT[2] << endl;
*/
    while(ros::ok())
    {

    // get current pose from ground truth
    currentPose[0] = poseGT[0];
    currentPose[1] = poseGT[1];
    currentPose[2] = poseGT[2];

        float desiredAngle; 
        switch(navigationState){
            // waiting for new goals
            case 0: 
                //newGoalReceived = true;
                //callbackPosition;
                if(newGoalReceived){
                    cout << "press enter" << endl;     
                    cin.get();
                    navigationState ++;
                    newGoalReceived = false;  
                    cout << "new state: " << navigationState << endl;          
                } 
                break; 
            // publish nav_status: in motion 
            case 1: 
                nav_status.data = 1; 
                navStatusPub.publish(nav_status);
                navigationState ++; 
                cout << "new state: " << navigationState;          
                break; 
            // calculate dif to new goal
            case 2: 
                cout << ":  Difference: x: " << desiredPose[0] - currentPose[0];
                cout << "\ty: " << desiredPose[1] - currentPose[1] << endl; 
                navigationState ++; 
                cout << "new state: " << navigationState << ":  Align towards new goal" << endl;          
                break; 
            // align towards new goal
            case 3: 
                if(motion(drivePub, desiredPose, currentPose, 0)){
                    navigationState ++; 
                    cout << "new state: " << navigationState << ":  Drive to new goal" << endl;}          
                break; 
            // drive to new goal 
            case 4: 
                if(drive(drivePub, desiredPose, currentPose)){
                    navigationState ++; 
                    cout << "new state: " << navigationState << ":  Goal reached" << endl;
                    //cout << endl << "goalReached " << endl; 
                    nav_status.data = 2; 
                    navStatusPub.publish(nav_status);  
                    navigationState = 0;
                }                        
                break; 
        }

        ros::spinOnce();        
    }  

return 0; 
}
