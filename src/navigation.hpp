#include "ros/ros.h"
                   
using namespace std;

class navigationClass{
public: 
    navigationClass(ros::Publisher &drive, float pose[3]);
    bool moveTo(float desiredPose[2]);

private: 
    bool motion(ros::Publisher &drive, float desPose[3], float curPose[3]);
    
    int navigationState = 0; 
    float *currentPose;
    ros::Publisher drivePub;

    // setup 
    const float toleranceDistance = 0.1;  
    const float toleranceAngle = 2; 
    const float toleranceAngleLinVel = 30; 
    const float angularVel = 1;
    const float linearVel = 0.3;
    const float decelerateDistance = 0.5;

};

navigationClass::navigationClass(ros::Publisher &drive, float pose[3]): drivePub(drive), currentPose(pose){
}

////////////////////////////////      ROTATE    ////////////////////////////////
bool navigationClass::moveTo(float desiredPose[2]){

    float desiredAngle; 
    switch(navigationState){
        // waiting for new goals
        case 0: 
            navigationState ++;
            cout << "TURTLEBOT MOTION:" << endl; 
            cout << "  new state: " << navigationState << ":   New goal received" << endl; 
            //cout << "desPosition " << desiredPose[0] << ", " << desiredPose[1] << endl;
            //cout << "curPosition " << currentPose[0] << ", " << currentPose[1] << endl;     
            break; 

        // publish nav_status: in motion 
        case 1: 
            navigationState ++; 
            cout << "  new state: " << navigationState;          
            break; 

        // calculate dif to new goal
        case 2: 
            cout << ":   Difference: x: " << (desiredPose[0] - currentPose[0]);
            cout << "\ty: " << (desiredPose[1] - currentPose[1]) << endl; 
            navigationState ++; 
            cout << "  new state: " << navigationState << ":   Drive to new goal" << endl;          
            break; 

        // align towards new goal
        case 3: 
            if(motion(drivePub, desiredPose, currentPose)){
                navigationState ++; 
                cout << "  new state: " << navigationState << ":   Goal reached" << endl;
            }
            break; 

        // drive to new goal 
        case 4: 

            navigationState = 0;       
            return true;
    }

    return false; 
}

////////////////////////////////      ROTATE    ////////////////////////////////
bool navigationClass::motion(ros::Publisher &drive, float desPose[3], float curPose[3]){

    float difPose[3] = {0,0,0}; 

    // calculate absolute angle to goal     
    for(int i = 0; i < 2; i++){    
        difPose[i] = desPose[i] - curPose[i];
        if(abs(difPose[i]) <  toleranceDistance) 
            difPose[i] = 0;          
    }
    float absAngleToGoal =  atan2(difPose[1], difPose[0]) * 180 / PI;         

    // calculate distance to goal     
    float distToGoal = sqrt(pow(difPose[0], 2) + pow(difPose[1], 2));    

    float relAngle = absAngleToGoal - curPose[2];
    while(relAngle > 180)
        relAngle = -360+relAngle;
    while (relAngle < -180)
        relAngle = 360-relAngle; 
   
    if(relAngle < -toleranceAngleLinVel) 
        relAngle = -toleranceAngleLinVel;
    else if(relAngle >= toleranceAngleLinVel)
        relAngle = toleranceAngleLinVel; 

    if(relAngle < -180 || relAngle > 180){ cout << "ERROR relAngle: " << relAngle << "press ENTER to continue" << endl; cin.get(); } 

    geometry_msgs::Twist driveVal;

    if(distToGoal > toleranceDistance){

        
        driveVal.angular.z = mapValue(relAngle, -90, 90, -angularVel, angularVel);

       // float relAngleAbs = abs(absAngleToGoal - curPose[2]);
       // if(relAngleAbs > toleranceAngleLinVel) 
       //     relAngleAbs = toleranceAngleLinVel; 
        driveVal.linear.x = mapValue(abs(relAngle), 0, toleranceAngleLinVel, linearVel, 0);
        if(distToGoal < decelerateDistance)   
            driveVal.linear.x = mapValue(driveVal.linear.x, decelerateDistance, 0, driveVal.linear.x, 0);

        drive.publish(driveVal);

    }
    else{
        driveVal.angular.z = 0;
        driveVal.linear.x = 0;
        drive.publish(driveVal);
        return 1; 
    }

    return 0;  
}
