/*     
    Maze Solving Algorithm:
    navigations 
    Version 1
    Autor: Hoerbst
    Contributor: Gmeiner
 */

#include "ros/ros.h"
                   
using namespace std;

/*! \brief brief descr
 *
 *  Detailed description starts here.
 */
class navigationClass
{
    public: 
        navigationClass(ros::Publisher &drive);     
        bool moveTo(float desiredPose[2]);          
        float *currentPose;                         /*!< Pointer to main */

    private: 
        bool motion(ros::Publisher &drive, float desPose[3], float curPose[3]);
        
        int navigationState = 0; 
        ros::Publisher drivePub;

        const float toleranceDistance = 0.1;        /*!< setup */
        const float toleranceAngle = 2;             /*!< setup */ 
        const float toleranceAngleLinVel = 30;      /*!< setup */ 
        const float angularVel = 1;                 /*!< setup */
        const float linearVel = 0.3;                /*!< setup */
        const float decelDist = 0.5;                /*!< setup */
};
/*! \brief Brauchts den? der ist ja leer .... 
 *
 *  Detailed description starts here.
 */
navigationClass::navigationClass(ros::Publisher &drive): drivePub(drive){}

/*! \brief brief descr
 *
 *  Detailed description starts here.
 */
bool navigationClass::moveTo(float desiredPose[2])
{
    float desiredAngle; 
    switch(navigationState)
    {
        // waiting for new goals
        case 0: 
            navigationState ++;
            cout << "TURTLEBOT MOTION:" << endl; 
            cout << "  new state: " << navigationState << ":   New goal received" << endl;    
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
            if(motion(drivePub, desiredPose, currentPose))
            {
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

/*! \brief brief descr
 *
 *  Detailed description starts here.
 */
bool navigationClass::motion(ros::Publisher &drive, float desPose[3], float curPose[3])
{
    float difPose[3] = {0,0,0}; 

    // calculate absolute angle to goal     
    for(int i = 0; i < 2; i++)
    {    
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

    geometry_msgs::Twist driveVal;

    if(distToGoal > toleranceDistance)
    {
        driveVal.angular.z = mapValue(relAngle, -90, 90, -angularVel, angularVel);

        driveVal.linear.x = mapValue(abs(relAngle), 0, toleranceAngleLinVel, linearVel, 0);
        if(distToGoal < decelDist)   
            driveVal.linear.x = mapValue(driveVal.linear.x, decelDist, 0, driveVal.linear.x, 0);
        drive.publish(driveVal);
    }
    else
    {
        driveVal.angular.z = 0;
        driveVal.linear.x = 0;
        drive.publish(driveVal);
        return 1; 
    }
    return 0;  
}
