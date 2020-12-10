#include <math.h>
#include <vector>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

const double PI = 3.14159265359;

EulerAngles ToEulerAngles(Quaternion q) {

    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

void angleTo360(double &angle){
    if(angle < 0) 
        angle = -angle; 
    else
        angle = 360 - angle; 
}





////////////////////////////////      LAGER     ////////////////////////////////
////////////////////////////////      LAGER     ////////////////////////////////
////////////////////////////////      LAGER     ////////////////////////////////



//float minWidth = 3.5;    


//bool  hold = false; 
//float toCloseTreshold = 0.2;
//float vel = 0.1; 
//drive
//bool    DRI_nextDecission = false;

//driveOneField: 
//float   DOF_currentLin = 0; 
//int     DOF_state = 0; 
//bool    DOF_next = false; 

//IMU 
//double   initOrientation = 0; 

//ros::Time       oldTime, newTime;

////////////////////////////////   distToWall   ////////////////////////////////
// finds angle to closest wall
/*
int distToWall(){

    int closestWallAlign; 
    float closestWallDist = 3.5; 

    for(int i = 0; i < 360; i++){

            if(closestWallDist > scanResult[i]){ 
                closestWallDist = scanResult[i]; 
                closestWallAlign = i; 
            }
           
    }
  
    return closestWallAlign; 
}
*/
/*
bool driveOneField(ros::Publisher &drive, int direction){

    geometry_msgs::Twist driveVal;
    
    switch(DOF_state)
    {
    case 0: 
        DOF_currentLin = odom.linX; 
        DOF_state++; 
        break; 
    case 1: 
        if((odom.linX - DOF_currentLin) < pathWidth){
            driveVal.linear.x = vel;
            drive.publish(driveVal);
            DOF_next = true; 
        }
        else{
            driveVal.linear.x = 0;
            drive.publish(driveVal);
            intersectionDetection();
            DOF_state ++; 
            return 0; 
        }
        break; 
    case 2: 
        return 0; 
        break; 
    }

    return 1;
}
*/

////////////////////////////////      DRIVE     ////////////////////////////////
/*
bool drive(ros::Publisher &drive){
    
    geometry_msgs::Twist driveVal;

    if(scanResult[0] > pathWidth/2) 
        //driveVal.linear.x = vel;
        return driveOneField(drive, 0); 
    else{
        driveVal.linear.x = 0; 
        drive.publish(driveVal);
        DRI_nextDecission = true; 
        intersectionDetection();
        return 0; 
    }        
    
    return 1;   
}
*/

////////////////////////////      measure width     ////////////////////////////
/*
float measureWidth()
{

    for(int i = 0; i < 180; i++){
        if(minWidth > scanResult[i]+scanResult[i+180])
            minWidth = scanResult[i]+scanResult[i+180];
    }
    
    return minWidth; 
}
*/

//////////////////////////// intersection detection ////////////////////////////
/*
void intersectionDetection()
{
    int notAWall = 0; 
    for(int i = 0; i < 4; i++){
        if(orientedDistances[i]>minWidth)
            notAWall++; 
    }

    if(notAWall>2){ 
        cout << "intersection detected" << endl; 
    
    }


}
*/



