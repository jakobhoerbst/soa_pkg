/*

this node ist for navigating the robot
*/


#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"

                                                             

using namespace std;

const double PI = 3.14159265359;

struct odom_callback{
    double posX; 
    double posY; 
    double linX;
    double angZ; 

};
odom_callback odom; 

struct nodestruct{
    double x; 
    double y; 

    double dir[4]; //r, d, l, u

    int move; 
};



struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};


////////////////////////////       prototypes       ////////////////////////////
void printNode(vector<nodestruct> currentNode);
void angleTo360(double &angle);



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



////////////////////////////////  callbackOdometry  ////////////////////////////////
void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odometry)
{

    odom.posX = odometry -> pose.pose.position.x; 
    odom.posY = odometry -> pose.pose.position.y;
    odom.linX = odometry -> twist.twist.linear.x; 
    odom.angZ = odometry -> twist.twist.angular.z; 

}

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
    //angleTo360(orientationDeg);

    
    //cout << "orientation: " << (e.yaw*180)/PI << endl; 

}



////////////////////////////////      DRIVE     ////////////////////////////////
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




////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
int main(int argc, char **argv ) {

    //////////////// ROS ////////////////
    ros::init(argc, argv, "navigationNode");
    ros::NodeHandle nh("~"); 
    
    //cout << "- PROJECT 2 -" << endl;

    ros::Subscriber subscriberOdometry; 
    //ros::Subscriber subscriberLiDAR;
    ros::Subscriber subscriberIMU;
    
    //subscriberOdometry  = nh.subscribe("cmd_vel", 10, callbackOdometry);
    subscriberOdometry  = nh.subscribe("/odom", 100, callbackOdometry);
    //subscriberLiDAR     = nh.subscribe("/scan", 100, callbackLiDAR);
    subscriberIMU       = nh.subscribe("/imu", 100, callbackIMU);
    
    drivePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    int currentPos[2] = {0.625, 0.625};
    int desiredPos[2] = {0.625, - 0.625};  
    int currentOrientation = 0; 

    while(ros::ok())
    {
    
       
        ros::spinOnce();
        
    }  

return 0; 
}
