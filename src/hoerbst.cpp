/*

note: visual studio code

next steps: 
_turn 90deg in relation to distances 
_align turtle to maze wall 
_intersection detection


sources: 
Quarternion - Euler
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles


0 ... right 
1 ... down
2 ... left 
3 ... up     

jakob@ubuntu:~$ roslaunch turtlebot3_bringup turtlebot3_model.launch 

*/


#include "ros/ros.h"
//#include "opencv2/core.hpp"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"


//#include "geometry_msgs//PoseWithCovarianceStamped.h"
#include <vector>
//#include <iterator>
#include <math.h>
//#include <numeric>                                                              

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


bool positionReached = false;

const float width = 1.25;           // kantenlänge ein einem Segment 
const float wallThickness = 0.15; 
const float pathWidth = 1.1;        // befahrbare Pfadbreit

static double closed = 0.8; 
static double current = 0.8;
static double visited  = 0.4;

double position[2] = {0,0};

float   dt = 0.2;

float scanResult[360]; 
bool newLidar = false;
float minWidth = 3.5;    

ros::Publisher drivePub;
bool  hold = false; 
float toCloseTreshold = 0.2;
float vel = 0.1; 
float orientedDistances[4] = {0,0,0,0}; 

//driveOneField: 
float   DOF_currentLin = 0; 
int     DOF_state = 0; 
bool    DOF_next = false; 

//IMU 
double   initOrientation = 0; 


//drive
bool    DRI_nextDecission = false;

//orientation 
double  orientationDeg = 0; 
int     orientation = 0; 


//start routine
bool    startupComplete = false; 


ros::Time       oldTime, newTime;

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


////////////////////////////      measure width     ////////////////////////////
float measureWidth()
{

    for(int i = 0; i < 180; i++){
        if(minWidth > scanResult[i]+scanResult[i+180])
            minWidth = scanResult[i]+scanResult[i+180];
    }
    
    return minWidth; 
}

//////////////////////////// intersection detection ////////////////////////////
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




////////////////////////////       directions       ////////////////////////////
// calculates mean distances to fall for front, right, back, left side of turtle
void directions(){
    
    float dirDistance[4] = {0,0,0,0}; 
    int range = 10; 

    for(int j = 0; j < 4; j++){
        for(int i = j*90-range; i < j*90+range; i++){
            if(i<0)
                dirDistance[j] += scanResult[i+360];
            else
                dirDistance[j] += scanResult[i];
        } 
        dirDistance[j] = dirDistance[j]/(2*range);
    }

    for(int i = 0; i < 4; i++){
        if(i-orientation < 0) 
            orientedDistances[i-orientation+4] = dirDistance[i];
        else            
            orientedDistances[i-orientation] = dirDistance[i];
    }

    cout << "_____dirDistance_____" << endl; 
    cout << "right: " << orientedDistances[0]; 
    cout << "\tdown: " << orientedDistances[1];
    cout << "\tleft: " << orientedDistances[2];
    cout << "\tup: " << orientedDistances[3] << endl;

}


////////////////////////////////  callbackOdometry  ////////////////////////////////
void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odometry)
{

    odom.posX = odometry -> pose.pose.position.x; 
    odom.posY = odometry -> pose.pose.position.y;
    odom.linX = odometry -> twist.twist.linear.x; 
    odom.angZ = odometry -> twist.twist.angular.z; 

}

////////////////////////////////  callbackLiDAR  ////////////////////////////////
void callbackLiDAR(const sensor_msgs::LaserScan::ConstPtr& LiDAR)
{

    newLidar = true; 
   // inRange = 0; 
  
    for(int i = 0; i < 360; i++)
        scanResult[359-i] = (LiDAR->ranges[i]);

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

////////////////////////////    get orientation     ////////////////////////////
int getOrientation(){
  //  cout << "orientation difference: " << orientation - initOrientation << endl; 
  //  cout << "initial Orientation" << initOrientation << endl; 

    //r, d, l, u    

    int toleranceAngle = 45; 
    
    cout << "orientationDeg: " << orientationDeg; 
    
    if(abs(orientationDeg) < toleranceAngle) 
        orientation = 0; 
    else if(orientationDeg < 90+toleranceAngle && orientationDeg > 90-toleranceAngle) 
        orientation = 1; 
    else if(abs(orientationDeg) < 180 && abs(orientationDeg) > 180-toleranceAngle) 
        orientation = 2; 
    else if(orientationDeg < -90+toleranceAngle && orientationDeg > -90-toleranceAngle) 
        orientation = 3;      
    else
        cout << "ERROR at finding orientation" << endl;  

    cout << "\torientation: " << orientation << endl; 
}

void angleTo360(double &angle){
    if(angle < 0) 
        angle = -angle; 
    else
        angle = 360 - angle; 
}


////////////////////////////         scan           ////////////////////////////
void scan(vector<nodestruct> &node){
    cout << node[node.size()-1].x << endl; 
    
    for(int i = 0; i < 4; i++){
        //cout << "distancse " << orientedDistances[i] << endl; 
        if(orientedDistances[i] > pathWidth) 
            node[node.size()-1].dir[i] = 0;
        else 
            node[node.size()-1].dir[i] = 1;

        //cout << "result " << node[node.size()-1].dir[i] << endl; 
    }

    node[node.size()-1].x = odom.posX;
    node[node.size()-1].y = odom.posY;
    
    node[node.size()-1].dir[2] = visited; // because we came from this direction
    //printNode(node);

}

////////////////////////////       printNode        ////////////////////////////
void printNode(vector<nodestruct> currentNode){

    cout << "node: " << currentNode.size();
    cout << "\tx: " /*<< std::setprecision(2)*/ << currentNode[currentNode.size()-1].x;
    cout << "\ty: " /*<< std::setprecision(2)*/ << currentNode[currentNode.size()-1].y;
    cout << "\tr: " << currentNode[currentNode.size()-1].dir[0] << 
            "\td: " << currentNode[currentNode.size()-1].dir[1] << 
            "\tl: " << currentNode[currentNode.size()-1].dir[2] << 
            "\tu: " << currentNode[currentNode.size()-1].dir[3] << 
            "\tmove: ";
    //r, d, l, u    
    switch(currentNode[currentNode.size()-1].move){
        case 0: 
            cout << "r" << endl; 
            break; 
        case 1: 
            cout << "d" << endl; 
            break; 
        case 2: 
            cout << "l" << endl; 
            break; 
        case 3:
            cout << "u" << endl; 
            break; 
        
    }

}


////////////////////////////////   distToWall   ////////////////////////////////
// finds angle to closest wall
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
    ros::init(argc, argv, "DFSnode");
    ros::NodeHandle nh("~"); 
    
    cout << "- PROJECT 2 -" << endl;

    ros::Subscriber subscriberOdometry; 
    ros::Subscriber subscriberLiDAR;
    ros::Subscriber subscriberIMU;
    
    //subscriberOdometry  = nh.subscribe("cmd_vel", 10, callbackOdometry);
    subscriberOdometry  = nh.subscribe("/odom", 100, callbackOdometry);
    subscriberLiDAR     = nh.subscribe("/scan", 100, callbackLiDAR);
    subscriberIMU       = nh.subscribe("/imu", 100, callbackIMU);
    
    drivePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    //////////////// directed graph ////////////////
    vector<nodestruct> graph; 
    graph.push_back(nodestruct());

    //while(drive(drivePub)){cout << "initial position" << endl;}

    startupComplete = true; 
    bool initialLiader = false; 
    positionReached = true; 
    while(ros::ok())
    {
    
        /*
        if(!startupComplete){
            initOrientation = orientation;
            if(initOrientation != 0.0)
                startupComplete = true; 
        }
        */

        //if(newLidar && startupComplete){
            //if(DOF_next)
        if(newLidar){
            //newPosition = true;      
            initialLiader = true;    
        }       

            //if(!drive(drivePub) && DOF_next){
            //    DOF_next = false; 
               // getOrientation();

                // cout << "posX: " << odom.posX << "\tposY: " << odom.posY << endl;


        if(positionReached && initialLiader){
            cout << "new Position" << endl; 
            getOrientation();
            directions(); 
            scan(graph);

                // deciding next movement (prefered: keep previous direction)
                for(int i = 0; i < 4; i++){
                    //if((prevDirection+i)>4)
                    //    prevDirection -= 4; 

                    if(graph[graph.size()-1].dir[i] == 0){ 
                        graph[graph.size()-1].move = (i); 
                        //newMovement = true; 
                        break; 
                    }
                  
                } 
            printNode(graph);
            positionReached = false;




        } 




            

               // graph[graph.size()-1] = scan(graph[graph.size()-1]);
            //newMovement = false; 
            //nodelist = setStatus(nodelist, visited);


            //getOrientation();
           // cout << "orienatation" << orientation << endl; 


            //cout << "move.linear.x: " << drive(drivePub).linear.x << endl;
         
           // newLidar = false; 
            
        


  //      ros::Duration(dt).sleep();
        ros::spinOnce();
        
    }  

return 0; 
}
