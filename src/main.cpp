/*

next steps: 
_ PID regler für navigation 
_ ground truth nicht mehr verwenden
_ klassen erstellen
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
#include "navigation.hpp"

using namespace std;

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

    double dir[5];                          //dead end, r, d, l, u

    int move; 
};

struct coordinatesStruct{
    double x; 
    double y; 
};
vector<coordinatesStruct> visitedVector;  


const float width           = 1.25;         // kantenlänge ein einem Segment 
const float wallThickness   = 0.15; 
const float pathWidth       = 1.1;          // befahrbare Pfadbreit
const float exitDistance    = 2.5;

// values from using cv::Mat for visualization
static double closed        = 0.8; 
static double current       = 0.8;
static double visited       = 0.4;

// LiDAR 
float scanResult[360]; 
bool initialLidar = false;

// ground truth position 
float poseGT[3] = {0,0,0};
bool initialGT = false; 

//orientation 
double  orientationDeg = 0; 
int     orientation = 0; 
float   orientedDistances[5] = {0,0,0,0,0}; 

//start routine
bool    startupComplete = false; 

//
bool    newMovement = true; 
bool    escaped = false; 

// status from navigation node
int nav_status = 0; 
bool positionReceived = false; 
bool positionReached = true;
bool sendPosition = false; 
bool navigationReady = false; 

// odom
bool initialOdom = false; 


float nextPosition[2] = {0,0};

////////////////////////////       prototypes       ////////////////////////////
void printNode(vector<nodestruct> currentNode);
void angleTo360(double &angle);

////////////////////////////////       SUB      ////////////////////////////////
////////////////////////////////       SUB      ////////////////////////////////
////////////////////////////////       SUB      ////////////////////////////////

////////////////////////////////callbackOdometry////////////////////////////////
void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odometry)
{

    odom.posX = odometry -> pose.pose.position.x; 
    odom.posY = odometry -> pose.pose.position.y;
    odom.linX = odometry -> twist.twist.linear.x; 
    odom.angZ = odometry -> twist.twist.angular.z; 

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

}


////////////////////////////////  callbackLiDAR ////////////////////////////////
void callbackLiDAR(const sensor_msgs::LaserScan::ConstPtr& LiDAR)
{

    initialLidar = true; 
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

    //cout << "orientationDeg: " << orientationDeg << endl; 

}

////////////////////////////////   FUNCTIONS    ////////////////////////////////
////////////////////////////////   FUNCTIONS    ////////////////////////////////
////////////////////////////////   FUNCTIONS    ////////////////////////////////

////////////////////////////       directions       ////////////////////////////
// calculates mean distances to fall for front, right, back, left side of turtle
void directions(){
    
    float dirDistance[5] = {0,0,0,0,0}; 
    int range = 10; 

    for(int j = 0; j < 4; j++){
        for(int i = j*90-range; i < j*90+range; i++){
            if(i<0)
                dirDistance[j+1] += scanResult[i+360];
            else
                dirDistance[j+1] += scanResult[i];
        } 
        dirDistance[j+1] = dirDistance[j+1]/(2*range);
    }

    for(int i = 1; i < 5; i++){
        if( (orientation+(i-1)) > 4 )
            orientedDistances[orientation+(i-1)-4] = dirDistance[i];
        else
            orientedDistances[orientation+(i-1)] = dirDistance[i]; 
    }   

    
/*
    cout << "[-dirDistance-]  Mean distance to wall of turtle to: " << endl; 
    cout << "right: " << orientedDistances[1]; 
    cout << "\tdown: " << orientedDistances[2];
    cout << "\tleft: " << orientedDistances[3];
    cout << "\tup: " << orientedDistances[4] << endl;
*/
}

////////////////////////////        checkExit       ////////////////////////////

bool checkExit(){

    int counter = 0; 
    for(int i = 1; i < 5; i++){
        if(orientedDistances[i] > exitDistance) 
            counter ++;  
    }   

    if(counter >= 4) 
        return true; 

    return false; 

}

////////////////////////////    get orientation     ////////////////////////////
int getOrientation(){
  //  cout << "orientation difference: " << orientation - initOrientation << endl; 
  //  cout << "initial Orientation" << initOrientation << endl; 

    //r, d, l, u    

    int toleranceAngle = 45; 
    
    //cout << "orientationDeg: " << orientationDeg; 
    
    if(abs(orientationDeg) < toleranceAngle) 
        orientation = 1; 
    else if(orientationDeg < 90+toleranceAngle && orientationDeg > 90-toleranceAngle) 
        orientation = 4; 
    else if(abs(orientationDeg) < 180 && abs(orientationDeg) > 180-toleranceAngle) 
        orientation = 3; 
    else if(orientationDeg < -90+toleranceAngle && orientationDeg > -90-toleranceAngle) 
        orientation = 2;      
    else
        cout << "ERROR at finding orientation" << endl;  

    //cout << "\torientation: " << orientation << endl; 

    // Sometimes error in catkin_make: no return statement in non-void !
    return orientation;
}

////////////////////////////       setStatus        ////////////////////////////
vector<nodestruct> setStatus(vector<nodestruct> &node, double newStatus){

    int motion = 0; 

    if(newStatus == closed)
        motion = node[node.size()-1].move;
    else if(newStatus == visited) 
        motion = node[node.size()-2].move;
    else
        cout << "- ERROR: newStatus -" << endl;  

    switch(motion){
        case 0: 
            break;
        case 1: 
            if(newStatus == closed)
                node[node.size()-1].dir[1] = newStatus;
            else
                node[node.size()-1].dir[3] = newStatus; 
            break; 
        case 2: 
            if(newStatus == closed)
                node[node.size()-1].dir[2] = newStatus;
            else            
                node[node.size()-1].dir[4] = newStatus; 
            break; 
        case 3: 
            if(newStatus == closed)
                node[node.size()-1].dir[3] = newStatus;
            else
                node[node.size()-1].dir[1] = newStatus; 
            break;
        case 4: 
            if(newStatus == closed)
                node[node.size()-1].dir[4] = newStatus;
            else            
                node[node.size()-1].dir[2] = newStatus; 
            break; 
    }

    return node;
}


////////////////////////////         scan           ////////////////////////////
void scan(vector<nodestruct> &node){
    
    for(int i = 1; i < 5; i++){
        if(orientedDistances[i] > pathWidth)
            node[node.size()-1].dir[i] = 0;
        else 
            node[node.size()-1].dir[i] = 1; 
    }

    node[node.size()-1].x = odom.posX;
    node[node.size()-1].y = odom.posY;

    //printNode(node);
}

////////////////////////////     checkIfVisited     ////////////////////////////
bool checkIfVisited(vector<nodestruct> &node){

    coordinatesStruct currentPos = {node[node.size()-1].x, node[node.size()-1].y};

    for(int i = 0; i < (visitedVector.size()-1); i++){
        if(visitedVector[i].x == node[node.size()-1].x && visitedVector[i].y == node[node.size()-1].y){
            cout << " - VISITED BEFORE -" << endl; 
            //visumaze.at<double>(NV[NV.size()-1].y,NV[NV.size()-1].x) = 0.1;    
            node.pop_back();
            //nodelist.pop_back();
            node = setStatus(node, closed);
            return 1; 
        }
    }
    visitedVector.push_back(currentPos);
    return 0;
}

////////////////////////////  correctNodePosition   ////////////////////////////
void correctNodePosition(vector<nodestruct> &node){

    float minimumPosition[2] = {-5.625, -5.625};
    
    for(int i = 0; i < 10; i++){
        if(abs(node[node.size()-1].x-(minimumPosition[0]+width*i)) < 0.2){
            node[node.size()-1].x = (minimumPosition[0]+width*i);
        }
        if(abs(node[node.size()-1].y-(minimumPosition[1]+width*i)) < 0.2){
            node[node.size()-1].y = (minimumPosition[1]+width*i);
        }
    }

}


////////////////////////////       printNode        ////////////////////////////
void printNode(vector<nodestruct> currentNode){

    cout << "  node: " << currentNode.size();
    cout << " pos: (" /*<< std::setprecision(2)*/ << currentNode[currentNode.size()-1].x;
    cout << ", " /*<< std::setprecision(2)*/ << currentNode[currentNode.size()-1].y;
    cout << ")\tr: " << currentNode[currentNode.size()-1].dir[1] << 
            "\td: " << currentNode[currentNode.size()-1].dir[2] << 
            "\tl: " << currentNode[currentNode.size()-1].dir[3] << 
            "\tu: " << currentNode[currentNode.size()-1].dir[4] << 
            "\tmove: ";
    //dead end, r, d, l, u    
    switch(currentNode[currentNode.size()-1].move){
        case 0: 
            cout << "back" << endl; 
            break;
        case 1: 
            cout << "r" << endl; 
            break; 
        case 2: 
            cout << "d" << endl; 
            break; 
        case 3: 
            cout << "l" << endl; 
            break; 
        case 4:
            cout << "u" << endl; 
            break; 
        default: 
            cout << endl;    
            break;
    }

}

////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
////////////////////////////////      main      ////////////////////////////////
int main(int argc, char **argv ) {

    cout << "- PROJECT 2 -" << endl << endl;

    //////////////// ROS ////////////////
    ros::init(argc, argv, "DFSnode");
    ros::NodeHandle nh("~"); 
    
    ros::Subscriber subscriberOdometry; 
    ros::Subscriber subscriberLiDAR;
    ros::Subscriber subscriberIMU;
    ros::Subscriber subscriberGT;

    ros::Publisher drivePub;

    subscriberOdometry  = nh.subscribe("/odom", 100, callbackOdometry);
    subscriberLiDAR     = nh.subscribe("/scan", 100, callbackLiDAR);
    subscriberIMU       = nh.subscribe("/imu", 100, callbackIMU);
    subscriberGT        = nh.subscribe("/gazebo/model_states", 100, callbackGT);

    drivePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    //////////////// directed graph ////////////////
    vector<nodestruct> graph;
    graph.push_back(nodestruct());

    // class for moving the robot
    navigationClass navigation(drivePub, poseGT);
 
    while(ros::ok())
    {

        if(initialLidar && initialOdom && !startupComplete){
            cout << "STARTUP complete" << endl; 
            coordinatesStruct initialPos = {odom.posX, odom.posY}; // noch ändern 
            visitedVector.push_back(initialPos); 
            correctNodePosition(graph);

            startupComplete = true;              
        }       


        if(positionReached && startupComplete){
            positionReached = false;

            cout << "__________________________________" << endl; 
            //cout << "new position reached" << endl; 
            cout << "DFS algorithm:" << endl; 

            getOrientation();
            directions();
            if(checkExit()){
                cout << endl; 
                cout << "----------------------------------" << endl; 
                cout << "            EXIT FOUND" << endl; 
                cout << "----------------------------------" << endl; 
                escaped = true;
                break; 
            }

            // scan if new node is reached
            if(newMovement){
                scan(graph);
                newMovement = false; 
                setStatus(graph, visited);
                correctNodePosition(graph);
            }

            // set previous direction (necessary for first node)
            int prevDirection; 
            if(graph.size()<2)
                prevDirection = 1; 
            else
                prevDirection = graph[graph.size()-2].move;
     
            // deciding next movement (prefered: keep previous direction)
            for(int i = 0; i < 4; i++){
                if((prevDirection+i)>4)
                    prevDirection -= 4; 

                if(graph[graph.size()-1].dir[(prevDirection+i)] == 0){ 
                    graph[graph.size()-1].move = (prevDirection+i); 
                    newMovement = true; 
                    break; 
                }
              
            }    
            if(!newMovement)
                graph[graph.size()-1].move = 0;
    
            // print the current node
            printNode(graph);

            //move
            int motion = graph[graph.size()-1].move;
            switch(motion){
                case 0: // move back 
                    cout << " - DEAD END -" << endl; 
                    graph.pop_back();
                    graph = setStatus(graph, closed);
                    
                    break;
                case 1: // move right
                    graph.push_back(nodestruct());
                    graph[graph.size()-1].x = graph[graph.size()-2].x + width;
                    graph[graph.size()-1].y = graph[graph.size()-2].y;
                    newMovement = !checkIfVisited(graph);         
                    break;
                case 2: // move dowm
                    graph.push_back(nodestruct());
                    graph[graph.size()-1].x = graph[graph.size()-2].x;
                    graph[graph.size()-1].y = graph[graph.size()-2].y - width;
                    newMovement = !checkIfVisited(graph); 
                    break;
                case 3: // move left
                    graph.push_back(nodestruct());
                    graph[graph.size()-1].x = graph[graph.size()-2].x - width;
                    graph[graph.size()-1].y = graph[graph.size()-2].y;
                    newMovement = !checkIfVisited(graph);
                    break;
                case 4: // move up
                    graph.push_back(nodestruct());
                    graph[graph.size()-1].x = graph[graph.size()-2].x;
                    graph[graph.size()-1].y = graph[graph.size()-2].y + width;
                    newMovement = !checkIfVisited(graph); 
                    break; 

            }
    
            // next node
            nextPosition[0] = graph[graph.size()-1].x;
            nextPosition[1] = graph[graph.size()-1].y;

        } 

        if(startupComplete){
            if(navigation.moveTo(nextPosition)){
                positionReached = true; 
            }
        }

        ros::spinOnce();
        
    }  

return 0; 
}
