/*

note: visual studio code

*/


#include "ros/ros.h"
#include "opencv2/core.hpp"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs//PoseWithCovarianceStamped.h"
#include <vector>
#include <iterator>
#include <math.h>
#include <numeric>                                                              

using namespace std;

#define PI 3.14159265359 

#define Mx 4
#define My -2

//// KF ////
cv::Mat_<float> A           = cv::Mat::eye(3,3,CV_64FC1);
cv::Mat_<float> B           = cv::Mat::zeros(3,2,CV_64FC1);
cv::Mat_<float> u_t         = cv::Mat::zeros(2,1,CV_64FC1);
cv::Mat_<float> my_t_pred   = cv::Mat::zeros(3,1,CV_64FC1);
cv::Mat_<float> my_tm1      = cv::Mat::zeros(3,1,CV_64FC1);
cv::Mat_<float> my_t        = cv::Mat::zeros(3,1,CV_64FC1);
cv::Mat_<float> sig_t_pred  = cv::Mat::zeros(3,3,CV_64FC1);
cv::Mat_<float> sig_tm1     = cv::Mat::zeros(3,3,CV_64FC1);
cv::Mat_<float> sig_t       = cv::Mat::zeros(3,3,CV_64FC1);
cv::Mat_<float> R           = cv::Mat::zeros(3,3,CV_64FC1);

//// EKF ///
cv::Mat_<float> z_t         = cv::Mat::zeros(3,1,CV_64FC1);
cv::Mat_<float> z_t_hat     = cv::Mat::zeros(3,1,CV_64FC1);
cv::Mat_<float> delta_z_t   = cv::Mat::zeros(3,1,CV_64FC1);
cv::Mat_<float> H_t         = cv::Mat::zeros(3,3,CV_64FC1); 
cv::Mat_<float> delta       = cv::Mat::zeros(2,1,CV_64FC1);
cv::Mat_<float> K_t         = cv::Mat::zeros(3,3,CV_64FC1);
cv::Mat_<float> Q_t         = cv::Mat::eye(3,3,CV_64FC1);
cv::Mat_<float> I           = cv::Mat::eye(3,3,CV_64FC1);
cv::Mat_<float> q           = cv::Mat::eye(1,1,CV_64FC1);


bool printOdometry = 0; 
bool inRange = 0; 
float dt = 0.025;
int correctionCounter = 0; 

ros::Publisher  publisherKFPrediction;
ros::Publisher  publisherEKFLocalization;
ros::Time       oldTime, newTime;

struct Quaternion
{
    double w, x, y, z;
};

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

void publisherFunction(cv::Mat_<float> my, cv::Mat_<float> sigma, string publishertopic){

    Quaternion my_t_Q;
    geometry_msgs::PoseWithCovarianceStamped prediction;  
    geometry_msgs::PoseWithCovarianceStamped correction;  

    my_t_Q = ToQuaternion(my[2][0],0,0);

    prediction.header.frame_id="odom";
    prediction.pose.pose.position.x = my[0][0];  
    prediction.pose.pose.position.y = my[1][0]; 
    prediction.pose.pose.position.z = 0;   

    prediction.pose.pose.orientation.x = my_t_Q.x;
    prediction.pose.pose.orientation.y = my_t_Q.y;
    prediction.pose.pose.orientation.z = my_t_Q.z;
    prediction.pose.pose.orientation.w = my_t_Q.w;

    prediction.pose.covariance[0] = sigma[0][0];
    prediction.pose.covariance[1] = sigma[0][1];
    prediction.pose.covariance[5] = sigma[0][2];
    prediction.pose.covariance[6] = sigma[1][0];
    prediction.pose.covariance[7] = sigma[1][1];
    prediction.pose.covariance[11] = sigma[1][2];
    prediction.pose.covariance[30] = sigma[2][0];
    prediction.pose.covariance[31] = sigma[2][1];
    prediction.pose.covariance[35] = sigma[2][2];
    
    if(publishertopic == "prediction")
        publisherKFPrediction.publish(prediction);
    else if(publishertopic == "correction")
        publisherEKFLocalization.publish(prediction);

}

////////////////////////////////  predictionFunction  ////////////////////////////////
void predictionFunction()
{

    cout << endl << "---- PREDICTION ----" << endl;

    B[0][0] = dt*cos(my_tm1[2][0]);
    B[1][0] = dt*sin(my_tm1[2][0]);
    B[2][1] = dt;

    my_t_pred = (A*my_tm1)+(B*u_t);
    while(my_t_pred[2][0] > 2*PI) 
        my_t_pred[2][0] -= 2*PI;

    while(my_t_pred[2][0] < 0) 
        my_t_pred[2][0] += 2*PI;

    cout << "my_t_pred:\t" << my_t_pred[0][0] << ", " << my_t_pred[1][0] << ", " << my_t_pred[2][0] << endl;

    R[0][0]=0.00001; R[1][1]=0.00001; R[2][2]=0.001;
    sig_t_pred = A*sig_tm1*A.t()+R;

    publisherFunction(my_t_pred, sig_t_pred, "prediction");

    my_tm1 = my_t_pred;
    sig_tm1 = sig_t_pred; 

}


////////////////////////////////  localizationFunction  ////////////////////////////////
void localizationFunction()
{
    
    cout << endl << "---- LOCALIZATION ----" << endl;
    cout << "inRange: " << inRange << endl;

    if(inRange){
        correctionCounter ++;

                cout << "z_t:\t\t" << z_t[0][0] << ", " << z_t[1][0] << endl;

        delta[0][0] = Mx - my_t_pred[0][0];
        delta[1][0] = My - my_t_pred[1][0];

                cout << "delta:\t\t" << delta[0][0] << ", " << delta[1][0] << endl;
       
        q = ( delta.t() )*delta;

                cout << "q:\t\t" << q[0][0] << endl;  

        z_t_hat[0][0] = sqrt(q[0][0]);

        float theta4z_t_hat = 0;
        if(my_t_pred[2][0] > PI)
            theta4z_t_hat = my_t_pred[2][0] - (2*PI) ;
        else 
            theta4z_t_hat = my_t_pred[2][0];

        z_t_hat[1][0] = atan2(delta[1][0], delta[0][0]) - theta4z_t_hat;
        if(z_t_hat[1][0] < (-PI) )
            z_t_hat[1][0] += 2*PI;
        if(z_t_hat[1][0] >= (PI) ) 
            z_t_hat[1][0] -= 2*PI;   

        z_t_hat[2][0] = 1; 

        cout << "z_t_hat:\t" << z_t_hat[0][0] << ", "<< z_t_hat[1][0] << ", "<< z_t_hat[2][0] << endl;
  
        // H_t
        H_t[0][0] = -sqrt(q[0][0]) * delta[0][0];
        H_t[0][1] = -sqrt(q[0][0]) * delta[1][0];
        H_t[1][0] = delta[1][0];
        H_t[1][1] = -delta[0][0];
        H_t[1][2] = -1;

        H_t = H_t * (1/q[0][0]);


        cout << "H_t:\t\t" << H_t[0][0] << ", " << H_t[0][1] << ", " << H_t[0][2] << endl;
        cout << "\t\t"   << H_t[1][0] << ", " << H_t[1][1] << ", " << H_t[1][2] << endl;
        cout << "\t\t"   << H_t[2][0] << ", " << H_t[2][1] << ", " << H_t[2][2] << endl;

        K_t = (sig_t_pred * (H_t.t())) * ((H_t * sig_t_pred * H_t.t() + Q_t).inv());

        cout << "K_t:\t\t" << K_t[0][0] << ", " << K_t[0][1] << ", " << K_t[0][2] << endl;
        cout << "\t\t"   << K_t[1][0] << ", " << K_t[1][1] << ", " << K_t[1][2] << endl;
        cout << "\t\t"   << K_t[2][0] << ", " << K_t[2][1] << ", " << K_t[2][2] << endl;
       
        delta_z_t = z_t - z_t_hat;
            if(z_t_hat[1][0] < (-PI) )
                z_t_hat[1][0] += 2*PI;
            if(z_t_hat[1][0] >= (PI) ) 
                z_t_hat[1][0] -= 2*PI;

        my_t = my_t_pred + K_t * (delta_z_t);
        cout << "my_t:\t\t" << my_t[0][0] << ", " << my_t[1][0] << ", " << my_t[2][0] << endl;

        sig_t = (I - K_t * H_t) * sig_t_pred;
        cout << "sig_t:\t\t" << sig_t[0][0] << ", " << sig_t[0][1] << ", " << sig_t[0][2] << endl;
        cout << "\t\t"   << sig_t[1][0] << ", " << sig_t[1][1] << ", " << sig_t[1][2] << endl;
        cout << "\t\t"   << sig_t[2][0] << ", " << sig_t[2][1] << ", " << sig_t[2][2] << endl;

        publisherFunction(my_t, sig_t, "correction");
        my_tm1 = my_t;
        sig_tm1 = sig_t;

        //cin.get();    
    }

}

////////////////////////////////  callbackOdometry  ////////////////////////////////
void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odometry)
//void callbackOdometry(const geometry_msgs::Twist::ConstPtr& odometry)
{
    u_t[0][0] = odometry -> twist.twist.linear.x; 
    u_t[1][0] = odometry -> twist.twist.angular.z;
    //u_t[0][0] = odometry -> linear.x; 
    //u_t[1][0] = odometry -> angular.z;
}

////////////////////////////////  callbackLiDAR  ////////////////////////////////
void callbackLiDAR(const sensor_msgs::LaserScan::ConstPtr& LiDAR)
{
    vector<int>   angle;
    vector<float> distance; 

    inRange = 0; 
  
    for(int i = 0; i < 360; i++){
        if( (LiDAR->ranges[i] > 0.1) && (LiDAR->ranges[i] < 3.5) ){
            inRange = 1; 
            distance.push_back(LiDAR->ranges[i]);
            if( i >= 180 )
                angle.push_back(i-360);
            else               
                angle.push_back(i);
        }
    }

    if(inRange){
        float       meanAngle = 0;
        float       meanDistance = 0;    

        meanAngle = accumulate(angle.begin(), angle.end(), 0.0)/angle.size();
        meanDistance = accumulate(distance.begin(), distance.end(), 0.0)/distance.size();

    //    if(meanAngle > 360)
    //        meanAngle -= 360;
        float meanAngleRad = (float)meanAngle*PI/180; 

        z_t[0][0] = meanDistance; 
        z_t[1][0] = meanAngleRad;
        z_t[2][0] = 1;
    } 
}


int main(int argc, char **argv ) {

    ros::init(argc, argv, "hoerbstnode");
    ros::NodeHandle nh("~"); 
    
    cout << "hoerbstprogramm" << endl;
/*
    printOdometry = 1; 

    newTime = ros::Time::now();

    ros::Subscriber subscriberOdometry; 
    ros::Subscriber subscriberLiDAR;
    
    //subscriberOdometry  = nh.subscribe("cmd_vel", 10, callbackOdometry);
    subscriberOdometry  = nh.subscribe("odom", 100, callbackOdometry);
    subscriberLiDAR     = nh.subscribe("scan", 100, callbackLiDAR);

    publisherKFPrediction = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("KFPrediction", 100);
    publisherEKFLocalization = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("EKFLocalization", 100);

    my_tm1[0][0] = 0; 
    my_tm1[1][0] = 0;
    my_tm1[2][0] = 0;
    
    Q_t = Q_t*(3*pow(10,-2));   //jakobnode 10⁻6 lt. Angabe

    while(ros::ok())
    {

        predictionFunction();
        localizationFunction();

        ros::spinOnce();
        ros::Duration(dt).sleep();
        
    }    
*/
return 0; 
}
