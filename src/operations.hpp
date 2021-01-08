/*     
    Maze Solving Algorithm:
    operations 
    Version 1
    Autor: Hoerbst
    Contributor: Gmeiner
 */

#include <math.h>
#include <vector>

using namespace std;

/*! \brief Quaternion Values
 *
 *   
 */
struct Quaternion
{
    double w, x, y, z;
};

/*! \brief Euler Values
 *
 *  
 */
struct EulerAngles
{
    double roll, pitch, yaw;
};

const double PI = 3.14159265359;

/*! \brief Quaternion to Euler Angle transformation
 *
 *  Transformation based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
EulerAngles ToEulerAngles(Quaternion q)
{
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

/*! \brief Euler Angles to Quaternion Transformation
 *
 *  Transformation based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
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

/*! \brief brief descr
 *
 *  Detailed description starts here.
 */
void angleTo360(double &angle)
{
    if(angle < 0) 
        angle = -angle; 
    else
        angle = 360 - angle; 
}

/*! \brief brief descr
 *
 *  Detailed description starts here.
 */
float mapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
