#include "offb.h"

 cv::Point3d OFFB::PIDcontrol(float des_x,float des_y,float x,float y, double &vx,double &vy,double kp){
    // PID設置

    // 誤差 (圖像座標與本身座標轉換)
    double error_x = (des_x- x);
    double error_y = (des_y- y);
    // 操作量
    vx = kp * error_x;
    vy = -kp * error_y ;

    cv::Point3d marker_error;
    marker_error.x=error_x;
    marker_error.y=error_y;
    return marker_error;
}

void OFFB::twist_stay     (geometry_msgs::TwistStamped &twist)
{
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    return ;
}
void OFFB::twist_up       (geometry_msgs::TwistStamped &twist)
{
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 1;
    return;
}
void OFFB::twist_down     (geometry_msgs::TwistStamped &twist)
{
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = -1;
    return;
}
void OFFB::twist_forward  (geometry_msgs::TwistStamped &twist)
{
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 1;
    twist.twist.linear.z = 0;
    return;
}
void OFFB::twist_backward (geometry_msgs::TwistStamped &twist)
{
    twist.twist.linear.x = 0;
    twist.twist.linear.y = -1;
    twist.twist.linear.z = 0;
    return;
}
void OFFB::twist_left     (geometry_msgs::TwistStamped &twist)
{
    twist.twist.linear.x = -1;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    return;
}
void OFFB::twist_right    (geometry_msgs::TwistStamped &twist)
{
    twist.twist.linear.x = 1;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    return;
}

double get_orientation(sensor_msgs::Imu orien_imu_state,bool &orien_yaw_bool,double orien_roll,double orien_pitch,double orien_yaw)
{
   double orien_init_yaw;

    if(orien_yaw_bool)
    {
        if(orien_yaw != orien_yaw) //if yaw eqal NAN
        {
            tf::Quaternion q(orien_imu_state.orientation.x, orien_imu_state.orientation.y, orien_imu_state.orientation.z,orien_imu_state.orientation.w); tf::Matrix3x3 m(q);
            m.getRPY(orien_roll, orien_pitch, orien_yaw);
            orien_init_yaw=orien_yaw;
            //ROS_INFO_STREAM("yaw is nan");
        }
        else
        {
            tf::Quaternion q(orien_imu_state.orientation.x, orien_imu_state.orientation.y, orien_imu_state.orientation.z,orien_imu_state.orientation.w); tf::Matrix3x3 m(q);
            m.getRPY(orien_roll, orien_pitch, orien_yaw);
             orien_init_yaw=orien_yaw;
            orien_yaw_bool=false;
            // ROS_INFO_STREAM("yaw not nan");
           //  ROS_INFO_STREAM("orien_init_yaw=  "<<orien_init_yaw);
            return orien_init_yaw;
        }
    }
    //ROS_INFO_STREAM("orien_init_yaw=  "<<orien_init_yaw);
}

