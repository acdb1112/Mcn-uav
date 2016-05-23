#include "offb.h"

void OFFB::PIDcontrol(float x,float y,int rows, int cols, double &vx,double &vy,double kp){
    // PID設置

    // 誤差 (圖像座標與本身座標轉換)
    double error_x = (rows / 2 - y);
    double error_y = (cols / 2 - x);
    // 操作量
    vx = kp * error_x ;
    vy = kp * error_y ;

    return;
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


