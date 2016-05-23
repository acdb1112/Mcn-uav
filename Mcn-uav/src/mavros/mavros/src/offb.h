#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

 #include <termios.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/TwistStamped.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/opencv.hpp"
#include <cv.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>


 struct OFFB{
    public:

	cv::Point3d PIDcontrol(float des_x,float des_y,float x,float y, double &vx,double &vy,double kp);
       	void twist_stay(geometry_msgs::TwistStamped &twist);
        void twist_up(geometry_msgs::TwistStamped &twist);
        void twist_down(geometry_msgs::TwistStamped &twist);
        void twist_forward(geometry_msgs::TwistStamped &twist);
        void twist_backward(geometry_msgs::TwistStamped &twist);
        void twist_left(geometry_msgs::TwistStamped &twist);
        void twist_right(geometry_msgs::TwistStamped &twist);

    };

double get_orientation(sensor_msgs::Imu,bool &,double,double,double);
