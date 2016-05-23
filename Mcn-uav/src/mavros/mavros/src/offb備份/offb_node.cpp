#include "offb.h"
using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::NavSatFix current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_gps = *msg;
}

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0);
        //ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

mavros_msgs::OverrideRCIn stay(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[0]= 1500;
    msg_override.channels[1]= 1500;
    msg_override.channels[2]= 1500;
    msg_override.channels[3]= 1500;
    msg_override.channels[4]= 1100;
    //msg_override.channels[5]= 1100;
    //msg_override.channels[6]= 1100;
    //msg_override.channels[7]= 1100;
    return msg_override;
}
mavros_msgs::OverrideRCIn forward(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[1]= 1400;
    return msg_override;
}
mavros_msgs::OverrideRCIn backward(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[1]= 1600;
    return msg_override;
}
mavros_msgs::OverrideRCIn left(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[0]= 1400;
    return msg_override;
}
mavros_msgs::OverrideRCIn right(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[0]= 1600;
    return msg_override;
}
mavros_msgs::OverrideRCIn up(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[2]= 1600;
    return msg_override;
}
mavros_msgs::OverrideRCIn down(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[2]= 1400;
    return msg_override;
}
mavros_msgs::OverrideRCIn zero(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[2]= 1100;
    return msg_override;
}
mavros_msgs::OverrideRCIn kill(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[4]= 1600;
    return msg_override;
}


int main(int argc, char **argv)
{
    ROS_INFO("START");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //Create a OFFB object
    OFFB offb;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
//----------------------------mavros_msgs/OverrideRCIn------------------------------------

    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",10);
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff");
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land");

    ros::Publisher vel = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel",10);

    mavros_msgs::OverrideRCIn msg_override;
    msg_override.channels[0]= 1500;
    msg_override.channels[1]= 1500;
    msg_override.channels[2]= 1500;
    msg_override.channels[3]= 1500;
    msg_override.channels[4]= 1100;
    //msg_override.channels[5]= 1100;
    //msg_override.channels[6]= 1100;
   // msg_override.channels[7]= 1100;
       rc_pub.publish(msg_override);   

    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 2;
    //srv_land.request.latitude = 0;
    //srv_land.request.longitude = 0;
    //srv_land.request.min_pitch = 0;
    //srv_land.request.yaw = 0;

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 2;

    cv::namedWindow("OPENCV_WINDOW");

//------------------------------------------------------------------------------------------------------
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    ros::Time last_request = ros::Time::now();
    ros::Time time = ros::Time::now();

while(ros::ok())
{		
	 
    int c = getch();

         if (c == 'r')
         {
              ROS_INFO("RC override");
	     rc_pub.publish(msg_override);  
	     offb_set_mode.request.custom_mode = "POSCTL";
             if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success)
             {
                 ROS_INFO("pose control enabled");
             }
	     rc_pub.publish(msg_override);  
             while(ros::ok())
             {
                 int x = getch();
                 if       (x == ' ')msg_override=stay     (msg_override);
                 else if (x == 'w')msg_override=up       (msg_override);
                 else if (x == 's')msg_override=down     (msg_override);
		 else if (x == 'e')msg_override=zero     (msg_override);

                 else if (x == 'i')msg_override=forward  (msg_override);
                 else if (x == 'k')msg_override=backward (msg_override);
                 else if (x == 'j')msg_override=left     (msg_override);
                 else if (x == 'l')msg_override=right    (msg_override);
                 else if (x == 'q')break;
		 else if (x == 'z')msg_override=kill    (msg_override);
                 rc_pub.publish(msg_override);

                 ros::spinOnce();
                 rate.sleep();
             }
	     offb_set_mode.request.custom_mode = "AUTO.LOITER";
          if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success)ROS_INFO("AUTO.LOITER enabled");
         }

         else if (c == 'o')
         {
             for(int i = 50; ros::ok() && i > 0; --i){
                  offb.twist_stay (twist);
                 vel.publish(twist);
                 ros::spinOnce();
                 rate.sleep();
             }
             offb_set_mode.request.custom_mode = "OFFBOARD";
              if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success) ROS_INFO("Offboard enabled");

             while(ros::ok())
             {
                 int x = getch();
                 if      (x == ' ')      offb.twist_stay     (twist);
                 else if (x == 'w') offb.twist_up       (twist);
                 else if (x == 's')  offb.twist_down     (twist);
                 else if (x == 'a')  offb.twist_stay     (twist);
                 else if (x == 'd')  offb.twist_stay     (twist);
                 else if (x == 'i')   offb.twist_forward  (twist);
                 else if (x == 'k')  offb.twist_backward (twist);
                 else if (x == 'j')   offb.twist_left     (twist);
                 else if (x == 'l')   offb.twist_right    (twist);
                 else if (x == 'q')break;

                 vel.publish(twist);

                 ros::spinOnce();
                 rate.sleep();
             }
 	     
             offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
             if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success) ROS_INFO("AUTO.TAKEOFF enabled");
         }
         else if (c == 'a')
         {
             arm_cmd.request.value = true;
             if( arming_client.call(arm_cmd) &&arm_cmd.response.success)ROS_INFO("Vehicle armed");
         }
         else if (c == 'd')
         {
             arm_cmd.request.value = false;
             if( arming_client.call(arm_cmd) &&arm_cmd.response.success) ROS_INFO("Vehicle disarmed");
         }

         else if (c == 's')
         {
              offb_set_mode.request.custom_mode = "MANUAL";
             if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success) ROS_INFO("MANUAL enabled");
         }

         else if (c == 't')
         {
	     offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
         if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success) ROS_INFO("AUTO.TAKEOFF enabled");
         }
         else if (c == 'l')
         {
             offb_set_mode.request.custom_mode = "AUTO.LAND";
             if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success) ROS_INFO("AUTO.LAND enabled");
         }

	else if (c == 'y')
         {
             while(1){
        ROS_INFO("RC override");
        msg_override.channels[0]= 1500;
        msg_override.channels[1]= 1500;
        msg_override.channels[2]= 1500;
        msg_override.channels[3]= 1500;
        msg_override.channels[4]= 1100;
        rc_pub.publish(msg_override);
        offb_set_mode.request.custom_mode = "POSCTL";
        if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success)ROS_INFO("pose control enabled");
        int x = getch();
        if(x == 't')
        {
		double x=0;
		double y=0;
		double vx=0;
		double vy=0;
		double kp=0.5;
		ROS_INFO("tacking start");
	     rc_pub.publish(msg_override);  
		while(1)
			{

            offb.PIDcontrol(x,y,480,640,vx,vy,kp);
	  		msg_override.channels[1] = 1500 + vx;  //前後
			msg_override.channels[0] = 1500 + vy;  //左右
		
             rc_pub.publish(msg_override);
            if (x == 's')break;
		         ros::spinOnce();
		         rate.sleep();
			}
        }
        }
	     offb_set_mode.request.custom_mode = "AUTO.LOITER";
             if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success)
             {
                 ROS_INFO("AUTO.LOITER enabled");
             }
         }

    ros::spinOnce();
    rate.sleep();
}

    return 0;
}


