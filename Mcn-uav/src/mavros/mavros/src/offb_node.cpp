#include "offb.h"
#include "aruco_mapping.h"

bool yaw_bool=true;
int kp=100;
float RotZ;
double init_yaw;
cv::Point3d marker;
cv::Point3d marker_error;

aruco_mapping::Num marker_exist;
void camera_pose(const aruco_mapping::Num::ConstPtr& msg){
    marker_exist=*msg;
}
using namespace std;
using namespace cv;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::Imu imu_state;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imu_state = *msg;
}

sensor_msgs::NavSatFix current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_gps = *msg;
}

void control_bar()
{
    //Create trackbars in "Control" window
    cvCreateTrackbar("gain_KP", "Control", &kp, 1000); //gain_K (0.00 - 1.00)
}

void draw_map(double circle_x,double circle_y,Mat desire_point,Mat1f velocity,float camera_rot)
{

    int  path_flag=0;
    vector<double>maker_array;

    //==============World coordinate =============//

    Mat map = cv::Mat::zeros(500, 500, CV_8UC3); //5M*5M

    circle(map, cv::Point(map.cols/2, map.rows/2), 5, CV_RGB(0,255,0),-1); //draw origin point
    putText(map, "origin point", Point(map.cols/2,map.rows/2+30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,200,200),1);

    circle(map,cv::Point(circle_x*100.0+map.cols/2, -circle_y*100.0 + map.rows/2) , 5, CV_RGB(255,0,0),-1); // curent position
    circle(map,cv::Point(desire_point.at<float>(0,path_flag)*100.0+map.cols/2, -desire_point.at<float>(1,path_flag)*100.0 + map.rows/2) , 5, CV_RGB(255,255,0),-1);// desired position
    putText(map, "desired point", cv::Point(desire_point.at<float>(0,path_flag)*100.0+map.cols/2, -desire_point.at<float>(1,path_flag)*100.0 + map.rows/2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0),1);

    rectangle(map, Point(desire_point.at<float>(0,path_flag)*100.0+map.cols/2-20.0, - desire_point.at<float>(1,path_flag)*100.0 + map.rows/2-20.0) ,
              Point(desire_point.at<float>(0,path_flag)*100.0+map.cols/2+20.0, - desire_point.at<float>(1,path_flag)*100.0 + map.rows/2+20.0) , CV_RGB(4,255,0), 1);

    // cout<<"velocity  X" <<velocity.at<float>(0,0)<<"    velocity  Y"<<velocity.at<float>(1,0)<<endl;

    //    line(map,cv::Point(circle_x*100.0+map.cols/2, -circle_y*100.0 + map.rows/2),
    //         Point (velocity.at<float>(0,0)*10000.0+circle_x*100.0+map.cols/2,-velocity.at<float>(1,0)*10000.0-circle_y*100.0 + map.rows/2) ,
    //         Scalar(200,200,200),5);   //uav velocity direction

    Mat1f init_uav_head(2,1),body_frame_dir(2, 1),rotation_matrix(2, 2);
    init_uav_head <<0.0,-50.0;
    rotation_matrix << cos(-camera_rot),-sin(-camera_rot),sin(-camera_rot),cos(-camera_rot);
    body_frame_dir=rotation_matrix*init_uav_head;

    //    line(map,cv::Point(circle_x*100.0+map.cols/2, -circle_y*100.0 + map.rows/2),
    //         Point (body_frame_dir.at<float>(0,0)+circle_x*100.0+map.cols/2,body_frame_dir.at<float>(1,0)-circle_y*100.0 + map.rows/2) ,
    //         Scalar(0,0,255),5);  //uav body frame direction

    imshow("map", map);

    //==============Camera cooridnate================//

    Mat local_coordinate = cv::Mat::zeros(500, 500, CV_8UC3); //5M*5M

    circle(local_coordinate, cv::Point(local_coordinate.cols/2, local_coordinate.rows/2), 5, CV_RGB(0,255,0),-1); //draw origin point
    putText(local_coordinate, "uav mass", Point(local_coordinate.cols/2,local_coordinate.rows/2+30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,200,200),1);

    Mat1f desired_frame_local(2, 1);
    desired_frame_local<<desire_point.at<float>(0,path_flag)-circle_x,desire_point.at<float>(1,path_flag)-circle_y;

    rotation_matrix << cos(-camera_rot),-sin(-camera_rot),sin(-camera_rot),cos(-camera_rot); // world frame to  body ＝local frame
    desired_frame_local=rotation_matrix*desired_frame_local;


    circle(local_coordinate,cv::Point(desired_frame_local.at<float>(0,0)*100.0+local_coordinate.cols/2, - desired_frame_local.at<float>(1,0)*100.0 + local_coordinate.rows/2) , 5, CV_RGB(255,255,0),-1);// desired position
    putText(local_coordinate, "desired point", cv::Point(desired_frame_local.at<float>(0,path_flag)*100.0+local_coordinate.cols/2, -desired_frame_local.at<float>(1,path_flag)*100.0 + local_coordinate.rows/2),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0),1);


    // cout<<"velocity  X" <<velocity.at<float>(0,0)<<"    velocity  Y"<<velocity.at<float>(1,0)<<endl;

    line(local_coordinate,cv::Point(local_coordinate.cols/2,  local_coordinate.rows/2),
         Point (velocity.at<float>(0,0)*1000.0+map.cols/2,-velocity.at<float>(1,0)*1000.0+ map.rows/2) ,
         Scalar(200,200,200),5);   //uav velocity direction

    line(local_coordinate,cv::Point(local_coordinate.cols/2, local_coordinate.rows/2),
         Point (body_frame_dir.at<float>(0,0)+local_coordinate.cols/2,body_frame_dir.at<float>(1,0)+ local_coordinate.rows/2) ,
         Scalar(0,0,255),5);  //uav body frame direction

    imshow("Local map", local_coordinate);

    //==========wave form======//

    Mat waveform = cv::Mat::zeros(500, 500, CV_8UC3); //draw PID wave

    maker_array.push_back(marker_error.y*100.0 + waveform.rows/2);

    if(maker_array.size()>50)
    {
        maker_array.erase(maker_array.begin());

        line(waveform,Point(0,waveform.rows/2),
             Point (500,waveform.rows/2) ,
             Scalar(0,255,255),5);  //desired position

        for(int i=0;i<(maker_array.size()-1);i++)
        {
            line(waveform,Point(i*10,maker_array[i]),
                 Point ((i+1)*10,maker_array[i+1]) ,
                    Scalar(0,0,255),5);  //error wave
        }
    }

    //     cout<<"maker draw=  "<<maker_array<<endl;

    imshow("PID waveform map", waveform);

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
    msg_override.channels[2]=1650;
    return msg_override;
}
mavros_msgs::OverrideRCIn down(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[2]= 1350;
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
mavros_msgs::OverrideRCIn turn_left(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[3]= 1400;
    return msg_override;
}
mavros_msgs::OverrideRCIn turn_right(mavros_msgs::OverrideRCIn msg_override)
{
    msg_override.channels[3]= 1600;
    return msg_override;
}

void RC_control( ros::Publisher rc_pub,mavros_msgs::OverrideRCIn msg_override,mavros_msgs::SetMode offb_set_mode, ros::ServiceClient set_mode_client,ros::Rate rate)
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
        //
        if (x == 'w')msg_override=up       (msg_override);
        else if (x == 's')msg_override=down     (msg_override);
        else if (x == 'z')msg_override=zero     (msg_override);

        else if (x == 'i')msg_override=forward  (msg_override);
        else if (x == 'k')msg_override=backward (msg_override);
        else if (x == 'j')msg_override=left     (msg_override);
        else if (x == 'l')msg_override=right    (msg_override);
        else if (x == 'u')msg_override=turn_left     (msg_override);
        else if (x == 'o')msg_override=turn_right     (msg_override);
        else if (x == 'q'){ROS_INFO("Break RC mode");break;}
        else if (x == 'e')msg_override=kill    (msg_override);
        else    msg_override=stay     (msg_override);
        rc_pub.publish(msg_override);

        ros::spinOnce();
        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ROS_INFO("START");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //Create a OFFB object
    OFFB offb;
    tf::TransformListener listener;

    ros::Subscriber camera_sub = nh.subscribe<aruco_mapping::Num>
            ("marker", 1,  &camera_pose);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, imu_cb);
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

    namedWindow("Control",WINDOW_NORMAL); //create a window called "Control"
    control_bar();

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

    double imu_roll, imu_pitch, imu_yaw;

    while(ros::ok())
    {
        tf::StampedTransform transform;

        tf::Quaternion q(imu_state.orientation.x, imu_state.orientation.y, imu_state.orientation.z,imu_state.orientation.w);
        tf::Matrix3x3 m(q);

      m.getRPY(imu_roll, imu_pitch, imu_yaw);

         if(yaw_bool)
        init_yaw=get_orientation(imu_state, yaw_bool,imu_roll,imu_pitch,imu_yaw);

        //ROS_INFO_STREAM("init_yaw= "<<init_yaw);
        //ROS_INFO_STREAM("yaw_bool= "<<yaw_bool);
        //ROS_INFO_STREAM("current yaw= "<<imu_yaw);
        ROS_INFO_STREAM("correct yaw= "<<imu_yaw-init_yaw);

        int c = getch();
        if (c == 'r')
        {
            RC_control(rc_pub, msg_override, offb_set_mode, set_mode_client,rate);
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
            ROS_INFO("RC override");
            msg_override.channels[0]= 1500;
            msg_override.channels[1]= 1500;
            msg_override.channels[2]= 1500;
            msg_override.channels[3]= 1500;
            msg_override.channels[4]= 1100;
            rc_pub.publish(msg_override);
            offb_set_mode.request.custom_mode = "POSCTL";
            if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.success)ROS_INFO("pose control enabled");

            while(1){
                int x = getch();
                if(x == 't')
                {
                    float x=0;
                    float y=0;
                    double vx=0;
                    double vy=0;

                    ROS_INFO("tacking start");
                    rc_pub.publish(msg_override);
                    while(1)
                    {
                        x = getch();
                        waitKey(30);

                        try{
                            listener.lookupTransform("/world", "/camera_position",ros::Time(0), transform);// local coordinate diff

                            tf::Vector3 vector_MKpose = transform.getOrigin();

                            marker.x=vector_MKpose.getX();
                            marker.y=vector_MKpose.getY();
                            marker.z=vector_MKpose.getZ();

                            double yaw, pitch, roll;
                            transform.getBasis().getRPY(roll, pitch, yaw);
                            RotZ=yaw; //camera rotation to yaw

                            if(yaw>0)
                                RotZ=yaw;
                            else
                                RotZ=yaw+6.28;

                            //std::cout << "- Translation: [" <<marker.x<< ", " <<marker.y<< ", " << marker.z<< "]" << std::endl<<"\n \n \n";
                            //                   std::cout << "Rotation: in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                            //              << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
                        }
                        catch (tf::TransformException ex){
                            ROS_ERROR("%s",ex.what());
                            ros::Duration(0.25).sleep();
                        }

                        if(marker_exist.marker_visibile == true)
                        {
                            if(RotZ>6.19 || RotZ<0.09)
                            {
                                ROS_INFO_STREAM("heading center");
                            }
                            else if (RotZ<3.14 && RotZ > 0.09)
                            {
                                ROS_INFO_STREAM("turn right");
                            }

                            else if (RotZ>3.14 && RotZ <6.19)
                            {
                                ROS_INFO_STREAM("turn left");
                            }

                            marker_error=offb.PIDcontrol(0,0,marker.x,marker.y,vx,vy,kp);
                            if  (vx>100)
                                vx=100;
                            else if (vx<-100)
                                vx=-100;

                            if  (vy>100)
                                vy=100;
                            else if (vy<-100)
                                vy=-100;


                            if(marker_error.x > 0.30 || marker_error.x< -0.30)
                            {
                                msg_override.channels[1] = 1500 + vx;  //左右
                            }

                            else
                            {
                                //ROS_INFO_STREAM( "center" );
                                msg_override.channels[1] = 1500;  //前後
                            }

                            if(marker_error.y > 0.30 || marker_error.y< -0.30)
                            {
                                msg_override.channels[0] = 1500 + vy;  //前小後大

                            }

                            else
                            {
                                //ROS_INFO_STREAM( "center" );
                                msg_override.channels[0] = 1500;  //前後
                            }

                            //cout <<"\n  linear velocity x&y :  " <<msg_override.channels[0] <<"   ,   "<< msg_override.channels[1]<<endl;

                        }

                        else
                        {
                            //ROS_INFO_STREAM( "marker lose" );
                            msg_override.channels[1] = 1500;  //前後
                            msg_override.channels[0] = 1500;  //左右
                        }

                        rc_pub.publish(msg_override);

                        if (x == 'q'){ROS_INFO("Break tracking mode"); break;}

                        ros::spinOnce();
                        rate.sleep();
                    }

                }
                else if (x == 'r')RC_control(rc_pub, msg_override, offb_set_mode, set_mode_client,rate);
                else if (x == 'q'){ ROS_INFO("pose control break");break;}
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



