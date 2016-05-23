#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  ros::Duration(0.5).sleep(); //let ros time initial

  ROS_INFO("Starting ");

  while (node.ok()){

    tf::StampedTransform transform;
    try{
     listener.lookupTransform("/board1", "camera1",
                               ros::Time(0), transform);// local coordinate diff

   double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);
   tf::Vector3 v = transform.getOrigin();
    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
      std::cout << "Rotation: in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
    << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;


    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

 //  ROS_INFO_STREAM("\n pose_x: " << transform.getOrigin().x() <<"\n pose_y: "<< transform.getOrigin().y()<<"\n pose_z: " << transform.getOrigin().z());

    rate.sleep();
    ros::Duration(0.25).sleep();
    ros::spinOnce();

  }
  return 0;
};
