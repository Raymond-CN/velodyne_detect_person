#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
  	
  	broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"world", "home"));
  
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.021, 0.0, 0.33)),
        ros::Time::now(),"base", "laser"));
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 1.17)),
        ros::Time::now(),"base", "rgbd"));
         
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.16, 1.78, 0.873)),
        ros::Time::now(),"world", "velodyne"));
        
    r.sleep();
  }
}
