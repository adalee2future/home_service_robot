#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

float start_x = 2.89;
float start_y = 0.09;
float end_x = 3.00;
float end_y = 2.68;
float size = 0.25;

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = start_x;
  marker.pose.position.y = start_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.95;
  marker.pose.orientation.w = 0.32;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  
  std::cout << "Publish box start position" << std::endl;
  marker_pub.publish(marker);

  std::cout << "Sleep 5 seconds" << std::endl;
  sleep(5);

  std::cout << "Hide the box" << std::endl;
  marker.scale.x = 0.0;
  marker_pub.publish(marker);

  std::cout << "Sleep 5 seconds" << std::endl;
  sleep(5);

  std::cout << "Publish box end position" << std::endl;
  marker.scale.x = size;
  marker.pose.position.x = end_x;
  marker.pose.position.y = end_y;
  marker.pose.orientation.z = -0.45;
  marker.pose.orientation.w = 0.89;
  marker_pub.publish(marker);

}
