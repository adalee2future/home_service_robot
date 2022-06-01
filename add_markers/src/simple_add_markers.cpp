#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

float pick_up_position_x;
float pick_up_position_y;
float pick_up_orientation_z;
float pick_up_orientation_w;
float drop_off_position_x;
float drop_off_position_y;
float drop_off_orientation_z;
float drop_off_orientation_w;

float size = 0.25;

visualization_msgs::Marker marker;
ros::Publisher marker_pub;


void viz_marker(float x, float y, float oz, float ow, float s) {
   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.orientation.z = oz;
  marker.pose.orientation.w = ow;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = s;
  marker.scale.y = s;
  marker.scale.z = s;

  marker_pub.publish(marker);
 }



int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Read pick up and drop off zones
  n.getParam("/pick_up/position_x", pick_up_position_x);
  n.getParam("/pick_up/position_y", pick_up_position_y);
  n.getParam("/pick_up/orientation_z", pick_up_orientation_z);
  n.getParam("/pick_up/orientation_w", pick_up_orientation_w);
  n.getParam("/drop_off/position_x", drop_off_position_x);
  n.getParam("/drop_off/position_y", drop_off_position_y);
  n.getParam("/drop_off/orientation_z", drop_off_orientation_z);
  n.getParam("/drop_off/orientation_w", drop_off_orientation_w);



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

  marker.lifetime = ros::Duration();


  std::cout << "Publish the marker at the pickup zone" << std::endl;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  viz_marker(
    pick_up_position_x, pick_up_position_y,
    pick_up_orientation_z, pick_up_orientation_w, size);


  std::cout << "Pause 5 seconds" << std::endl;
  sleep(5);


  std::cout << "Hide the marker" << std::endl;
  viz_marker(
    pick_up_position_x, pick_up_position_y,
    pick_up_orientation_z, pick_up_orientation_w, 0.0);


  std::cout << "Pause 5 seconds" << std::endl;
  sleep(5);
 

  std::cout << "Publish the marker at the drop off zone" << std::endl;
  viz_marker(
    drop_off_position_x, drop_off_position_y,
    drop_off_orientation_z, drop_off_orientation_w, size);

}
