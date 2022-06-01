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
float min_diff = 100.0;
float near_x = 0.0;
float near_y = 0.0;
float min_distance = 0.2;
bool is_picked = false;
bool is_dropped = false;

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

void follow_and_hide(const geometry_msgs::PoseWithCovarianceStamped pose) 
{
  //ROS_INFO("follow and hide");

  float diff = std::abs(pose.pose.pose.position.x - marker.pose.position.x) + std::abs(pose.pose.pose.position.y - marker.pose.position.y);
  if (diff < min_diff) {
    min_diff = diff;
    near_x = pose.pose.pose.position.x;
    near_y = pose.pose.pose.position.y;
  }

  std::cout << "min_idff " << min_diff << " " << near_x << " " << near_y << std::endl;
  std::cout << "diff " << diff << std::endl;
  std::cout << "robot position: " << pose.pose.pose.position << std::endl;
  std::cout << "marker position: " << marker.pose.position << std::endl;
  std::cout << "picked and dropped: " << is_picked << " " << is_dropped << std::endl;

  bool is_close_to_robot = std::abs(pose.pose.pose.position.x - marker.pose.position.x) < min_distance && std::abs(pose.pose.pose.position.y - marker.pose.position.y) < min_distance;
  bool is_close_to_pickup_zone = std::abs(pose.pose.pose.position.x - pick_up_position_x) < min_distance && std::abs(pose.pose.pose.position.y - pick_up_position_y) < min_distance; 
  bool is_close_to_dropoff_zone = std::abs(pose.pose.pose.position.x - drop_off_position_x) < min_distance && std::abs(pose.pose.pose.position.y - drop_off_position_y) < min_distance; 

  if (!is_picked && is_close_to_robot) {
    std::cout << "picked" << std::endl;
    is_picked = true;
    viz_marker(
      pose.pose.pose.position.x, 
      pose.pose.pose.position.y,
      marker.pose.orientation.z,
      marker.pose.orientation.w, 
      0.0);
  }
  else if(is_dropped || (is_picked && is_close_to_dropoff_zone)) 
  {
    std::cout << "dropped" << std::endl;
    is_dropped = true;
    viz_marker(
      drop_off_position_x, drop_off_position_y, 
      drop_off_orientation_z, drop_off_orientation_w, size);

  }

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


  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  viz_marker(
    pick_up_position_x, pick_up_position_y, 
    pick_up_orientation_z, pick_up_orientation_w, size);

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }


  ros::Subscriber sub = n.subscribe("/amcl_pose", 10, follow_and_hide);

  ros::spin();

}
