#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// Define pick-up and drop-off
double pick_up[2] = {2.0, -2.0}; // odom: x = 0.0
double drop_off[2] = {2.0, 2.0}; // odom: x = 0.0
double pose[2] = {0, 0};

void get_pose(const nav_msgs::Odometry::ConstPtr &msg) {
    pose[0] = msg->pose.pose.position.x;
    pose[1] = msg->pose.pose.position.y;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, get_pose);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int state = 0;

  while (ros::ok())
  {
    ROS_INFO("%f, %f", pose[0], pose[1]);

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Marker Parameters
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.5f;
    marker.color.g = 0.0f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ros::spinOnce();

    if (state == 0) {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pick_up[0];
        marker.pose.position.y = pick_up[1];
        if (abs(pose[0]-pick_up[0])-2 < 0.1 && abs(pose[1]-pick_up[1]) < 0.1) {
            state = 2;
        }
    } else if (state == 1) {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = drop_off[0];
        marker.pose.position.y = drop_off[1];
    } else if (state == 2) {
        marker.action = visualization_msgs::Marker::DELETE;
        if (abs(pose[0]-drop_off[0])-2 < 0.1 && abs(pose[1]-drop_off[1]) < 0.1) {
            state = 1;
        }
    }


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

    marker_pub.publish(marker);

  }
}