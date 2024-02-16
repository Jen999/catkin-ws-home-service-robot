#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal(MoveBaseClient& ac, double x, double y, double orient) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orient;

  // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Successfully moved to goal");
  else
    ROS_INFO("The base failed to move to goal for some reason");
}

int main(int argc, char** argv){

  // Define pick-up and drop-off
  double pick_up[2] = {2.0, -2.0}; // odom: x = 0.0
  double drop_off[2] = {2.0, 2.0}; // odom: x = 0.0
  double base[2] = {0.0, 0.0};
  
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending pick-up goal...");
  send_goal(ac, pick_up[0], pick_up[1], 1.0);
  ROS_INFO("Arrived at pick-up [%f, %f]", pick_up[0], pick_up[1]);

  // Wait for 5 seconds
  ros::Duration(5.0).sleep();

  ROS_INFO("Sending drop-off goal...");
  send_goal(ac, drop_off[0], drop_off[1], 1.0);
  ROS_INFO("Arrived at drop-off [%f, %f]", drop_off[0], drop_off[1]);

  ROS_INFO("Returning to base...");
  send_goal(ac, base[0], base[1], 1.0);
  ROS_INFO("Arrived at base [%f, %f]", base[0], base[1]);

  return 0;
}