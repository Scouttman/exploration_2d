#include "navigation_utils.h"

bool goToDest(point3d go_posi, tf::Quaternion q) {

  // make an action client that spins up a thread
  MoveBaseClient ac("move_base", true);

  // cancel previous goals
  ac.cancelAllGoals();

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = go_posi.x();
  goal.target_pose.pose.position.y = go_posi.y();
  goal.target_pose.pose.position.z = go_posi.z();

  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  ROS_INFO("Sending robot to the viewpoint...");
  ac.sendGoal(goal);

  // while(ros::ok())
    ac.waitForResult(ros::Duration(120.0));

  // Returns true iff we reached the goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
}

// goToDest but returns when withing specified distance so momentum can be kept
bool goToDestEarly(point3d go_posi, tf::Quaternion q, double dis) {
  tf::StampedTransform transform;
  tf_listener_ex = new tf::TransformListener();

  // make an action client that spins up a thread
  MoveBaseClient ac("move_base", true);

  // cancel previous goals
  ac.cancelAllGoals();

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = go_posi.x();
  goal.target_pose.pose.position.y = go_posi.y();
  goal.target_pose.pose.position.z = go_posi.z();

  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  ROS_INFO("Sending robot to the viewpoint...");
  ac.sendGoal(goal);
  bool success = false;

  float cur_dis = 0;
  double begin = ros::Time::now().toSec();
  double last_scan = begin;
  while(ros::ok()){
    try{
      tf_listener_ex->lookupTransform("/map", "/base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: base_link"); 
        ros::Duration(0.05).sleep();
        continue;
    }
    cur_dis = hypot(go_posi.x()-transform.getOrigin().x(),go_posi.y()-transform.getOrigin().y());
    if(cur_dis<dis){
      success = true;
      ROS_INFO("Goal %fm away moving on", cur_dis);
      break;
    }
    if(ros::Time::now().toSec()-begin>120){ // Timeout
      ROS_WARN("Time out moving robot");
      break;
    }
    // take a scan because why not
    //if(ros::Time::now().toSec()-last_scan>4){
    //  ROS_INFO("taking a scan because why not");
    //  ros::spinOnce();
    //  last_scan = ros::Time::now().toSec();
    //}
    ros::Duration(1).sleep();
  }

  return success;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_utils");

  return 0;
}
