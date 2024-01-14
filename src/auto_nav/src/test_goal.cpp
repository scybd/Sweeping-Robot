#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // 初始化 ROS 节点
  ros::init(argc, argv, "test_goal");

  // 告诉 action 客户端，我们想要连接到 move_base 服务器
  MoveBaseClient ac("move_base", true);

  // 等待 move_base action 服务器启动
  ROS_INFO("Waiting for the move_base action server");
  ac.waitForServer();

  // 设置目标
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // 定义目标位置和方向
  goal.target_pose.pose.position.x = 0.0; // 示例坐标
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // 等待目标完成
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base moved to the goal position");
  else
    ROS_INFO("The base failed to move to the goal position");

  return 0;
}
