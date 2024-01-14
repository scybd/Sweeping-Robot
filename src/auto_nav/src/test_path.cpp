#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  // 初始化 ROS 节点
  ros::init(argc, argv, "test_path");
  ros::NodeHandle n;

  // 创建一个路径发布者
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("plan_path", 10);

  // 创建路径消息
  nav_msgs::Path path;
  path.header.frame_id = "map"; // 或者使用你使用的坐标系
  path.header.stamp = ros::Time::now();

  // 添加路径点
  for (int i = 0; i < 10; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    pose.pose.position.x = i;  // 示例坐标
    pose.pose.position.y = i;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }

  // 发布路径
  while (ros::ok()) {
    path_pub.publish(path);
    ros::spinOnce();
  }

  return 0;
}
