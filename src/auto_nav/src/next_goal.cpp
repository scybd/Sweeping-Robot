#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace tf;

float x_current;
float y_current;

float normeNextGoal;  // 发送下一个点的阈值

// 欧拉角到四元数的转换，pitch和yaw为0
void eulerAngles2Quaternion(float yaw, float& w, float& x, float& y, float& z)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(0 * 0.5);
    float sr = sin(0 * 0.5);
    float cp = cos(0 * 0.5);
    float sp = sin(0 * 0.5);

    w = cy * cr * cp + sy * sr * sp;
    x = cy * sr * cp - sy * cr * sp;
    y = cy * cr * sp + sy * sr * cp;
    z = sy * cr * cp - cy * sr * sp;
}

class PathPlanner
{
public:
  struct Goal
  {
    float x;
    float y;
    bool visited;
  };

  vector<Goal> Path;

  void addGoal(float X, float Y, bool visit);
};


void PathPlanner::addGoal(float X, float Y, bool visit)
{
  PathPlanner::Goal newGoal;
  newGoal.x = X;
  newGoal.y = Y;
  newGoal.visited = visit;
  Path.push_back(newGoal);
}


PathPlanner pathPlanner;
nav_msgs::Path passedPath;
ros::Publisher pubPassedPath;
void pose_callback(const nav_msgs::Odometry &poses)
{ // 里程计回调函数,用来计算当前机器人位置与前面目标点的距离,判断是否要发新的目标点
  x_current = poses.pose.pose.position.x;
  y_current = poses.pose.pose.position.y;
  passedPath.header = poses.header;
  geometry_msgs::PoseStamped p;
  p.header = poses.header;
  p.pose = poses.pose.pose;
  passedPath.poses.emplace_back(p);
  pubPassedPath.publish(passedPath);
}

int taille_last_path = 0;
bool new_path = false;


// 接受规划的路径
void path_callback(const nav_msgs::Path &path)
{
  // 注意为了rviz显示方便 路径一直在发,但是这里只用接受一次就好,当规划的路径发生变化时候再重新装载,目前没有写在运动过程中重新规划路径的代码，不会进第二次
  if ((pathPlanner.Path.size() == 0) || (path.poses.size() != taille_last_path))
  {
    pathPlanner.Path.clear();
    new_path = true;
    for (int i = 0; i < path.poses.size(); i++)
    {
      pathPlanner.addGoal(path.poses[i].pose.position.x, path.poses[i].pose.position.y, false);
    }
    taille_last_path = path.poses.size();
  }
}


int main(int argc, char *argv[])
{
  srand(time(0));
  ros::init(argc, argv, "next_goal");
  ros::NodeHandle next_goal;
  ros::Subscriber sub1 = next_goal.subscribe("/odom", 1000, pose_callback);
  ros::Subscriber sub2 = next_goal.subscribe("/plan_path", 1000, path_callback);

  ros::Publisher pub1 = next_goal.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  pubPassedPath = next_goal.advertise<nav_msgs::Path>("passedPath", 1000);

  ros::Rate loop_rate(10);

  geometry_msgs::PoseStamped goal_msgs;
  int count = 0;
  double angle;
  bool goal_reached = false;
  // 获取判断到达当前目标的阈值
  if (!next_goal.getParam("/NextGoal/tolerance_goal", normeNextGoal))
  {
    ROS_ERROR("Please set your tolerance_goal");
    return 0;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    if (new_path)
    {
      count = 0;
      new_path = false;
    }

    if (!pathPlanner.Path.empty())
    {
      // 当前距离达到了
      if (sqrt(pow(x_current - pathPlanner.Path[count].x, 2) + pow(y_current - pathPlanner.Path[count].y, 2)) <= normeNextGoal)
      {
        count++;
        goal_reached = false;
      }
      if (goal_reached == false)
      {
        goal_msgs.header.frame_id = "odom";
        goal_msgs.header.stamp = ros::Time::now();
        goal_msgs.pose.position.x = pathPlanner.Path[count].x;
        goal_msgs.pose.position.y = pathPlanner.Path[count].y;
        goal_msgs.pose.position.z = 0;
        if (count < pathPlanner.Path.size())
          angle = atan2(pathPlanner.Path[count + 1].y - pathPlanner.Path[count].y, pathPlanner.Path[count + 1].x - pathPlanner.Path[count].x);
        else
          angle = atan2(pathPlanner.Path[0].y - pathPlanner.Path[count].y, pathPlanner.Path[0].x - pathPlanner.Path[count].x);
        float w,x,y,z;
        eulerAngles2Quaternion(angle,w,x,y,z);
        goal_msgs.pose.orientation.w = w;
        goal_msgs.pose.orientation.x = x;
        goal_msgs.pose.orientation.y = y;
        goal_msgs.pose.orientation.z = z;

        goal_reached = true;
        pub1.publish(goal_msgs);
      }
      
    }

    loop_rate.sleep();
  }

  return 0;
}
