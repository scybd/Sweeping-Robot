#!/usr/bin/env python3
import rospy
import roslaunch
import signal
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point, Quaternion


# 向explore服务器发送终止请求
def stop_explore_lite():
    pass

# 保存地图
def save_map():
    # 获取amcl的launch文件路径
    launch_file_path = roslaunch.rlutil.resolve_launch_arguments(['auto_nav', 'save_map.launch'])
    # 创建Launch对象
    launch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), launch_file_path)
    # 启动launch文件
    launch.start()
    try:
        # 等待节点结束
        launch.spin()
    finally:
        # 关闭launch
        launch.shutdown()

# 启动amcl返航
def run_amcl():
    # 获取amcl的launch文件路径
    launch_file_path = roslaunch.rlutil.resolve_launch_arguments(['auto_nav', 'amcl.launch'])
    # 创建Launch对象
    launch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), launch_file_path)
    # 启动launch文件
    launch.start()
    try:
        # 等待节点结束
        launch.spin()
    finally:
        # 关闭launch
        launch.shutdown()


class GoalMonitorNode:
    def __init__(self):
        rospy.init_node('return_to_start', anonymous=True)

        # Handle SIGINT and SIGTERM signals
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

        # 订阅/move_base/goal话题
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_status_callback)
        
        # 定义发布器
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

        # 设置计时器，每秒触发一次回调函数
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        # 记录上次收到消息的时间
        self.last_goal_time = rospy.Time.now()

        rospy.spin()

    def signal_handler(self, signum, frame):
        rospy.loginfo("Received signal {}, shutting down...".format(signum))
        # Implement cleanup or additional shutdown logic if needed
        rospy.signal_shutdown("Received signal, shutting down")


    def goal_status_callback(self, msg):
        # 更新上次收到消息的时间
        self.last_goal_time = rospy.Time.now()

    def timer_callback(self, event):
        # 检查是否超过20秒没有收到消息
        if rospy.Time.now() - self.last_goal_time > rospy.Duration(25):
            rospy.loginfo("No goal received in the last 20 seconds. Sending a new goal.")

            stop_explore_lite()
            rospy.loginfo("1")
            rospy.sleep(1.0)    # 确保停止
            rospy.loginfo("2")
            save_map()
            rospy.loginfo("3")
            #run_amcl()

            # 创建返航的目标消息
            new_goal = MoveBaseActionGoal()
            new_goal.goal_id.stamp = rospy.Time.now()
            new_goal.goal.target_pose.header.frame_id = 'map'
            new_goal.goal.target_pose.pose.position = Point(x=-8.0, y=0.0, z=0.0)
            new_goal.goal.target_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # 发布新的目标消息到/move_base/goal话题
            self.goal_publisher.publish(new_goal)

if __name__ == '__main__':
    try:
        # Set up signal handling in the main thread
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

        node = GoalMonitorNode()
    except rospy.ROSInterruptException:
        pass

    