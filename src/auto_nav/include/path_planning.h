#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace cv;
using namespace std;


constexpr double PI =3.14159;


struct CellIndex
{
    int row;
    int col;
    double theta;       // {0,45,90,135,180,225,270,315}
};


class PathPlanning
{
public:
    PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos);         

    vector<geometry_msgs::PoseStamped> getPathInROS();      // 得到最终在ROS中的路径

    void publishCoveragePath();                             // 用于可视化

    int getSizeOfCell() { return this->m_cellSize; }        // 获取一个单元格有几个栅格，只能为奇数

private:
    void initMat();                         // 初始化m_cellMat和m_neuralMat
    void initCoveredGrid();                 // 初始化m_coveredPathGrid
    void getPathInCV();                     // 利用OpenCV的API求出规划路径，之后再转到ros中
    bool boundingJudge(int a, int b);       // 判断
    void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);

    bool m_initialized;                         // 类初始化成功与否标志位
    costmap_2d::Costmap2DROS *m_cosmap2dRos;    // A ROS wrapper for a 2D Costmap，处理订阅以PointCloud或LaserScan消息形式提供障碍物观察结果的主题。
    costmap_2d::Costmap2D* m_costmap2d;         // 原始代价地图的指针
    Mat m_srcMap;                               // 原始代价地图转为Mat
    Mat m_cellMat;                              // 原始栅格地图按m_CELL_SIZE的大小合并后的地图
    Mat m_neuralMat;                            // 该地图用于路径规划
    vector<CellIndex> m_freeSpaceVec;           // m_cellMat中无障碍的地方
    nav_msgs::OccupancyGrid m_coveredPathGrid;  // 占据栅格地图， 0 表示未占据, 1 表示占据, -1 表示未知 

    geometry_msgs::PoseStamped m_initPose;              // 初始位姿
    vector<CellIndex> m_pathVec;                        // 利用OpenCV的API求得的路径
    vector<geometry_msgs::PoseStamped> m_pathVecInROS;  // m_pathVec转到ros中的路径

    ros::Publisher m_planPub;               // 发布规划好的路径，rviz显示
    ros::Publisher m_gridPub;               // 发布机器人走过的路径，rviz显示

    int m_cellSize;             // 新的栅格大小，必须是原来栅格的奇数倍
    int m_gridCoveredValue;    
};

#endif