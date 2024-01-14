#include "path_planning.h"
#include <costmap_2d/cost_values.h>


PathPlanning::PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos)
{
    m_cosmap2dRos = costmap2dRos;
    m_costmap2d = costmap2dRos->getCostmap();

    ros::NodeHandle private_nh;
    m_planPub = private_nh.advertise<nav_msgs::Path>("plan_path", 1);               // 用于在rviz中画出规划路径
    m_gridPub = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid", 1);   // 用于在rviz中画出已清扫区域

    string sizeOfCellString, coveredValueStr;
    m_cellSize = 3;
    if (private_nh.searchParam("size_of_cell", sizeOfCellString))   // 搜索参数,根据名称"size of cell"搜索参数，将对应名称下的参数值赋给sizeOfCellString.
        private_nh.param("size_of_cell", m_cellSize, 3);            // 设置机器人占据n*n的栅格，决定规划的稀疏   

    m_gridCoveredValue = 0;
    if (private_nh.searchParam("grid_covered_value", coveredValueStr))
        private_nh.param("grid_covered_value", m_gridCoveredValue, 0);


    // 获取原始地图尺寸
    int costmap2dX = m_costmap2d->getSizeInCellsX();
    int costmap2dY = m_costmap2d->getSizeInCellsY();

    m_srcMap = Mat(costmap2dY, costmap2dX, CV_8U);  // 先raws后cols
    // Mat中原点在左上，Costmap2D中原点在左下，需要做一个变换
    for (int r=0; r<costmap2dY; r++) {
        for (int c=0; c<costmap2dX; c++) {
            m_srcMap.at<uchar>(r, c) = m_costmap2d->getCost(c, costmap2dY-r-1);
        }
    }

    initMat();  
    initCoveredGrid();

    if (!m_srcMap.empty())
        m_initialized = true;
    else
        m_initialized = false;
}


vector<geometry_msgs::PoseStamped> PathPlanning::getPathInROS()
{
    if (!m_pathVecInROS.empty()) m_pathVecInROS.clear();
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    getPathInCV();      // 得到m_pathVec，即在CV中的路径
    
    /*trasnsform*/
    vector<CellIndex>::iterator iter;
    int sizey = m_cellMat.rows;

    for (iter = m_pathVec.begin(); iter != m_pathVec.end(); iter++)
    {
        m_costmap2d->mapToWorld((*iter).col * m_cellSize + m_cellSize / 2, (sizey - (*iter).row - 1) * m_cellSize + m_cellSize / 2, pose.position.x, pose.position.y);
        pose.orientation.w = cos((*iter).theta * PI / 180 / 2); //(sizey-(*iter).row-1)
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = sin((*iter).theta * PI / 180 / 2);
        posestamped.header.stamp = ros::Time::now();
        posestamped.header.frame_id = "map";
        posestamped.pose = pose;

        m_pathVecInROS.push_back(posestamped);
    }
    // publishPlan(m_pathVecInROS);
    return m_pathVecInROS;
}


void PathPlanning::initMat()
{
    // 以下代码为初始化m_cellMat和m_freeSpaceVec
    m_cellMat = Mat(m_srcMap.rows / m_cellSize, m_srcMap.cols / m_cellSize, m_srcMap.type()); // cellMat是以之前规定的cell为最小单位重新划分的地图
    m_freeSpaceVec.clear();
    bool isFree = true;
    for (int r=0; r<m_cellMat.rows; r++)
    {
        for (int c=0; c<m_cellMat.cols; c++)
        {
            isFree = true;
            // 新划分的地图栅格中若有一个原来栅格是有障碍的，就把它所处的新单元格也设置为有障碍的
            for (int i=0; i<m_cellSize; i++)
            {
                for (int j=0; j<m_cellSize; j++)
                {
                    if (m_srcMap.at<uchar>(r * m_cellSize + i, c * m_cellSize + j) != costmap_2d::FREE_SPACE)
                    {
                        isFree = false;
                        break;
                    }
                }
            }
            if (isFree)
            {
                CellIndex ci;
                ci.row = r;
                ci.col = c;
                ci.theta = 0;
                m_freeSpaceVec.push_back(ci);
                m_cellMat.at<uchar>(r, c) = costmap_2d::FREE_SPACE;         // 0
            }
            else m_cellMat.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;   // 254
        }
    }

    // 以下代码为初始化m_neuralMat
    m_neuralMat = Mat(m_cellMat.rows, m_cellMat.cols, CV_32F);  
    for (int i=0; i<m_neuralMat.rows; i++)
    {
        for (int j=0; j<m_neuralMat.cols; j++)
        {
            if (m_cellMat.at<uchar>(i, j) == costmap_2d::LETHAL_OBSTACLE)
                m_neuralMat.at<float>(i, j) = -100000.0; 
            else
                m_neuralMat.at<float>(i, j) = 50.0;
        }
    }
    return;
}


void PathPlanning::initCoveredGrid()
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(m_costmap2d->getMutex())); // 获取m_costmap2d的互斥锁，因为 Costmap2D 对象可能会被多个线程访问和修改
    double resolution = m_costmap2d->getResolution();       // 分辨率，即每个栅格单元的尺寸

    m_coveredPathGrid.header.frame_id = "map";              // m_coveredPathGrid是costmap中的占据栅格地图消息。
    m_coveredPathGrid.header.stamp = ros::Time::now();
    m_coveredPathGrid.info.resolution = resolution;

    m_coveredPathGrid.info.width = m_costmap2d->getSizeInCellsX();
    m_coveredPathGrid.info.height = m_costmap2d->getSizeInCellsY();

    double wx, wy;
    m_costmap2d->mapToWorld(0, 0, wx, wy);              // 从地图坐标系转换至世界坐标系。
    m_coveredPathGrid.info.origin.position.x = wx - resolution / 2;
    m_coveredPathGrid.info.origin.position.y = wy - resolution / 2;
    m_coveredPathGrid.info.origin.position.z = 0.0;
    m_coveredPathGrid.info.origin.orientation.w = 1.0;

    m_coveredPathGrid.data.resize(m_coveredPathGrid.info.width * m_coveredPathGrid.info.height);

    unsigned char *data = m_costmap2d->getCharMap();
    for (unsigned int i = 0; i < m_coveredPathGrid.data.size(); i++)
        m_coveredPathGrid.data[i] = data[i];             // 将代价值赋予到栅格地图的每个对应栅格

}


void PathPlanning::getPathInCV()
{
    CellIndex initPoint, nextPoint, currentPoint;
 
    initPoint.theta = 0;
    bool isok = m_cosmap2dRos->getRobotPose(m_initPose);    // Get the pose of the robot in the global frame of the costmap

    unsigned int mx, my;
    double wx = m_initPose.pose.position.x;     // 获取原点的x坐标（世界）
    double wy = m_initPose.pose.position.y;     // 获取原点的y坐标（世界）
    bool getmapcoor = m_costmap2d->worldToMap(wx, wy, mx, my);
    initPoint.row = m_cellMat.rows - my / m_cellSize - 1;   // 初始点的转换
    initPoint.col = mx / m_cellSize;

    currentPoint = initPoint;
    m_pathVec.clear();
    m_pathVec.push_back(initPoint);     // 在最终生成的路径中压入起始点

    float initTheta = initPoint.theta; 
    const float c_0 = 50;               // 
    float e = 0.0, v = 0.0, v_1 = 0.0, deltaTheta = 0.0, lastTheta = initTheta, PI = 3.14159;
    vector<float> thetaVec = {0, 45, 90, 135, 180, 225, 270, 315};

    for (int loop=0; loop<9000; loop++)
    {
        vector<CellIndex>::iterator it;

        int maxIndex = 0;       
        float max_v = -300;     // v阈值初始值，选取thetaVec中第maxIndex个角度
        m_neuralMat.at<float>(currentPoint.row, currentPoint.col) = -250.0; // 当前点权值初始化
        lastTheta = currentPoint.theta;
        // 该for循环确定下一个点的方向
        for (int id = 0; id < 8; id++)
        {
            deltaTheta = max(thetaVec[id], lastTheta) - min(thetaVec[id], lastTheta);
            if (deltaTheta > 180)
                deltaTheta = 360 - deltaTheta;
            e = 1 - abs(deltaTheta) / 180;          // deltaTheta越大e越小
            switch (id)
            {
            case 0:
                if (currentPoint.col == m_neuralMat.cols - 1)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row, currentPoint.col + 1) + c_0 * e;    // 加入方向权重
                break;

            case 1:
                if (currentPoint.col == m_neuralMat.cols - 1 || currentPoint.row == 0)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row - 1, currentPoint.col + 1) + c_0 * e - 200;  // 对于倾斜的情况额外减点权重
                break;

            case 2:
                if (currentPoint.row == 0)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row - 1, currentPoint.col) + c_0 * e;
                break;

            case 3:
                if (currentPoint.col == 0 || currentPoint.row == 0)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row - 1, currentPoint.col - 1) + c_0 * e - 200;
                break;

            case 4:
                if (currentPoint.col == 0)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row, currentPoint.col - 1) + c_0 * e;
                break;

            case 5:
                if (currentPoint.col == 0 || currentPoint.row == m_neuralMat.rows - 1)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row + 1, currentPoint.col - 1) + c_0 * e - 200;
                break;

            case 6:
                if (currentPoint.row == m_neuralMat.rows - 1)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row + 1, currentPoint.col) + c_0 * e;
                break;

            case 7:
                if (currentPoint.col == m_neuralMat.cols - 1 || currentPoint.row == m_neuralMat.rows - 1)
                {
                    v = -100000;
                    break;
                }
                v = m_neuralMat.at<float>(currentPoint.row + 1, currentPoint.col + 1) + c_0 * e - 200;
                break;

            default:
                break;
            }
            if (v > max_v)
            {
                max_v = v;
                maxIndex = id;
            }

            if (v == max_v && id > maxIndex)
            {
                max_v = v;
                maxIndex = id;
            }
        }
        // 经过与八个方向的方向计算，得到max_v和maxIndex

        // 以下代码确定下一个点的位置，并结合方向和位置确定下一个点
        if (max_v <= 0)
        {
            float dist = 0.0, minDist = 100000000;
            int ii = 0, minIndex = -1;
            for (it = m_freeSpaceVec.begin(); it != m_freeSpaceVec.end(); it++)
            {
                if (m_neuralMat.at<float>((*it).row, (*it).col) > 0)
                {
                    if (boundingJudge((*it).row, (*it).col))    // 周围是否存在-250的点，也就是当前点
                    {
                        dist = sqrt((currentPoint.row - (*it).row) * (currentPoint.row - (*it).row) + (currentPoint.col - (*it).col) * (currentPoint.col - (*it).col));
                        // 找到离当前点最近的无障碍点
                        if (dist < minDist)
                        {
                            minDist = dist;
                            minIndex = ii;
                        }
                    }
                }
                ii++;
            }

            if (minIndex != -1 && minDist != 100000000)
            {
                cout << "next point index: " << minIndex << endl;
                cout << "distance: " << minDist << endl;
                nextPoint = m_freeSpaceVec[minIndex];
                currentPoint = nextPoint;
                m_pathVec.push_back(nextPoint);

                continue;
            }
            else
            {
                ROS_INFO("The program has been dead because of the self-locking");
                ROS_INFO("The program has gone through %ld steps", m_pathVec.size());
                break;
            }
        }

        switch (maxIndex)
        {
        case 0:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col + 1;
            break;
        case 1:
            nextPoint.row = currentPoint.row - 1;
            nextPoint.col = currentPoint.col + 1;
            break;
        case 2:
            nextPoint.row = currentPoint.row - 1;
            nextPoint.col = currentPoint.col;
            break;
        case 3:
            nextPoint.row = currentPoint.row - 1;
            nextPoint.col = currentPoint.col - 1;
            break;
        case 4:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col - 1;
            break;
        case 5:
            nextPoint.row = currentPoint.row + 1;
            nextPoint.col = currentPoint.col - 1;
            break;
        case 6:
            nextPoint.row = currentPoint.row + 1;
            nextPoint.col = currentPoint.col;
            break;
        case 7:
            nextPoint.row = currentPoint.row + 1;
            nextPoint.col = currentPoint.col + 1;
            break;
        default:
            break;
        }
        nextPoint.theta = thetaVec[maxIndex];
        currentPoint = nextPoint;
        m_pathVec.push_back(nextPoint);

    }
}


bool PathPlanning::boundingJudge(int a, int b)
{
    int num = 0;
    for (int i = -1; i <= 1; i++)
    {
        for (int m = -1; m <= 1; m++)
        {
            if (i == 0 && m == 0)
            {
                continue;
            }
            if (m_neuralMat.at<float>((a + i), (b + m)) == -250.0)
                num++;
        }
    }
    if (num != 0)
        return true;
    else
        return false;
}


void PathPlanning::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
{
    if (!m_initialized)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    m_planPub.publish(gui_path);
}


void PathPlanning::publishCoveragePath()
{
    publishPlan(this->m_pathVecInROS);
}

