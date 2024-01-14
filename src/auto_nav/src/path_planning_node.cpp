#include "path_planning.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning");

    tf2_ros::Buffer tf;
    tf2_ros::TransformListener tf2_listener(tf);
    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);
    
    ros::Duration(5).sleep();
    PathPlanning clr(&lcr);
    clr.getPathInROS();         // 根据栅格地图做路径规划
    ros::Rate r(1);
    while(ros::ok()){           // 循环发布规划好的路径
      clr.publishCoveragePath();
      ros::spinOnce();
      r.sleep();
    }

    ros::shutdown();
    return 0;
}

