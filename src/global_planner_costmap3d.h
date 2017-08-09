/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <autonomous_wc/AStar3D_Global_Planner.h>
#include <autonomous_wc/Costmap2D.h>
#include "cnpy.h"

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {

class GlobalPlannerCostmap3D : public nav_core::BaseGlobalPlanner {

  public:

    GlobalPlannerCostmap3D();
    GlobalPlannerCostmap3D(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan
                 );

    ros::NodeHandle n_;
    bool initialized_;
    ros::ServiceClient client_path_planner_;
    // std::vector<int(*)[4]> costmap3d_;
    autonomous_wc::Costmap2D costmap2d_;

  private:

    




};
};
#endif
