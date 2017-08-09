#include <pluginlib/class_list_macros.h>
#include "global_planner_costmap3d.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlannerCostmap3D, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

GlobalPlannerCostmap3D::GlobalPlannerCostmap3D ()
{

}

GlobalPlannerCostmap3D::GlobalPlannerCostmap3D(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialized_ = false;
}

void GlobalPlannerCostmap3D::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{ 
  // // Read 3D costmap
  // cnpy::NpyArray arr = cnpy::npy_load("/home/robin/catkin_ws/src/osu_research/autonomous_wc/data/costmap3d.npy");
  // int* loaded_data = reinterpret_cast<int*>(arr.data);

  // cout << endl << arr.shape[0] << ", " << arr.shape[1] << ", " << arr.shape[2] << endl;

  // // Load data where value is greater 0 into vector
  // for (int i=0;i<arr.shape[0];i++)
  //   for (int j=0;j<arr.shape[1];j++)
  //     for (int k=0;k<arr.shape[2];k++)  
  //     {
  //       int value = *(loaded_data++);

  //       if (value > 0)
  //       {
  //         int tmp[4] = {i, j, k, value};

  //         cout << value << "  :  " << i << ", " << j << ", " << k << endl;       

  //         costmap3d_.push_back(&tmp);
  //       }
  //     }
  // cout << "ASD";
  // int a[2000][2000][4];
  // cout << "DAS";

  if(!initialized_)
  {
    costmap_2d::Costmap2D* costmap2d;
    unsigned int size_x;
    unsigned int size_y;

    // Initialize srv call
    ros::NodeHandle n;
    client_path_planner_ = n.serviceClient<autonomous_wc::AStar3D_Global_Planner>("AStar3D_Global_Planner");

    // Get the costmap_ from costmap_ros
    costmap2d = costmap_ros->getCostmap();      
    
    // Get the size of the costmap in cells
    size_x = costmap2d->getSizeInCellsX();
    size_y = costmap2d->getSizeInCellsY();

    // std::cout << costmap_2d->LEATHAL_OBSTACLE;

    // Convert the costmap to a 2D array used by the path planner
    costmap2d_ = autonomous_wc::Costmap2D();
    for(int y_i=0;y_i<size_y;y_i++)
    {
      for(int x_i=0;x_i<size_x;x_i++) 
      {
        costmap2d_.data.push_back((int) costmap2d->getCost(x_i, y_i));
      }      
    }
    costmap2d_.resolution = costmap2d->getResolution();
    costmap2d_.height = size_y;
    costmap2d_.width = size_x;


    initialized_ = true;
  }
} 

bool GlobalPlannerCostmap3D::makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal,  
                            std::vector<geometry_msgs::PoseStamped>& plan )
{
  // Create service request
  autonomous_wc::AStar3D_Global_Planner srv_path_planner;
  
  // Update information for the service request
  srv_path_planner.request.map = costmap2d_;
  srv_path_planner.request.start = start;
  srv_path_planner.request.goal = goal;

  geometry_msgs::PoseArray path;

  // Try to call the path planner service
  if (client_path_planner_.call(srv_path_planner))
  {
    ROS_INFO("Path Length (in cells):     %d", (int)srv_path_planner.response.path.poses.size());
    path = srv_path_planner.response.path;
  }
  else
  {
    ROS_ERROR("Failed to call path planner service!");
    return 1;
  }

  geometry_msgs::PoseStamped new_pose;
  for (int i=0;i<path.poses.size();i++)
  {
    new_pose = geometry_msgs::PoseStamped();
    new_pose.header = path.header;
    new_pose.header.seq = i;
    new_pose.pose = path.poses.at(i);

    plan.push_back(new_pose);
  }
}



};