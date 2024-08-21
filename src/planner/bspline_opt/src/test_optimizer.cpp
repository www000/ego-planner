#include <ros/ros.h>

#include "plan_env/grid_map.h"
#include "path_searching/dyn_a_star.h"
#include "bspline_opt/bspline_optimizer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_optimizer");
  ros::NodeHandle nh("~");
  std::cout << "test optimizer start" << std::endl;

  auto grid_map = std::make_shared<GridMap>();
  grid_map->InitMapWithDefault(nh);

  auto optimizer = std::make_shared<ego_planner::BsplineOptimizer>();
  optimizer->InitParam();
  optimizer->setEnvironment(grid_map);
  optimizer->a_star_ = std::make_shared<AStar>();
  optimizer->a_star_->initGridMap(grid_map, Eigen::Vector3i(100, 100, 100));

  return 0;
}
