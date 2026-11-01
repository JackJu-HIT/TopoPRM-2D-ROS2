#ifndef KINODYNAMIC_ASTAR_PLANNER_INTERFACE_H_
#define KINODYNAMIC_ASTAR_PLANNER_INTERFACE_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/grid_map.h>

namespace topoprm_2d_ros {

struct PathPoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float v = 0.0F;
  float theta = 0.0F;
};

struct ObstacleInfo {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

class PlannerInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void initParam(double max_vel, double max_acc);
  void initMap(double x_size, double y_size, double resolution,
               double inflate_values);
  void setPathPoint(const std::vector<PathPoint>& plan_traj);
  void setObstacles(const std::vector<ObstacleInfo>& obstacles);
  void setCurrentVehiclePos(const PathPoint& cur_pose);
  void setGridMap(const PathPoint& cur_pose);
  void makePlan();
  void getLocalPlanTrajResults(std::vector<PathPoint>& plan_traj_results) const;
  void getObstacles(std::vector<ObstacleInfo>& obstacles) const;
  void getSearchPath(std::vector<std::vector<Eigen::Vector2d>>& paths) const;

 private:
  double max_vel_ = 2.0;
  double max_acc_ = 3.0;
  double resolution_ = 0.1;
  std::shared_ptr<GridMap2D> grid_map_;
  PathPoint cur_pose_;
  std::vector<PathPoint> global_plan_traj_;
  std::vector<PathPoint> plan_traj_results_;
  std::vector<std::vector<Eigen::Vector2d>> search_paths_;
  fast_planner::KinodynamicAstar::Ptr planner_;
  fast_planner::EDTEnvironment::Ptr kino_environment_;
};

}  // namespace topoprm_2d_ros

#endif  // KINODYNAMIC_ASTAR_PLANNER_INTERFACE_H_
