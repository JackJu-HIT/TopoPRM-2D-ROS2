#include "planner_interface.h"

#include <cmath>
#include <utility>

namespace topoprm_2d_ros {

void PlannerInterface::initParam(double max_vel, double max_acc) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
}

void PlannerInterface::initMap(double x_size, double y_size, double resolution,
                               double inflate_values) {
  resolution_ = resolution;
  const Eigen::Vector2i map_size(static_cast<int>(x_size), static_cast<int>(y_size));
  grid_map_ = std::make_shared<GridMap2D>(resolution_, map_size);
  grid_map_->setInflateRadius(inflate_values);

  kino_environment_ = std::make_shared<fast_planner::EDTEnvironment>(grid_map_);
  planner_ = std::make_shared<fast_planner::KinodynamicAstar>();
  planner_->setParam(resolution_, max_vel_, max_acc_);
  planner_->setEnvironment(kino_environment_);
  planner_->init();

}

void PlannerInterface::setPathPoint(const std::vector<PathPoint>& plan_traj) {
  global_plan_traj_ = plan_traj;
}

void PlannerInterface::setObstacles(const std::vector<ObstacleInfo>& obstacles) {
  if (!grid_map_) return;
  grid_map_->resetMap();
  for (const auto& obstacle : obstacles) {
    const Eigen::Vector2i index = grid_map_->worldToGrid(Eigen::Vector2d(obstacle.x, obstacle.y));
    grid_map_->setObstacle(index, true);
  }
  grid_map_->inflate();
  if (kino_environment_) kino_environment_->updateMap();
}

void PlannerInterface::setCurrentVehiclePos(const PathPoint& cur_pose) {
  cur_pose_ = cur_pose;
  cur_pose_.theta = cur_pose.z;
}

void PlannerInterface::setGridMap(const PathPoint& cur_pose) {
  if (!grid_map_) return;
  grid_map_->setCurPose(cur_pose.x, cur_pose.y);
  if (planner_) planner_->init();
}

void PlannerInterface::makePlan() {
  plan_traj_results_.clear();
  search_paths_.clear();
  if (!planner_ || !grid_map_ || global_plan_traj_.empty()) return;

  const auto& goal = global_plan_traj_.back();
  const Eigen::Vector3d start(cur_pose_.x, cur_pose_.y, 0.0);
  const Eigen::Vector3d goal_pos(goal.x, goal.y, 0.0);
  // Match the standalone planner's boundary conditions: start and stop, with
  // a small initial acceleration toward the destination. Heading is carried
  // separately in PathPoint::theta, estimated from the reference path.
  const Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
  const Eigen::Vector3d goal_vel(0.1 * std::cos(goal.theta),
                                 0.1 * std::sin(goal.theta), 0.0);
  const Eigen::Vector3d displacement = goal_pos - start;
  Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
  if (displacement.norm() > 1e-6) {
    start_acc = 0.5 * displacement.normalized();
  }

  planner_->reset();
  const int result = planner_->search(start, start_vel, start_acc,
                                                goal_pos, goal_vel, true);
  planner_->getTopoPaths(search_paths_);
  if (result != fast_planner::KinodynamicAstar::REACH_END &&
      result != fast_planner::KinodynamicAstar::NEAR_END) {
    return;
  }

  const auto points = planner_->getKinoTraj(0.1);
  if (points.size() < 2) return;

  plan_traj_results_.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    PathPoint point;
    point.x = static_cast<float>(points[i].x());
    point.y = static_cast<float>(points[i].y());
    if (i + 1 < points.size()) {
      point.theta = static_cast<float>(std::atan2(points[i + 1].y() - points[i].y(),
                                                  points[i + 1].x() - points[i].x()));
    } else {
      // The terminal state is near-stationary, so use the requested yaw.
      point.theta = goal.theta;
    }
    plan_traj_results_.push_back(point);
  }
}

void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint>& results) const {
  results = plan_traj_results_;
}

void PlannerInterface::getObstacles(std::vector<ObstacleInfo>& obstacles) const {
  if (!grid_map_) return;
  for (const auto& position : grid_map_->getObstaclePointCloud()) {
    ObstacleInfo obstacle;
    obstacle.x = static_cast<float>(position.x());
    obstacle.y = static_cast<float>(position.y());
    obstacles.push_back(obstacle);
  }
}

void PlannerInterface::getSearchPath(
    std::vector<std::vector<Eigen::Vector2d>>& paths) const {
  paths = search_paths_;
}

}  // namespace topoprm_2d_ros
