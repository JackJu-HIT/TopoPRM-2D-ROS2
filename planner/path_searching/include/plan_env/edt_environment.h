#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include "plan_env/grid_map.h"

namespace fast_planner {
using std::list;
using std::shared_ptr;
using std::vector;

// 2D occupancy and distance-field adapter used by TopoPRM.
class EDTEnvironment {
 public:
  using Ptr = std::shared_ptr<EDTEnvironment>;
  explicit EDTEnvironment(const std::shared_ptr<GridMap2D>& map)
      : map_(map), sdf_map_(std::make_shared<Occupancy>(map)) { updateMap(); }
  struct Occupancy {
    explicit Occupancy(std::shared_ptr<GridMap2D> map) : map_(std::move(map)) {}
    void update() { obstacle_points_ = map_->getObstaclePointCloud(false); }
    double getResolution() const { return map_->resolution(); }
    Eigen::Vector3d getOrigin() const {
      return Eigen::Vector3d(map_->origin().x(), map_->origin().y(), 0.0);
    }
    void getRegion(Eigen::Vector3d& origin, Eigen::Vector3d& size) const {
      origin.head<2>() = map_->origin();
      origin.z() = -0.5;
      size.head<2>() = map_->mapSize().cast<double>() * map_->resolution();
      size.z() = 1.0;
    }
    int getInflateOccupancy(const Eigen::Vector3d& p) const {
      return (std::abs(p.z()) > 1e-3 ||
              map_->queryInflated(p.head<2>()) != GridMap2D::CellState::kFree) ? 1 : 0;
    }
    double distance(const Eigen::Vector2d& p) const {
      const Eigen::Vector2d local = p - map_->origin();
      const Eigen::Vector2d size = map_->mapSize().cast<double>() * map_->resolution();
      if (!p.allFinite() || (local.array() < 0.0).any() ||
          (local.array() >= size.array()).any()) return 0.0;
      if (obstacle_points_.empty()) return 1e3;
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto& obstacle : obstacle_points_)
        nearest = std::min(nearest, (p - obstacle).norm());
      return std::max(0.0, nearest - map_->circleRadius());
    }
    double getDistance(const Eigen::Vector3i& index) const {
      const Eigen::Vector2i idx = index.head<2>();
      if (!map_->isIndexValid(idx)) return 0.0;
      return distance(map_->gridToWorld(idx));
    }
    void indexToPos(const Eigen::Vector3i& index, Eigen::Vector3d& p) const {
      p.head<2>() = map_->gridToWorld(index.head<2>());
      p.z() = 0.0;
    }
    std::shared_ptr<GridMap2D> map_;
    std::vector<Eigen::Vector2d> obstacle_points_;
  };
 private:
  std::shared_ptr<GridMap2D> map_;
 public:
  std::shared_ptr<Occupancy> sdf_map_;
  void updateMap() { sdf_map_->update(); }
  double evaluateCoarseEDT(Eigen::Vector3d& p, double) const {
    return sdf_map_->distance(p.head<2>());
  }
  void evaluateEDTWithGrad(const Eigen::Vector3d& p, double, double& distance,
                           Eigen::Vector3d& gradient) const {
    const double step = sdf_map_->getResolution();
    distance = sdf_map_->distance(p.head<2>());
    gradient.setZero();
    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Vector2d forward = p.head<2>();
      Eigen::Vector2d backward = p.head<2>();
      forward(axis) += step;
      backward(axis) -= step;
      gradient(axis) = (sdf_map_->distance(forward) - sdf_map_->distance(backward)) /
                       (2.0 * step);
    }
  }
};
}
