/*
 * This file is part of Fast-Planner.
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, HKUST.
 *
 * Fast-Planner is free software under the GNU Lesser General Public License
 * version 3 or (at your option) any later version.
 *
 * This project replaces the Kinodynamic A* search implementation with the
 * TopoPRM frontend while preserving its public interface for ROS integration.
 */
#include "path_searching/kinodynamic_astar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace fast_planner {

void KinodynamicAstar::setParam(double map_resolution, double, double) {
  if (std::isfinite(map_resolution) && map_resolution > 0.0)
    resolution_ = map_resolution;
}

void KinodynamicAstar::init() {
  candidate_paths_.clear();
  best_path_.clear();
  if (edt_environment_) {
    topology_prm_.setEnvironment(edt_environment_);
    topology_prm_.init();
  }
}

void KinodynamicAstar::reset() {
  candidate_paths_.clear();
  best_path_.clear();
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env) {
  edt_environment_ = env;
  topology_prm_.setEnvironment(env);
  if (env) topology_prm_.init();
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d,
                             Eigen::Vector3d, Eigen::Vector3d end_pt,
                             Eigen::Vector3d, bool, bool, double) {
  reset();
  if (!edt_environment_ || !edt_environment_->sdf_map_) return NO_PATH;

  // The local map origin rolls with the robot, so refresh TopoPRM's voxel
  // offset before each search.
  topology_prm_.init();

  std::list<GraphNode::Ptr> graph;
  std::vector<std::vector<Eigen::Vector3d>> raw_paths;
  std::vector<std::vector<Eigen::Vector3d>> filtered_paths;
  std::vector<std::vector<Eigen::Vector3d>> selected_paths;
  topology_prm_.findTopoPaths(start_pt, end_pt, {}, {}, graph, raw_paths,
                              filtered_paths, selected_paths);

  const double collision_step = std::max(0.02, 0.5 * resolution_);
  double best_length = std::numeric_limits<double>::infinity();
  // selectShortPaths removes its selected routes from filtered_paths, so join
  // both outputs to retain every route produced by TopoPRM for validation.
  std::vector<std::vector<Eigen::Vector3d>> candidate_routes = filtered_paths;
  candidate_routes.insert(candidate_routes.end(), selected_paths.begin(), selected_paths.end());
  for (const auto& route : candidate_routes) {
    if (route.size() < 2) continue;
    bool collision_free = true;
    for (size_t i = 1; i < route.size() && collision_free; ++i) {
      const Eigen::Vector3d delta = route[i] - route[i - 1];
      const int samples = std::max(1, static_cast<int>(std::ceil(delta.head<2>().norm() /
                                                                  collision_step)));
      for (int j = 0; j <= samples; ++j) {
        const double ratio = static_cast<double>(j) / samples;
        const Eigen::Vector3d point = route[i - 1] + ratio * delta;
        if (edt_environment_->sdf_map_->getInflateOccupancy(point) != 0) {
          collision_free = false;
          break;
        }
      }
    }
    if (!collision_free) continue;

    candidate_paths_.push_back(route);
    const double length = topology_prm_.pathLength(route);
    if (length < best_length) {
      best_length = length;
      best_path_ = route;
    }
  }

  // TopoPRM can return no connected route when its finite random sampling
  // misses a narrow passage. Fall back to a complete 8-connected grid search
  // over the same inflated occupancy map so feasible scenes still get a plan.
  if (candidate_paths_.empty()) {
    const auto map = edt_environment_->sdf_map_->map_;
    const Eigen::Vector2i start_index = map->worldToGrid(start_pt.head<2>());
    const Eigen::Vector2i goal_index = map->worldToGrid(end_pt.head<2>());
    const auto is_free = [&map](const Eigen::Vector2i& index) {
      if (!map->isIndexValid(index)) return false;
      const Eigen::Vector2d world = map->gridToWorld(index);
      return map->getInflateOccupancy(world);
    };

    if (is_free(start_index) && is_free(goal_index)) {
      struct OpenNode {
        double f;
        double g;
        Eigen::Vector2i index;
        bool operator<(const OpenNode& other) const { return f > other.f; }
      };
      const int width = map->mapSize().x();
      const int height = map->mapSize().y();
      const size_t cell_count = static_cast<size_t>(width) * height;
      const auto cell_id = [width](const Eigen::Vector2i& index) {
        return static_cast<size_t>(index.y()) * width + index.x();
      };
      const auto heuristic = [&goal_index](const Eigen::Vector2i& index) {
        const Eigen::Vector2i delta = (index - goal_index).cwiseAbs();
        const int diagonal = std::min(delta.x(), delta.y());
        return static_cast<double>(delta.x() + delta.y()) +
               (std::sqrt(2.0) - 2.0) * diagonal;
      };

      std::vector<double> costs(cell_count, std::numeric_limits<double>::infinity());
      std::vector<size_t> parents(cell_count, cell_count);
      std::vector<unsigned char> closed(cell_count, 0);
      std::priority_queue<OpenNode> open;
      const size_t start_id = cell_id(start_index);
      const size_t goal_id = cell_id(goal_index);
      costs[start_id] = 0.0;
      open.push({heuristic(start_index), 0.0, start_index});
      constexpr int directions[8][2] = {
          {1, 0}, {-1, 0}, {0, 1}, {0, -1},
          {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

      while (!open.empty()) {
        const OpenNode current = open.top();
        open.pop();
        const size_t current_id = cell_id(current.index);
        if (closed[current_id] || current.g > costs[current_id]) continue;
        if (current_id == goal_id) break;
        closed[current_id] = 1;

        for (const auto& direction : directions) {
          const Eigen::Vector2i next = current.index + Eigen::Vector2i(direction[0], direction[1]);
          if (!is_free(next)) continue;
          const bool diagonal = direction[0] != 0 && direction[1] != 0;
          if (diagonal &&
              (!is_free(current.index + Eigen::Vector2i(direction[0], 0)) ||
               !is_free(current.index + Eigen::Vector2i(0, direction[1])))) continue;

          const size_t next_id = cell_id(next);
          const double next_g = current.g + (diagonal ? std::sqrt(2.0) : 1.0);
          if (next_g >= costs[next_id]) continue;
          costs[next_id] = next_g;
          parents[next_id] = current_id;
          open.push({next_g + heuristic(next), next_g, next});
        }
      }

      if (std::isfinite(costs[goal_id])) {
        std::vector<Eigen::Vector2i> reversed_indices;
        for (size_t id = goal_id; id != cell_count; id = parents[id]) {
          reversed_indices.emplace_back(static_cast<int>(id % width),
                                        static_cast<int>(id / width));
          if (id == start_id) break;
        }
        if (!reversed_indices.empty() && cell_id(reversed_indices.back()) == start_id) {
          std::reverse(reversed_indices.begin(), reversed_indices.end());
          std::vector<Eigen::Vector3d> fallback_path;
          fallback_path.reserve(reversed_indices.size() + 2);
          fallback_path.push_back(start_pt);
          for (size_t i = 1; i + 1 < reversed_indices.size(); ++i) {
            const Eigen::Vector2d point = map->gridToWorld(reversed_indices[i]);
            fallback_path.emplace_back(point.x(), point.y(), 0.0);
          }
          fallback_path.push_back(end_pt);
          candidate_paths_.push_back(fallback_path);
          best_path_ = std::move(fallback_path);
        }
      }
    }
  }

  std::sort(candidate_paths_.begin(), candidate_paths_.end(),
            [this](const auto& lhs, const auto& rhs) {
              return topology_prm_.pathLength(lhs) < topology_prm_.pathLength(rhs);
            });
  return best_path_.empty() ? NO_PATH : REACH_END;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t) {
  if (best_path_.size() < 2) return best_path_;
  const double step = std::isfinite(delta_t) && delta_t > 0.0 ? delta_t : resolution_;

  std::vector<Eigen::Vector3d> sampled;
  sampled.push_back(best_path_.front());
  for (size_t i = 1; i < best_path_.size(); ++i) {
    const Eigen::Vector3d delta = best_path_[i] - best_path_[i - 1];
    const int segments = std::max(1, static_cast<int>(std::ceil(delta.head<2>().norm() / step)));
    for (int j = 1; j <= segments; ++j) {
      const double ratio = static_cast<double>(j) / segments;
      sampled.push_back(best_path_[i - 1] + ratio * delta);
    }
  }
  return sampled;
}

void KinodynamicAstar::getTopoPaths(
    std::vector<std::vector<Eigen::Vector2d>>& paths) const {
  paths.clear();
  paths.reserve(candidate_paths_.size());
  for (const auto& candidate : candidate_paths_) {
    std::vector<Eigen::Vector2d> path;
    path.reserve(candidate.size());
    for (const auto& point : candidate) path.push_back(point.head<2>());
    paths.push_back(std::move(path));
  }
}

}  // namespace fast_planner
