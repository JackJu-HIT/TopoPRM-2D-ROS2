#ifndef KINODYNAMIC_ASTAR_H
#define KINODYNAMIC_ASTAR_H

#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include "path_searching/topo_prm.h"

namespace fast_planner {

// Compatibility wrapper for the existing planner interface. The search
// implementation is TopoPRM; getKinoTraj returns its shortest valid route.
class KinodynamicAstar {
 public:
  using Ptr = std::shared_ptr<KinodynamicAstar>;
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  KinodynamicAstar() = default;
  ~KinodynamicAstar() = default;

  void setParam(double map_resolution = 0.1, double max_vel = 2.0,
                double max_acc = 3.0);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);
  void setEnvironment(const EDTEnvironment::Ptr& env);
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
  void getTopoPaths(std::vector<std::vector<Eigen::Vector2d>>& paths) const;

 private:
  double resolution_ = 0.1;
  EDTEnvironment::Ptr edt_environment_;
  TopologyPRM topology_prm_;
  std::vector<std::vector<Eigen::Vector3d>> candidate_paths_;
  std::vector<Eigen::Vector3d> best_path_;
};

}  // namespace fast_planner

#endif  // KINODYNAMIC_ASTAR_H
