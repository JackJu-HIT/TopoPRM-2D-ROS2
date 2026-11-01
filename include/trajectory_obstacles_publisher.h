/*
MIT License

Copyright (c) 2025 Chunyu Ju

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
@Function: Kinodynamic A* + RViz Interactive Input (Global Path + Obstacles)
@Create by: juchunyu@qq.com
@Date: 2025-11-01 18:58:01
*/
#ifndef TRAJECTORY_OBSTACLES_PUBLISHER_H
#define TRAJECTORY_OBSTACLES_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <memory>
#include <mutex>

// Planner interface
#include "planner_interface.h"

using namespace topoprm_2d_ros;

class TrajectoryAndObstaclesPublisher : public rclcpp::Node
{
public:
    TrajectoryAndObstaclesPublisher();
    ~TrajectoryAndObstaclesPublisher() = default;

private:
    void init_planner();
    void publish_and_plan();
    
    // 回调函数
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    // 发布函数
    void publish_global_path();
    void publish_planned_trajectory();
    void publish_obstacles();
    void publish_search_path();
    void publish_local_obstacles();

    // 辅助函数
    void add_obstacle_at_position(double x, double y);

    void discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                           std::vector<PathPoint>& discrete_trajectory,
                           double interval = 0.1);

    double distance(const PathPoint& p1, const PathPoint& p2);

    // ROS 2发布者/订阅者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr search_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_traj_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_local_pub_;

    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_estimate_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    // Planning interface
    std::shared_ptr<PlannerInterface> planner_;

    // 数据存储
    std::vector<PathPoint> global_plan_traj_;
    std::vector<ObstacleInfo> obstacles_;
    std::mutex data_mutex_;
    bool has_valid_global_path_;
    bool has_obstacles_;
    bool has_current_pose_ = false;
    bool needs_replan_;  // 新增：是否需要重新规划的标志
    std::vector<PathPoint> planned_traj;

    PathPoint  cur_pose_;

    std::vector<std::vector<Eigen::Vector2d>> search_paths_;

    // 参数
    double max_vel_ = 2.0;
    double max_acc_ = 3.0;
    double map_resolution_ = 0.1;
    double map_x_size_ = 50.0;
    double map_y_size_ = 50.0;
    double map_inflate_value_ = 0.4;

};

#endif // TRAJECTORY_OBSTACLES_PUBLISHER_H
