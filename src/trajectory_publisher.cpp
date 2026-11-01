/*
 * @Function: TopoPRM + RViz Interactive Input (Global Path + Obstacles)
 * @Create by: juchunyu@qq.com
 * @Date: 2025-11-01 18:58:01
 */
#include "trajectory_obstacles_publisher.h"
#include <chrono>
#include <tf2/utils.h>
#include "geometry_msgs/msg/quaternion.hpp"  // 确保包含四元数消息类型

TrajectoryAndObstaclesPublisher::TrajectoryAndObstaclesPublisher() 
    : Node("topoprm_2d_interactive_node"),
      has_valid_global_path_(false),
      has_obstacles_(false),
      needs_replan_(false)
{
    // 1. 创建发布者（可视化用）
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_global_path", 10);
    search_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectories", 10);
    local_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_local_trajectory", 10);
    obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_obstacles", 10);
    obs_local_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_local_obstacles", 10);

    // 2. 创建订阅者（接收RViz下发的数据）
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::goal_pose_callback, this, std::placeholders::_1)
    );

    // 路径添加：使用Publish Point工具逐个添加
    rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::rviz_point_callback, this, std::placeholders::_1)
    );

    // 障碍物输入方式3：使用2D Pose Estimate工具添加
    pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::pose_estimate_callback, this, std::placeholders::_1)
    );

    // 3. Initialize the planner
    init_planner();

    // 4. 5 Hz planning and visualization timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TrajectoryAndObstaclesPublisher::publish_and_plan, this)
    );

    RCLCPP_INFO(this->get_logger(), "TopoPRM planner node ready.");
    RCLCPP_INFO(this->get_logger(), "=== 使用方法 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加全局路径：使用2D Nav Goal工具设置终点");
    RCLCPP_INFO(this->get_logger(), "2. 添加障碍物（三种方法任选）：");
    RCLCPP_INFO(this->get_logger(), "   - 方法A：使用Publish Point工具点击地图");
    RCLCPP_INFO(this->get_logger(), "   - 方法B：使用2D Pose Estimate工具点击Grid任意位置（推荐）");
    RCLCPP_INFO(this->get_logger(), "=== RViz2设置步骤 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加Grid显示");
    RCLCPP_INFO(this->get_logger(), "2. 添加2D Nav Goal工具（话题：/goal_pose）");
    RCLCPP_INFO(this->get_logger(), "3. 添加2D Pose Estimate工具（使用默认话题：/initialpose）");
    RCLCPP_INFO(this->get_logger(), "4. 添加Publish Point工具（使用默认话题：/clicked_point）");
    RCLCPP_INFO(this->get_logger(), "5. 添加PointCloud2显示（话题：/visual_obstacles）");
    RCLCPP_INFO(this->get_logger(), "6. 添加Path显示（话题：/visual_global_path和/visual_local_trajectory）");
}

// Initialize planner parameters
void TrajectoryAndObstaclesPublisher::init_planner()
{
    const double circle_radius = declare_parameter<double>("vehicle.circle_radius", map_inflate_value_);
    planner_ = std::make_shared<PlannerInterface>();
    planner_->initParam(max_vel_, max_acc_);
    planner_->initMap(map_x_size_, map_y_size_, map_resolution_, circle_radius);
}

// 统一添加障碍物函数
void TrajectoryAndObstaclesPublisher::add_obstacle_at_position(double x, double y)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    ObstacleInfo obs;
    obs.x = x;
    obs.y = y;
    obs.z = 0.0;
    
    obstacles_.push_back(obs);
    has_obstacles_ = true;
    
    if (has_current_pose_ && has_valid_global_path_) needs_replan_ = true;
    
    RCLCPP_INFO(this->get_logger(), "添加障碍物: (%.2f, %.2f), 总障碍物数量: %zu", 
                x, y, obstacles_.size());
}

// 2D Pose Estimate回调函数
void TrajectoryAndObstaclesPublisher::pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
   // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.pose.position.x;
    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.z = tf2::getYaw(q);
    std::cout << "pose_estimate_callback " << std::endl;
    has_current_pose_ = true;
    if (has_valid_global_path_) needs_replan_ = true;
}

// 处理2D Nav Goal - 生成障碍物
void TrajectoryAndObstaclesPublisher::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    add_obstacle_at_position(msg->pose.position.x, msg->pose.position.y);
}

// Collect a clicked point as part of the reference path
void TrajectoryAndObstaclesPublisher::rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // global_plan_traj_.clear();
    PathPoint point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = 0; // 使用z作为theta角度
    global_plan_traj_.push_back(point);
    std::cout << "从RViz接收到全局路径点: (" << point.x << ", " << point.y << ")" << std::endl;
    has_valid_global_path_ = true;
    
    if (has_current_pose_) needs_replan_ = true;
}

// 核心逻辑：检查数据更新→触发规划→发布结果
void TrajectoryAndObstaclesPublisher::publish_and_plan()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Replan when path, pose, or obstacle data changes.
    if (needs_replan_ && has_valid_global_path_ && has_current_pose_)
    {
        search_paths_.clear();
        // 设置全局路径
        if (!global_plan_traj_.empty())
        {
            std::vector<PathPoint> global_plan_traj_temp;


            discretize_trajectory(global_plan_traj_, global_plan_traj_temp, 0.4);
             
            float mindist = 100000000;
            int minddex = 0;
            for(int i = 0; i < global_plan_traj_temp.size();i++)
            {
                double dist =  distance(global_plan_traj_temp[i], cur_pose_); 
                if(dist < mindist)
                {
                    mindist = dist;
                    minddex = i;
                } 
  
            }

            std::vector<PathPoint> global_plan_traj_after;
            global_plan_traj_after.push_back(cur_pose_);
            for(int i = minddex; i < global_plan_traj_temp.size();i++)
            {
                global_plan_traj_after.push_back(global_plan_traj_temp[i]);
            }
            discretize_trajectory(global_plan_traj_after, global_plan_traj_temp, 0.4);

            planner_->setPathPoint(global_plan_traj_temp);
        }
        // std::cout << "cur_pose_x =" << cur_pose_.x << "cur_pose_y =" << cur_pose_.y << std::endl;
        // Rebuild the XY local map around the current pose on every planning update.
        planner_->setGridMap(cur_pose_);
        planner_->setCurrentVehiclePos(cur_pose_);
        // 设置障碍物
        planner_->setObstacles(obstacles_);

        // Retry once only when the planner did not produce a usable trajectory.
        constexpr int max_plan_attempts = 2;
        std::vector<PathPoint> planned_result;
        for (int attempt = 1; attempt <= max_plan_attempts; ++attempt) {
            const auto plan_start = std::chrono::steady_clock::now();
            planner_->makePlan();
            const auto plan_end = std::chrono::steady_clock::now();
            const auto plan_duration_ms = std::chrono::duration<double, std::milli>(
                plan_end - plan_start).count();
            planner_->getLocalPlanTrajResults(planned_result);
            RCLCPP_INFO(this->get_logger(), "planner_->makePlan() 第 %d 次耗时: %.3f ms，轨迹点: %zu",
                        attempt, plan_duration_ms, planned_result.size());
            if (planned_result.size() >= 2) break;
            if (attempt < max_plan_attempts) {
                RCLCPP_WARN(this->get_logger(), "首次规划未生成有效轨迹，正在重试一次");
            }
        }

        planner_->getSearchPath(search_paths_);

        if (has_obstacles_) {
            if (planned_result.size() >= 2) {
                RCLCPP_INFO(this->get_logger(), "规划完成，候选路线: %zu，障碍物: %zu",
                           search_paths_.size(), obstacles_.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "重试后仍未找到可行轨迹，候选路线: %zu，障碍物: %zu",
                             search_paths_.size(), obstacles_.size());
            }
        } else {
            if (planned_result.size() >= 2) {
                RCLCPP_INFO(this->get_logger(), "规划完成! 无障碍物. 路径点: %zu",
                           global_plan_traj_.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "重试后仍未生成有效轨迹");
            }
        }
        
        needs_replan_ = false;

    }

    // 发布所有可视化数据（无论是否更新，保持实时显示）
    publish_global_path();
    publish_planned_trajectory();
    publish_obstacles();
    publish_search_path();
    publish_local_obstacles();
}

// 发布可视化全局路径
void TrajectoryAndObstaclesPublisher::publish_global_path()
{

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = "map";

    for (const auto& path_point : global_plan_traj_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, path_point.theta);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    global_path_pub_->publish(visual_path);
}

void TrajectoryAndObstaclesPublisher::publish_search_path()
{
    // 创建一个MarkerArray消息
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_msg.markers.push_back(clear_marker);

    // 遍历 search_paths_ 中的每一条路径
    for (size_t i = 0; i < search_paths_.size(); ++i)
    {
      const std::vector<Eigen::Vector2d>& current_path = search_paths_[i];

      // 1. 创建一个Marker对象
      visualization_msgs::msg::Marker marker;

      // 2. 设置Marker的基本信息
      marker.header.frame_id = "map"; // 非常重要！指定轨迹所在的坐标系
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "topoprm_candidates";
      marker.id = i;              // 每条路径必须有唯一的ID
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 类型为线串
      marker.action = visualization_msgs::msg::Marker::ADD;      // 动作是添加

      // 3. 设置线的视觉属性
      marker.scale.x = 0.06;

      // 设置颜色（RGBA格式），每条路径可以设置不同的颜色
      marker.color.r = 0.05F;
      marker.color.g = 0.35F + 0.15F * static_cast<float>(i % 3);
      marker.color.b = 0.7F + 0.25F * static_cast<float>(i % 2);
      marker.color.a = 1.0F;

      // 4. 填充路径点
      for (const auto& eigen_point : current_path)
      {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = eigen_point.x();
        ros_point.y = eigen_point.y();
        ros_point.z = 0.02;

        marker.points.push_back(ros_point);
      }

      // 5. 将当前路径的Marker添加到MarkerArray中
      marker_array_msg.markers.push_back(marker);
    }

    // 6. 发布MarkerArray消息
    search_path_pub_->publish(marker_array_msg);
    // RCLCPP_INFO(this->get_logger(), "search_path_pub_ Published %zu trajectories.", search_paths_.size());

}

// Publish the planned local trajectory
void TrajectoryAndObstaclesPublisher::publish_planned_trajectory()
{
    planned_traj.clear();
    planner_->getLocalPlanTrajResults(planned_traj);

    // if (planned_traj.empty()) return;

    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = "map";

    for (size_t i = 0; i < planned_traj.size(); ++i)
    {
        // std::cout << "[publish_planned_trajectory] x = " << planned_traj[i].x << " y =" << planned_traj[i].y << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = planned_traj[i].x;
        pose.pose.position.y = planned_traj[i].y;
        pose.pose.position.z = 0.05;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, planned_traj[i].theta);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    local_traj_pub_->publish(visual_traj);
}

// 发布可视化障碍物
void TrajectoryAndObstaclesPublisher::publish_obstacles()
{
    // if (obstacles_.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = "map";
    visual_obs.width = obstacles_.size();
    visual_obs.height = 1;
    visual_obs.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles_)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_pub_->publish(visual_obs);
}

void TrajectoryAndObstaclesPublisher::publish_local_obstacles()
{
    std::vector<ObstacleInfo> obstacles;
    planner_->getObstacles(obstacles);

    if (obstacles.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = "map";

    // 正确设置点云大小
    visual_obs.height = 1;
    visual_obs.width = obstacles.size();
    visual_obs.is_dense = true;

    // 为点云分配字段
    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles.size());   // 关键！分配空间

    // 创建迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_local_pub_->publish(visual_obs);
}

// 计算两点之间的欧氏距离（单位：米）
double TrajectoryAndObstaclesPublisher::distance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * 将轨迹离散为均匀间隔的点（间隔10cm）
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米，默认0.1米即10cm）
 */
void TrajectoryAndObstaclesPublisher::discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                                            std::vector<PathPoint>& discrete_trajectory,
                                                            double interval) {
    if (original_trajectory.size() < 2) {
        std::cerr << "原始轨迹至少需要2个点！" << std::endl;
        return;
    }

    discrete_trajectory.clear();
    // 添加轨迹起点
    // discrete_trajectory.push_back(cur_pose_);
    discrete_trajectory.push_back(original_trajectory[0]);

    // 遍历原始轨迹的每一段线段
    for (size_t i = 0; i < original_trajectory.size() - 1; ++i) {
        const PathPoint& start = original_trajectory[i];
        const PathPoint& end = original_trajectory[i+1];
        double seg_length = distance(start, end);  // 线段总长度

        if (seg_length < 1e-6) {  // 跳过长度接近0的线段（避免除零）
            continue;
        }

        // 计算当前线段需要插入的点数（不含起点，含终点）
        int num_points = static_cast<int>(seg_length / interval);
        // 最后一个点到终点的距离（避免累积误差）
        double last_interval = seg_length - num_points * interval;

        // 生成线段上的离散点
        for (int j = 1; j <= num_points; ++j) {
            double ratio;
            if (j < num_points) {
                // 前num_points-1个点：按均匀间隔计算
                ratio = (j * interval) / seg_length;
            } else {
                // 最后一个点：直接对齐到线段终点（避免累积误差）
                ratio = 1.0;
            }

            // 线性插值计算点坐标
            PathPoint p;
            p.x = start.x + ratio * (end.x - start.x);
            p.y = start.y + ratio * (end.y - start.y);
            discrete_trajectory.push_back(p);
        }
    }

    // Estimate each point's forward heading from the next segment.
    double heading = 0.0;
    for (size_t i = 0; i + 1 < discrete_trajectory.size(); ++i) {
        const double dx = discrete_trajectory[i + 1].x - discrete_trajectory[i].x;
        const double dy = discrete_trajectory[i + 1].y - discrete_trajectory[i].y;
        if (std::hypot(dx, dy) >= 1e-6) {
            heading = std::atan2(dy, dx);
        }
        discrete_trajectory[i].theta = static_cast<float>(heading);
    }

    // A single final segment is sensitive to discretization and tiny endpoint
    // errors. Estimate terminal heading from a short window of the final path
    // points by fitting x(s) and y(s) against cumulative path distance.
    if (discrete_trajectory.size() >= 2) {
        constexpr size_t kHeadingWindowPoints = 5;
        const size_t begin = discrete_trajectory.size() > kHeadingWindowPoints
                                 ? discrete_trajectory.size() - kHeadingWindowPoints
                                 : 0;
        std::vector<double> arc_length(1, 0.0);
        arc_length.reserve(discrete_trajectory.size() - begin);
        for (size_t i = begin + 1; i < discrete_trajectory.size(); ++i) {
            const double dx = discrete_trajectory[i].x - discrete_trajectory[i - 1].x;
            const double dy = discrete_trajectory[i].y - discrete_trajectory[i - 1].y;
            arc_length.push_back(arc_length.back() + std::hypot(dx, dy));
        }

        const size_t count = arc_length.size();
        double mean_s = 0.0;
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (size_t i = 0; i < count; ++i) {
            mean_s += arc_length[i];
            mean_x += discrete_trajectory[begin + i].x;
            mean_y += discrete_trajectory[begin + i].y;
        }
        mean_s /= count;
        mean_x /= count;
        mean_y /= count;

        double direction_x = 0.0;
        double direction_y = 0.0;
        for (size_t i = 0; i < count; ++i) {
            const double centered_s = arc_length[i] - mean_s;
            direction_x += centered_s * (discrete_trajectory[begin + i].x - mean_x);
            direction_y += centered_s * (discrete_trajectory[begin + i].y - mean_y);
        }
        if (std::hypot(direction_x, direction_y) > 1e-8) {
            heading = std::atan2(direction_y, direction_x);
        }
    }
    discrete_trajectory.back().theta = static_cast<float>(heading);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryAndObstaclesPublisher>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
