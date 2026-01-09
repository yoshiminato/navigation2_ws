#include "nav2_rl_controller/rl_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include <fstream>

namespace nav2_rl_controller
{
    /*
    コントローラの初期化と設定(コントローラサーバー起動時に呼ばれる)
    */
    void RLController::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
      node_ = parent;
      auto node = node_.lock();
      
      robot_ns_ = node->get_namespace();
      costmap_ros_ = costmap_ros;
      tf_ = tf;
      plugin_name_ = name;
      logger_ = node->get_logger();
      clock_ = node->get_clock();
    
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
          0.2));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist",
        rclcpp::ParameterValue(0.4));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
          1.0));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
          0.1));
        
      node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
      node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
      node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
      double transform_tolerance;
      node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
      transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

      // 強化学習からの制御入力をサブスクライブ
      std::string cmd_vel_rl_topic = robot_ns_ + "/cmd_vel_rl";
      cmd_vel_rl_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_rl_topic, 10,
        std::bind(&RLController::cmdVelRLCallback, this, std::placeholders::_1));
      
      RCLCPP_INFO(logger_, "RLController configured successfully");
      RCLCPP_INFO(logger_, "Robot namespace: %s", robot_ns_.c_str());
      RCLCPP_INFO(logger_, "Subscribed to %s for RL control input", cmd_vel_rl_topic.c_str());

      // transformed planのパブリッシャを追加
      std::string transformed_plan_topic = robot_ns_ + "/transformed_global_plan";

      transformed_pub_ = node->create_publisher<nav_msgs::msg::Path>(
          transformed_plan_topic, rclcpp::QoS(10));
    }

    /*
    グローバル経路をロボットローカル座標系に変換(プランナーから新しいグローバル経路が送られてきたとき)
    */
    void RLController::setPlan(const nav_msgs::msg::Path & path)
    {
      // Transform global path into the robot's frame
      global_plan_ = path;
      global_plan_up_to_inversion_ = path;

      // global_plan_ = transformGlobalPlan(path);
    }

    /*
    速度制限を設定する(外部？から呼ばれたとき)
    */
    void RLController::setSpeedLimit(const double & speed_limit, const bool & percentage)
    {
      // 必要に応じて速度制限を設定する処理を実装
      (void)percentage;
      desired_linear_vel_ = speed_limit;
    }

    /*
    強化学習からの制御入力コールバック
    */
    void RLController::cmdVelRLCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      latest_cmd_vel_rl_ = *msg;

      // Twistの内容をファイルに保存
      std::ofstream ofs("/tmp/cmd_vel_rl.txt", std::ios::app);
      if (ofs.is_open()) {
        ofs << "linear: [" << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z << "], "
            << "angular: [" << msg->angular.x << ", " << msg->angular.y << ", " << msg->angular.z << "]\n";
        ofs.close();
      }
    }
      

geometry_msgs::msg::TwistStamped RLController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  using nav2_util::geometry_utils::euclidean_distance;
  using nav2_util::geometry_utils::first_after_integrated_distance;
  using nav2_util::geometry_utils::min_by;

  // auto global_plan_up_to_inversion_ = global_plan_;

  auto begin = global_plan_up_to_inversion_.poses.begin();
  const double max_robot_pose_search_dist = 2.0;

  double transform_tolerance = 0.1;

  geometry_msgs::msg::PoseStamped global_pose;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, global_pose, *tf_,
      global_plan_up_to_inversion_.header.frame_id, transform_tolerance))
  {
    RCLCPP_ERROR(logger_, "Could not transform robot pose to global plan frame");
    // return geometry_msgs::msg::TwistStamped();
  }

  auto closest_pose_upper_bound =
    first_after_integrated_distance(
      global_plan_up_to_inversion_.poses.begin(),
      global_plan_up_to_inversion_.poses.end(),
      max_robot_pose_search_dist);
  
  auto closest_point = min_by(
    begin, closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);
    });

  
  // RCLCPP_ERROR(logger_, "Closest point index: %ld", std::distance(global_plan_up_to_inversion_.poses.begin(), closest_point));
  // RCLCPP_ERROR(logger_, "Global plan size: %ld", global_plan_up_to_inversion_.poses.size());
  // RCLCPP_ERROR(logger_, "Closest point position: x=%f, y=%f", closest_point->pose.position.x, closest_point->pose.position.y);


  if (closest_point != global_plan_up_to_inversion_.poses.begin()) {
    global_plan_up_to_inversion_.poses.erase(global_plan_up_to_inversion_.poses.begin(), closest_point);
  }

  // RCLCPP_ERROR(logger_, "Pruned plan size: %ld", global_plan_up_to_inversion_.poses.size());
  // RCLCPP_ERROR(logger_, "First point after pruning: x=%f, y=%f", global_plan_up_to_inversion_.poses.front().pose.position.x, global_plan_up_to_inversion_.poses.front().pose.position.y);

  // ★ロボット座標系への変換して配信
  std::string robot_base_frame = robot_ns_ + "/base_footprint";
  std::string target_frame = robot_ns_ + "/base_footprint";
  if (!robot_ns_.empty() && robot_ns_[0] == '/') {
      target_frame = robot_ns_.substr(1) + "/base_footprint";
  }
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = target_frame;
  transformed_plan.header.stamp = clock_->now();

  size_t published_points = 0;
  const size_t max_published_points = 20;  // 最大点数を制限

  for (auto & global_pose : global_plan_up_to_inversion_.poses) {
    if (published_points >= max_published_points) 
        break;
    global_pose.header.stamp = clock_->now();
    geometry_msgs::msg::PoseStamped local_pose;
    if (nav2_util::transformPoseInTargetFrame(global_pose, local_pose, *tf_, target_frame, 0.5)) {
      transformed_plan.poses.push_back(local_pose);
      published_points++;
    }
  }

  // ★重要：base_footprint座標系のパスを配信
  if (transformed_pub_) {
    transformed_pub_->publish(transformed_plan);
  }

  (void)velocity;
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist = latest_cmd_vel_rl_;

  return cmd_vel;
}

  
void RLController::cleanup() {
  RCLCPP_INFO(logger_, "RLController cleanup called");
}   

void RLController::activate() {
  RCLCPP_INFO(logger_, "RLController activated");
}

void RLController::deactivate() {
  RCLCPP_INFO(logger_, "RLController deactivated");
}

void RLController::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end) {
  // プランの不要な部分を削除する処理を実装
  plan.poses.erase(plan.poses.begin(), end);
}

nav_msgs::msg::Path RLController::transformGlobalPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return path;
  }

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = clock_->now();

  // Transform each pose in the global plan to the robot's frame
  for (const auto & pose : path.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    try {
      // global座標系→ロボット座標系変換？
      auto timeout = tf2::durationFromSec(transform_tolerance_.seconds());
      tf_->transform(pose, transformed_pose, costmap_ros_->getBaseFrameID(), timeout);
      transformed_plan.poses.push_back(transformed_pose);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Could not transform pose: %s", ex.what());
    }
  }
  publishTransformedPlan(transformed_plan);

  return transformed_plan;
}

void RLController::publishTransformedPlan(const nav_msgs::msg::Path plan)
{
    transformed_pub_->publish(plan);
}

}  // namespace nav2_rl_controller

PLUGINLIB_EXPORT_CLASS(nav2_rl_controller::RLController, nav2_core::Controller)
