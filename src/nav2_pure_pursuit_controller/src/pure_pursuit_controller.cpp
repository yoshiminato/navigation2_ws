#include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// todo 座標変換にずれが生じているのでは？
// global-local座標変換を検証　自身の位置をこの変換にかけて0になるか検証したい

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nav2_pure_pursuit_controller
{
    /*
    コントローラの初期化と設定(コントローラサーバー起動時に呼ばれる)
    */
    void PurePursuitController::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
      node_ = parent;
      auto node = node_.lock();
    
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

      transformed_pub_ = node->create_publisher<nav_msgs::msg::Path>(
          "transformed_global_plan", rclcpp::QoS(10));

    }

    /*
    グローバル経路をロボットローカル座標系に変換(プランナーから新しいグローバル経路が送られてきたとき)
    */
    void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
    {
      // Transform global path into the robot's frame
      global_plan_ = transformGlobalPlan(path);

      // Display transformed plan points (base_link座標系) to file
      std::ofstream plan_file;
      plan_file.open("/tmp/pure_pursuit_transformed_plan.txt", std::ios_base::app);
      plan_file << "[PurePursuit] Transformed plan points (base_link):\n";
      for (const auto & pose : global_plan_.poses) {
        double yaw = std::atan2(2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y),
                                1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z));
        plan_file << "  x=" << pose.pose.position.x << " y=" << pose.pose.position.y << " yaw=" << yaw << "\n";
      }
      plan_file.close();
    }

    /*
    速度制限を設定する(外部？から呼ばれたとき)
    */
    void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
    {
      // 必要に応じて速度制限を設定する処理を実装
      (void)percentage;
      desired_linear_vel_ = speed_limit;
    }

    /*
    制御コマンドを計算(デフォルト設定で20Hzで呼び出し)
    */
    geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * /*goal_checker*/)
    {
      (void)velocity;  // Suppress unused parameter warning

      // Create a default zero velocity command
      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.header.frame_id = pose.header.frame_id;
      cmd_vel.header.stamp = clock_->now();
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;

      // Check if the global plan is empty
      if (global_plan_.poses.empty()) {
        RCLCPP_ERROR(logger_, "Global plan is empty!");
        return cmd_vel;
      }

      // Log current robot pose (incoming pose) for debugging
      double qx_pose = pose.pose.orientation.x;
      double qy_pose = pose.pose.orientation.y;
      double qz_pose = pose.pose.orientation.z;
      double qw_pose = pose.pose.orientation.w;
      double yaw_pose = std::atan2(2.0 * (qw_pose * qz_pose + qx_pose * qy_pose), 1.0 - 2.0 * (qy_pose * qy_pose + qz_pose * qz_pose));
      // RCLCPP_INFO(logger_, "[PurePursuit] Current pose: x=%.3f y=%.3f yaw=%.3f", pose.pose.position.x, pose.pose.position.y, yaw_pose);

      // Find lookahead point: first forward point (x > 0) at lookahead distance
      auto goal = std::find_if(
        global_plan_.poses.begin(), global_plan_.poses.end(),
        [&](const auto & global_plan_pose) {
          return global_plan_pose.pose.position.x > 0.0 && 
                 hypot(global_plan_pose.pose.position.x,
                       global_plan_pose.pose.position.y) >= lookahead_dist_;
      });
      
      // If no suitable lookahead point found, use the last point in the plan
      if (goal == global_plan_.poses.end()) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 500, "No suitable lookahead point found, using last point in plan");
        goal = std::prev(global_plan_.poses.end());
      }
      
      auto goal_pose = goal->pose;
      

      double linear_vel, angular_vel;

      // Calculate distance to the goal pose
      double distance_to_goal = hypot(goal_pose.position.x, goal_pose.position.y);
      
      // Prevent division by zero - if too close to lookahead point, stop or use minimal angular velocity
      if (distance_to_goal < 1e-3) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 500, "Distance to goal too small: %.6f, stopping", distance_to_goal);
        linear_vel = 0.0;
        angular_vel = 0.0;
      } else {
        auto curvature = 2.0 * goal_pose.position.y / (distance_to_goal * distance_to_goal);
        linear_vel = desired_linear_vel_;
        angular_vel = desired_linear_vel_ * curvature;
      }
     
      // Update the velocity command
      cmd_vel.twist.linear.x = linear_vel;
      cmd_vel.twist.angular.z = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);


      {
        std::ofstream log_file;
        log_file.open("/tmp/pure_pursuit_log.txt", std::ios_base::app);
        geometry_msgs::msg::Pose end_pose = global_plan_.poses.back().pose;
        log_file << "Goal pose(relative): x=" << end_pose.position.x    << " y=" << end_pose.position.y << std::endl;
        log_file << "Lookahead pose:      x=" << goal_pose.position.x   << " y=" << goal_pose.position.y << std::endl;
        log_file << "Distance to goal:    " << distance_to_goal << std::endl;
        log_file << "Current pose:        x=" << pose.pose.position.x   << " y=" << pose.pose.position.y    << " yaw=" << yaw_pose << std::endl;
        log_file << "Command vel :        x=" << cmd_vel.twist.linear.x << " yaw=" << cmd_vel.twist.angular.z << std::endl;
      }

      return cmd_vel;
    }

void PurePursuitController::cleanup() {
  // transformed_pub_.reset();
}

void PurePursuitController::activate() {
  // transformed_pub_->on_activate();
}
void PurePursuitController::deactivate() {
  // transformed_pub_->on_deactivate();
}

nav_msgs::msg::Path PurePursuitController::transformGlobalPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return path;
  }

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = clock_->now();
  
  for (const auto & pose : path.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    try{
      auto timeout = tf2::durationFromSec(transform_tolerance_.seconds());
      tf_->transform(
        pose, transformed_pose, costmap_ros_->getBaseFrameID(), timeout);
      transformed_plan.poses.push_back(transformed_pose);
    }
    catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_, "Could not transform pose from %.3f to %s: %s",
        rclcpp::Time(pose.header.stamp).seconds(),
        costmap_ros_->getBaseFrameID().c_str(), ex.what());
    }
  }

  publishTransformedPlan(transformed_plan);

  return transformed_plan;
}

void PurePursuitController::publishTransformedPlan(const nav_msgs::msg::Path plan)
{
    transformed_pub_->publish(plan);
}

}  // namespace nav2_pure_pursuit_controller



PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
