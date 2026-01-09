#ifndef NAV2_RL_CONTROLLER__RL_CONTROLLER_HPP_
#define NAV2_RL_CONTROLLER__RL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"


namespace nav2_rl_controller
{

using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;

class RLController : public nav2_core::Controller
{
public:
  RLController() = default;
  ~RLController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;


  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;


  bool cancel() override { return true; }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // transformed planをパブリッシュする関数を追加
  void publishTransformedPlan(const nav_msgs::msg::Path plan);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string robot_ns_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("RLController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_inversion_;

  nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path & plan);
  void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

  double desired_linear_vel_{0.2};
  double lookahead_dist_{0.4};
  double max_angular_vel_{1.0};
  rclcpp::Duration transform_tolerance_{rclcpp::Duration::from_seconds(0.1)};

  // 強化学習からの制御入力用
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_rl_sub_;
  geometry_msgs::msg::Twist latest_cmd_vel_rl_;
  void cmdVelRLCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

protected:
  // transformed planのパブリッシャを追加
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> transformed_pub_;
};

}  // namespace nav2_rl_controller

#endif
