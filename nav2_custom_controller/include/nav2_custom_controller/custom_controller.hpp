// 防止头文件重复包含
#ifndef NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_

// 包含必要的头文件
#include <memory>
#include <string>
#include <vector>
#include "nav2_core/controller.hpp"        // Nav2控制器基类
#include "rclcpp/rclcpp.hpp"               // ROS2核心功能
#include "nav2_util/robot_utils.hpp"       // Nav2机器人工具

namespace nav2_custom_controller {

// 自定义控制器类，继承自Nav2的核心控制器接口
class CustomController : public nav2_core::Controller {
public:
  CustomController() = default;          // 默认构造函数
  ~CustomController() override = default; // 默认析构函数（使用override确保正确重写）

  // 配置控制器（生命周期节点配置阶段调用）
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, // 父节点的弱指针
      std::string name,                                       // 插件名称
      std::shared_ptr<tf2_ros::Buffer> tf,                    // TF变换缓冲区
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override; // 代价地图ROS接口

  void cleanup() override;     // 清理资源（生命周期节点清理阶段）
  void activate() override;    // 激活控制器（节点激活时调用）
  void deactivate() override;  // 停用控制器（节点停用时调用）

  // 核心方法：计算速度命令（由Nav2控制器管理器定期调用）
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,  // 当前机器人位姿（带时间戳）
      const geometry_msgs::msg::Twist &velocity,    // 当前机器人速度
      nav2_core::GoalChecker * goal_checker) override; // 目标检查器（判断是否到达目标）

  void setPlan(const nav_msgs::msg::Path &path) override; // 设置全局路径规划结果

  // 设置速度限制（百分比或绝对值）
  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

protected:
  // 成员变量说明
  std::string plugin_name_;           // 控制器插件名称（用于日志和参数命名空间）
  std::shared_ptr<tf2_ros::Buffer> tf_; // TF变换缓冲区（用于坐标系转换查询）
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_; // 代价地图ROS接口（包含障碍物信息）
  nav2_util::LifecycleNode::SharedPtr node_; // 生命周期节点共享指针（用于参数获取和日志）
  nav2_costmap_2d::Costmap2D *costmap_;     // 原始代价地图指针（直接访问地图数据）
  nav_msgs::msg::Path global_plan_;          // 存储全局路径（由路径规划器生成）
  double max_angular_speed_;                 // 最大角速度参数（弧度/秒）
  double max_linear_speed_;                  // 最大线速度参数（米/秒）

  // 内部工具方法：获取距离当前位置最近的路径点作为目标
  geometry_msgs::msg::PoseStamped
  getNearestTargetPose(const geometry_msgs::msg::PoseStamped &current_pose);

  // 内部工具方法：计算机器人当前朝向与目标点朝向的角度差
  double calculateAngleDifference(
      const geometry_msgs::msg::PoseStamped &current_pose,
      const geometry_msgs::msg::PoseStamped &target_pose);
};

} // namespace nav2_custom_controller

#endif // 结束头文件保护宏
