#ifndef NAV2_HYBRIDASTAR_PLANNER__NAV2_HYBRIDASTAR_PLANNER_HPP_
#define NAV2_HYBRIDASTAR_PLANNER__NAV2_HYBRIDASTAR_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"  // 添加TF2缓冲器头文件

namespace nav2_hybridAstar_planner
{
    /**
     * @struct Node
     * @brief 定义Hybrid A*算法中的节点结构，包含位置、角度和代价信息
     */
    struct Node
    {
        double x;        // x坐标
        double y;        // y坐标
        double theta;    // 角度(弧度)，表示机器人朝向
        double g_cost;   // 从起点到当前节点的实际代价
        double h_cost;   // 从当前节点到目标点的估计代价
        Node *parent;    // 父节点指针，用于路径回溯

        /**
         * @brief 计算节点的总代价f_cost
         * @return 总代价 = g_cost + h_cost
         */
        double f_cost() const { return g_cost + h_cost; }  // 添加const修饰符，确保const对象可调用
    };

    /**
     * @class HybridAstar
     * @brief 实现Hybrid A*路径规划算法，继承自nav2_core::GlobalPlanner接口
     */
    class HybridAstar : public nav2_core::GlobalPlanner
    {
    public:
        /**
         * @brief 构造函数
         */
        HybridAstar() = default;

        /**
         * @brief 析构函数
         */
        ~HybridAstar() override = default;  // 明确指定override

        /**
         * @brief 配置规划器
         * @param parent 生命周期节点的弱指针
         * @param name 规划器名称
         * @param tf TF缓冲器共享指针
         * @param costmap_ros 代价地图ROS包装器
         */
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, 
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        /**
         * @brief 清理规划器资源
         */
        void cleanup() override;

        /**
         * @brief 激活规划器
         */
        void activate() override;

        /**
         * @brief 停用规划器
         */
        void deactivate() override;

        /**
         * @brief 生成从起点到目标点的路径
         * @param start 起始位姿
         * @param goal 目标位姿
         * @return 生成的路径
         */
        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

    private:
        /**
         * @brief 计算启发式代价h_cost（欧氏距离）
         * @param x 当前节点x坐标
         * @param y 当前节点y坐标
         * @param goal 目标位姿
         * @return 启发式代价
         */
        double calculateCost(double x, double y, const geometry_msgs::msg::PoseStamped &goal);

        /**
         * @brief 生成当前节点的邻居节点（基于运动学约束）
         * @param current 当前节点
         * @return 邻居节点列表
         */
        std::vector<Node> generateNeighbors(const Node &current);  // 修正函数名拼写

        /**
         * @brief 标准化角度到[-π, π]范围
         * @param angle 原始角度（弧度）
         * @return 标准化后的角度
         */
        double normalizeAngle(double angle);  // 添加角度标准化函数声明

        /**
         * @brief 检查节点是否在安全区域（非障碍物）
         * @param x 节点x坐标
         * @param y 节点y坐标
         * @return 安全返回true，否则返回false
         */
        bool isNodeSafe(double x, double y);

        std::shared_ptr<tf2_ros::Buffer> tf_;  // TF缓冲器，用于坐标变换
        nav2_util::LifecycleNode::SharedPtr node_;  // 生命周期节点指针
        nav2_costmap_2d::Costmap2D *costmap_;  // 代价地图指针
        std::string global_frame_;  // 全局坐标系名称
        std::string name_;  // 规划器名称
        double interpolation_resolution_;  // 插值分辨率
        double min_turning_radius_;  // 添加最小转弯半径参数，用于生成邻居节点
    };
}  // namespace nav2_hybridAstar_planner

#endif  // NAV2_HYBRIDASTAR_PLANNER__NAV2_HYBRIDASTAR_PLANNER_HPP_
    