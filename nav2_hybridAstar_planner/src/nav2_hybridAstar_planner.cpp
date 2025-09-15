#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include "tf2/utils.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_hybridAstar_planner/nav2_hybridAstar_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#define MaxCount 100000 

namespace nav2_hybridAstar_planner
{

    void HybridAstar::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        name_ = name;
        node_ = parent.lock();
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".min_turning_radius", rclcpp::ParameterValue(1.0));
        node_->get_parameter(name_ + ".min_turning_radius", min_turning_radius_);
    }

    void HybridAstar::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理 HybridAstar 规划器: %s", name_.c_str());
    }

    void HybridAstar::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活 HybridAstar 规划器: %s", name_.c_str());
    }

    void HybridAstar::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用 HybridAstar 规划器: %s", name_.c_str());
    }

    nav_msgs::msg::Path HybridAstar::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        // 1. 初始化路径
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_; // 通常是 "map" 或 "odom"

        // 2. 检查 start 和 goal 的坐标系是否与 global_frame_ 一致
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "HybridAstar: 起点坐标系错误！期望: %s, 实际: %s",
                         global_frame_.c_str(), start.header.frame_id.c_str());
            return global_path; // 坐标系不对，直接返回空路径
        }

        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "HybridAstar: 目标坐标系错误！期望: %s, 实际: %s",
                         global_frame_.c_str(), goal.header.frame_id.c_str());
            return global_path; // 坐标系不对，直接返回空路径
        }

        RCLCPP_INFO(node_->get_logger(), "HybridAstar: 开始规划，从 (%.2f, %.2f) 到 (%.2f, %.2f)",
                    start.pose.position.x, start.pose.position.y,
                    goal.pose.position.x, goal.pose.position.y);

        // 3. 检查起点和目标点是否在合法位置
        Node start_node{
            start.pose.position.x,
            start.pose.position.y,
            tf2::getYaw(start.pose.orientation),
            0.0, 0.0, nullptr};
        Node goal_node{
            goal.pose.position.x,
            goal.pose.position.y,
            tf2::getYaw(goal.pose.orientation),
            0.0, 0.0, nullptr};

        if (!isNodeSafe(start_node.x, start_node.y))
        {
            RCLCPP_ERROR(node_->get_logger(), "HybridAstar: 起点位置不安全！");
            RCLCPP_INFO(node_->get_logger(), "起点 (%.2f, %.2f) 是否安全？ --> %d",
                        start_node.x, start_node.y, isNodeSafe(start_node.x, start_node.y));
            return global_path;
        }
        if (!isNodeSafe(goal_node.x, goal_node.y))
        {
            RCLCPP_ERROR(node_->get_logger(), "HybridAstar: 终点位置不安全！");
            RCLCPP_INFO(node_->get_logger(), "目标点 (%.2f, %.2f) 是否安全？ --> %d",
                        goal_node.x, goal_node.y, isNodeSafe(goal_node.x, goal_node.y));
            return global_path;
        }

        // 4. 初始化 open_list 和 close_list
        std::vector<Node *> open_list;
        std::vector<Node *> close_list;

        // 5. 设置起点
        start_node.g_cost = 0;
        start_node.h_cost = calculateCost(start_node.x, start_node.y, goal);
        open_list.push_back(new Node(start_node));

        Node *final_node = nullptr;
        int Count = 0;

        // 6. A星主循环
        while (!open_list.empty() && Count <= MaxCount)
        {
            // 找到 f_cost 最小的节点（简单实现）
            auto current_ = open_list.begin();
            Node *current_node = *current_;

            for (auto now_ = open_list.begin(); now_ != open_list.end(); ++now_)
            {
                if ((*now_)->f_cost() < current_node->f_cost() ||
                    ((*now_)->f_cost() == current_node->f_cost() && (*now_)->h_cost < current_node->h_cost))
                {
                    current_node = *now_;
                    current_ = now_;
                }
            }

            // 7. 判断是否到达目标点
            double dx = current_node->x - goal_node.x;
            double dy = current_node->y - goal_node.y;
            if (std::sqrt(dx * dx + dy * dy) < 0.1)
            {
                RCLCPP_INFO(node_->get_logger(), "HybridAstar: 找到终点！搜索节点数 = %d", Count);
                final_node = current_node;
                break;
            }

            // 8. 移动当前节点到 close_list
            open_list.erase(current_);
            close_list.push_back(current_node);

            // 9. 生成邻居节点
            std::vector<Node> neighbors = generateNeighbors(*current_node);
            /////////////////////////////

            for (const auto &neighbor : neighbors)
            {
                RCLCPP_INFO(node_->get_logger(), "  检查邻居节点: x=%.2f, y=%.2f", neighbor.x, neighbor.y);

                if (!isNodeSafe(neighbor.x, neighbor.y))
                {
                    RCLCPP_INFO(node_->get_logger(), "  邻居节点被判定为不安全: x=%.2f, y=%.2f", neighbor.x, neighbor.y);
                    continue;
                }
                ////////////////////////////////

                ////////////////////////////////
                bool in_close = false;
                for (auto it : close_list)
                {
                    if (std::fabs(it->x - neighbor.x) < 0.1 && std::fabs(it->y - neighbor.y) < 0.1)
                    {
                        in_close = true;
                        break;
                    }
                }
                if (in_close)
                    continue;

                double new_g_cost = current_node->g_cost + std::sqrt(std::pow(neighbor.x - current_node->x, 2) +
                                                                     std::pow(neighbor.y - current_node->y, 2));

                bool is_open = false;
                for (auto it : open_list)
                {
                    if (std::fabs(it->x - neighbor.x) < 0.1 && std::fabs(it->y - neighbor.y) < 0.1)
                    {
                        if (new_g_cost < it->g_cost)
                        {
                            it->g_cost = new_g_cost;
                            it->parent = current_node;
                        }
                        is_open = true;
                        break;
                    }
                }

                if (!is_open)
                {
                    Node *new_node = new Node(neighbor);
                    new_node->g_cost = new_g_cost;
                    new_node->h_cost = calculateCost(neighbor.x, neighbor.y, goal);
                    new_node->parent = current_node;
                    open_list.push_back(new_node);
                    ///////////////////
                        RCLCPP_INFO(node_->get_logger(), "已将邻居节点加入 open_list: x=%.2f, y=%.2f, f=%.2f", 
                neighbor.x, neighbor.y, new_node->f_cost());
                }
                ///////////////////////////////////
                RCLCPP_INFO(node_->get_logger(), " 生成邻居节点: x=%.2f, y=%.2f, theta=%.2f", neighbor.x, neighbor.y, neighbor.theta);
            }

            Count++;
            ////////////////////////////////////////
            RCLCPP_INFO(node_->get_logger(), "当前 open_list 大小 = %lu, close_list 大小 = %lu, Count = %d",
                        open_list.size(), close_list.size(), Count);
        }

        // 10. 如果找到了路径，重构并返回
        if (final_node != nullptr)
        {
            RCLCPP_INFO(node_->get_logger(), "HybridAstar: 成功生成路径，共 %d 个点", /* 你可以计算路径点数量 */ 10); // 可以补充路径点计数
            std::vector<Node> path_nodes;
            Node *current = final_node;

            while (current != nullptr)
            {
                path_nodes.push_back(*current);
                current = current->parent;
            }
            std::reverse(path_nodes.begin(), path_nodes.end());

            for (const auto &node : path_nodes)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = global_path.header;
                pose.pose.position.x = node.x;
                pose.pose.position.y = node.y;
                pose.pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, node.theta);
                pose.pose.orientation = tf2::toMsg(q);

                global_path.poses.push_back(pose);
            }

            return global_path;
        }
        else
        {
            // 未找到路径时，抛出异常，而不是返回空路径
            RCLCPP_ERROR(node_->get_logger(), "HybridAstar: 未找到可行路径到目标！");
            throw nav2_core::PlannerException("HybridAstar: 未找到可行路径到目标");
        }
    }

    // 生成邻居节点 - 修正函数名拼写错误Neighbros->generateNeighbors
    std::vector<Node> HybridAstar::generateNeighbors(const Node &current)
    {
        std::vector<Node> neighbors;

        const double step_size = 0.25; // 推荐步长：0.2 ~ 0.3
        // const double angles_deg[8] = {0, 45, 90, 135, 180, 225, 270, 315};  // 8 方向
        const double angles[8] = {
            0.0,
            M_PI / 4.0,
            M_PI / 2.0,
            3 * M_PI / 4.0,
            M_PI,
            -3 * M_PI / 4.0,
            -M_PI / 2.0,
            -M_PI / 4.0};

        for (int i = 0; i < 8; ++i) // 8 方向
        {
            double angle = normalizeAngle(current.theta + angles[i]);

            Node neighbor;
            neighbor.x = current.x + step_size * std::cos(angle);
            neighbor.y = current.y + step_size * std::sin(angle);
            neighbor.theta = angle; // 转向后朝向新方向

            RCLCPP_DEBUG(node_->get_logger(), " 生成邻居节点: x=%.2f, y=%.2f, theta=%.2f",
                         neighbor.x, neighbor.y, neighbor.theta);
            neighbors.push_back(neighbor);
        }

        return neighbors;
    }

    // 检查节点是否安全（不在障碍物上）
    bool HybridAstar::isNodeSafe(double x, double y)
    {
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my))
        {
            RCLCPP_INFO(node_->get_logger(), " isNodeSafe: 坐标 (%.2f, %.2f) 超出地图范围！", x, y);
            return false;
        }

        unsigned char cost = costmap_->getCost(mx, my);
        RCLCPP_INFO(node_->get_logger(), "  检查点 (%.2f, %.2f) 的 cost = %d", x, y, cost);

        bool is_safe = (cost < nav2_costmap_2d::LETHAL_OBSTACLE);
        RCLCPP_INFO(node_->get_logger(), "  isNodeSafe(%.2f, %.2f) = %d", x, y, is_safe);
        return is_safe;
    }

    // 计算启发式代价 - 修正参数类型和计算错误
    double HybridAstar::calculateCost(double x, double y, const geometry_msgs::msg::PoseStamped &goal)
    {
        double dx = goal.pose.position.x - x;
        double dy = goal.pose.position.y - y; // 修正运算符错误：=→-
        return std::sqrt(dx * dx + dy * dy);
    }

    // 添加角度标准化函数，之前缺失
    double HybridAstar::normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_hybridAstar_planner::HybridAstar, nav2_core::GlobalPlanner)
