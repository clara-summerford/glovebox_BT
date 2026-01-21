#pragma once

#include <optional>
#include <future> 

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class MoveArmToPose : public BT::StatefulActionNode
{
public:
    MoveArmToPose(
        const std::string& name, 
        const BT::NodeConfig& config,
        std::weak_ptr<rclcpp::Node> node_ptr = {},
        tf2_ros::Buffer::SharedPtr tf_buffer = nullptr
        );

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;


protected:
    // ROS
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::optional<tf2_ros::TransformListener> tf_listener_;
    std::weak_ptr<rclcpp::Node> node_ptr_;

    // MoveIt
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

};

