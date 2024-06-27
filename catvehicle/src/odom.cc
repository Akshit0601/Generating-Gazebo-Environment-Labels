#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>
#include <memory>

namespace gazebo_ros2_plugin
{
  class GazeboRos2Plugin : public gazebo::ModelPlugin
  {
  public:
    GazeboRos2Plugin() : ModelPlugin() {}

    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Create ROS node
      rclcpp::NodeOptions options;
      node_ = gazebo_ros::Node::Get(_sdf, options);

      // Get model and world
      model_ = _model;
      world_ = model_->GetWorld();

      // Subscribe to cmd_vel
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&GazeboRos2Plugin::OnCmdVel, this, std::placeholders::_1));

      // Publisher for odom
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

      // Initialize last update time
      last_update_time_ = world_->SimTime();

      // Connect to the update event
      update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRos2Plugin::OnUpdate, this));

      RCLCPP_INFO(node_->get_logger(), "Loaded GazeboRos2Plugin");
    }

  private:
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      // Update the linear and angular velocity from the cmd_vel message
      linear_vel_ = msg->linear;
      angular_vel_ = msg->angular;
    }

    void OnUpdate()
    {
      auto current_time = world_->SimTime();
      double dt = (current_time - last_update_time_).Double();

      // Update position and orientation based on velocities
      auto pose = model_->WorldPose();
      double yaw = pose.Rot().Yaw();
      pose.Pos().X() += linear_vel_.x * cos(yaw) * dt - linear_vel_.y * sin(yaw) * dt;
      pose.Pos().Y() += linear_vel_.x * sin(yaw) * dt + linear_vel_.y * cos(yaw) * dt;
      pose.Rot().Euler().Z() += angular_vel_.z * dt;
      model_->SetWorldPose(pose);

      // Create and publish odometry message
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = node_->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = pose.Pos().X();
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      odom.twist.twist.linear = linear_vel_;
      odom.twist.twist.angular = angular_vel_;

      odom_pub_->publish(odom);

      // Update the last update time
      last_update_time_ = current_time;
    }
    
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    ignition::math::Vector3d linear_vel_;
    ignition::math::Vector3d angular_vel_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time last_update_time_;
  };

  GZ_REGISTER_MODEL_PLUGIN(GazeboRos2Plugin)
}

