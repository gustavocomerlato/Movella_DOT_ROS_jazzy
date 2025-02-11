#ifndef DOT_FUSION_HPP
#define DOT_FUSION_HPP

//ROS
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <movella_msgs/msg/dot_sensor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

//Libraries
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // Updated header file
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <eigen3/Eigen/Core>


namespace dot_fusion
{
  class DotFusion : public rclcpp::Node
  {
  public:
    DotFusion(void);
    // ~DotFusion(void);

    geometry_msgs::msg::PoseStamped IMU2Pose(const sensor_msgs::msg::Imu::SharedPtr msg);

  private:

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // TF2 related objects
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    //Parameters
    std::string pub_topic_;
    std::string sub_topic_;
  };
}
#endif