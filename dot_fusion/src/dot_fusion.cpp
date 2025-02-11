#include "dot_fusion/dot_fusion.hpp"


//Code based on https://stackoverflow.com/questions/79134268/how-to-transform-imu-data-from-sensor-frame-to-baselink-frame
namespace dot_fusion
{
  using namespace std;
	using std::placeholders::_1;
	using namespace std::string_literals;

  /// @brief Class constructor
  DotFusion::DotFusion(void) : Node("dot_fusion"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter("sub_topic", "/IMU");
    this->declare_parameter("pub_topic", "/pose");
    this->get_parameter("sub_topic",sub_topic_);
    this->get_parameter("pub_topic",pub_topic_);

    //Define a compatible QoS profile
    auto qos_profile = rclcpp::SystemDefaultsQoS();

    //Publishers setup
		rclcpp::PublisherOptions pub_options;
		pub_options.qos_overriding_options =
				rclcpp::QosOverridingOptions::with_default_policies();	// Allow overriding QoS settings (history, depth, reliability)

    this->imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(sub_topic_, qos_profile, bind(&DotFusion::ImuCallback, this, _1));
    this->posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pub_topic_, qos_profile, pub_options);
  }

  void DotFusion::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    //Get topic, transform to pose, and publishs
    auto poseMsg = this->IMU2Pose(msg);
    posePub_->publish(poseMsg);
  }

  geometry_msgs::msg::PoseStamped DotFusion::IMU2Pose(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.stamp = this->get_clock()->now();
    poseMsg.header.frame_id = msg->header.frame_id;
    poseMsg.pose.position.x = 0;
    poseMsg.pose.position.y = 0;
    poseMsg.pose.position.z = 0;
    poseMsg.pose.orientation = msg->orientation;
    return poseMsg;
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<dot_fusion::DotFusion>());
  rclcpp::shutdown();
  return 0;
}
