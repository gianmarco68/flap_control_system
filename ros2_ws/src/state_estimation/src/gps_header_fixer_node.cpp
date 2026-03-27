// ros2_ws/src/state_estimation/src/gps_header_fixer_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "state_estimation/ros_endpoints.hpp"

using std::placeholders::_1;

class FixGPSHeader : public rclcpp::Node
{
  public:
    FixGPSHeader() : Node("gps_header_fixer_node")
    {
      // Niente declare_parameter, frame hardcodato in modo pulito
      frame_id_ = "base_link"; 
      
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        ros_endpoints::GPS_HEADER_FIXER_NODE::SUB::GPS_DATA, 10, std::bind(&FixGPSHeader::topic_callback, this, _1));

      // SOSTITUITO: Ora usa la macro!
      publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        ros_endpoints::GPS_HEADER_FIXER_NODE::PUB::GPS_DATA_FIXED, 10);

      RCLCPP_INFO(this->get_logger(), "Nodo gps_header_fixer avviato. Frame ID fisso: %s", frame_id_.c_str());
    }

  private:
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
      auto fixed_msg = *msg;
      fixed_msg.header.frame_id = frame_id_;
      
      // Mantiene il tempo originale
      fixed_msg.header.stamp = msg->header.stamp; 
      
      fixed_msg.position_covariance[0]=0.5;
      fixed_msg.position_covariance[4]=0.5;
      fixed_msg.position_covariance[8]=1.0;
      fixed_msg.position_covariance_type= sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      
      publisher_->publish(fixed_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixGPSHeader>());
  rclcpp::shutdown();
  return 0;
}