// ros2_ws/src/state_estimation/src/height_estimator_node.cpp
#include "state_estimation/height_estimator_node.hpp"
#include "state_estimation/ros_endpoints.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

HeightEstimatorNode::HeightEstimatorNode() : Node("height_estimator_node") 
{
    // 1. Inizializzazione Matematica
    // Convertiamo l'angolo hardcodato in radianti una volta sola
    alpha_rad_ = alpha_deg_ * (M_PI / 180.0); 

    // 2. Sottoscrizioni (USANDO LO SCOPE DEL NODO INVECE DI SHARED)
    wand_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ros_endpoints::HEIGHT_ESTIMATOR_NODE::SUB::WAND_ANGLE, 10,
        std::bind(&HeightEstimatorNode::wand_callback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        ros_endpoints::HEIGHT_ESTIMATOR_NODE::SUB::IMU_DATA, 10,
        std::bind(&HeightEstimatorNode::imu_callback, this, _1));

    // 3. Pubblicazioni (USANDO LO SCOPE DEL NODO)
    est_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ros_endpoints::HEIGHT_ESTIMATOR_NODE::PUB::CURRENT_HEIGHT_EST, 10);    
  
    // Setup messaggio statico
    est_msg_.header.frame_id = "base_link";
    est_msg_.pose.covariance[14] = 0.01; 

    // 4. Timer (50Hz)
    timer_ = this->create_wall_timer(
        20ms, std::bind(&HeightEstimatorNode::est_pub_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "Height Estimator Started. Hardcoded length: %.2f", wand_length_);
}

void HeightEstimatorNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->orientation.x, 
        msg->orientation.y, 
        msg->orientation.z, 
        msg->orientation.w);
      
    double roll, pitch, yaw; 
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
    roll_rad_ = roll;
    pitch_rad_ = pitch;
}

void HeightEstimatorNode::wand_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double wand_angle_deg = msg->data;
    wand_angle_rad_ = wand_angle_deg * (M_PI / 180.0);
}

double HeightEstimatorNode::getZest()
{
    double z_est = (wand_length_ * std::cos(alpha_rad_) * std::sin(wand_angle_rad_ - pitch_rad_) 
                  + d_ * std::sin(pitch_rad_)) * std::cos(roll_rad_);
    return z_est;
}

void HeightEstimatorNode::est_pub_callback()
{
    est_msg_.header.stamp = this->now();
    est_msg_.pose.pose.position.z = getZest();
    est_pub_->publish(est_msg_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
