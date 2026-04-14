// ros2_ws/src/state_estimation/src/height_estimator_node.cpp
#include "state_estimation/height_estimator_node.hpp"
#include "state_estimation/ros_endpoints.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

HeightEstimatorNode::HeightEstimatorNode() : Node("height_estimator_node") 
{
    alpha_rad_ = alpha_deg_ * (M_PI / 180.0);

    wand_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ros_endpoints::HEIGHT_ESTIMATOR_NODE::SUB::WAND_ANGLE, 10,
        std::bind(&HeightEstimatorNode::wand_callback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        ros_endpoints::HEIGHT_ESTIMATOR_NODE::SUB::PROCESSED_IMU_DATA, 
        rclcpp::QoS(10).best_effort(), 
        std::bind(&HeightEstimatorNode::imu_callback, this, std::placeholders::_1));

    est_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ros_endpoints::HEIGHT_ESTIMATOR_NODE::PUB::CURRENT_HEIGHT_EST, 10);    
  
    est_msg_.header.frame_id = "base_link";
    est_msg_.pose.covariance[14] = 0.01; 

    timer_ = this->create_wall_timer(
        20ms, std::bind(&HeightEstimatorNode::est_pub_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "Height Estimator Started. L=%.2fm, Rest=%.2fm, Alpha=%.1f°", wand_length_, rest_height_, alpha_deg_);
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
    double calculated_angle_rad = msg->data;

    // Limiti: 9.8 gradi (in acqua) e 90 gradi (verticale)
    double min_angle_rad = 0.171; 
    double max_angle_rad = 1.570; 

    if (calculated_angle_rad < min_angle_rad) {
        calculated_angle_rad = min_angle_rad; 
    } 
    else if (calculated_angle_rad > max_angle_rad) {
        calculated_angle_rad = max_angle_rad; 
    }

    wand_angle_rad_ = calculated_angle_rad;
}

double HeightEstimatorNode::getZest()
{
    // Stima dell'altezza del centro barca rispetto al livello dell'acqua (orientamento secondo standard NED, quindi PITCH positivo verso l'alto)
    double z_est = (wand_length_ * std::cos(alpha_rad_) * std::sin(wand_angle_rad_ - pitch_rad_) 
                  + d_ * std::sin(pitch_rad_) - height_shift_ * std::cos(pitch_rad_)) * std::cos(roll_rad_);
    return z_est;

    // CLAMP DI SICUREZZA
    if (z_est < 0.0) {
        z_est = 0.0;
    }

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
