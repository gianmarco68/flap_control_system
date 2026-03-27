// ros2_ws/src/state_estimation/include/state_estimation/height_estimator_node.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp" // Il fix fondamentale per la wand

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class HeightEstimatorNode : public rclcpp::Node
{
    public:
        HeightEstimatorNode();

    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void wand_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void est_pub_callback();

        double getZest();

        // --- Sottoscrizioni e Pubblicazioni ---
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wand_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr est_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        // --- Variabili di Stato (in Radianti) ---
        double wand_angle_rad_{0.0};
        double roll_rad_{0.0};
        double pitch_rad_{0.0};
    
        // --- Parametri Fissi (Compile-time) ---
        double d_{1.74};
        double alpha_deg_{15.0};
        double alpha_rad_{0.0}; // Verrà calcolato una sola volta nel costruttore
        double wand_length_{1.05};

        geometry_msgs::msg::PoseWithCovarianceStamped est_msg_;
};