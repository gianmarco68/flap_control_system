// ros2_ws/src/flap_control/src/pid_controller_node.cpp
#include "flap_control/pid_controller_node.hpp"
#include "flap_control/ros_endpoints.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm> // Per std::clamp

using namespace std::chrono_literals;

namespace flap_control {

    PIDControllerNode::PIDControllerNode() : Node("pid_controller_node"){
        // 1. Inizializziamo il ConfigManager 
        config_manager_.init(this);
        auto init_cond = config_manager_.getInitialConditions();

        dt_ = init_cond.dt;

        // 2. Popoliamo le variabili di lavoro correnti
        kp_ = init_cond.kp;
        ki_ = init_cond.ki;
        kd_ = init_cond.kd;
        flap_angle_limit_min_ = init_cond.flap_angle_limit_min;
        flap_angle_limit_max_ = init_cond.flap_angle_limit_max;
        cutoff_freq_ = init_cond.cutoff_freq;
        setpoint_ = init_cond.height_target;
        feedback_ = init_cond.feedback;
        neutral_flap_angle_ = init_cond.neutral_flap_angle;
        gear_ratio_ = init_cond.gear_ratio;
        servo_offset_ = init_cond.servo_offset;

        // 3. Creiamo le istanze dei Moduli Core
        state_machine_ = std::make_unique<BoatStateMachine>(
            init_cond.sog_threshold, init_cond.roll_max, init_cond.pitch_max);

        pid_math_ = std::make_unique<PIDMath>(
            kp_, ki_, kd_, dt_, flap_angle_limit_min_, flap_angle_limit_max_, cutoff_freq_);
            
        gui_manager_ = std::make_unique<GuiManager>(config_manager_); // <-- INIZIALIZZAZIONE GUI MANAGER

        RCLCPP_INFO(this->get_logger(), "PID Node Initialized. Target: %.3f, Limits: [%.1f, %.1f]", 
                    setpoint_, flap_angle_limit_min_, flap_angle_limit_max_);

        // 4. Inizializzazione Pub/Sub
        update_sub_ = this->create_subscription<sail_msgs::msg::Update>(
            ros_endpoints::PID_CONTROLLER_NODE::SUB::LOCAL_CMD_UPDATE, 10,
            std::bind(&PIDControllerNode::updateFromGui, this, std::placeholders::_1));

        show_servo_limit_sub_ = this->create_subscription<sail_msgs::msg::ShowLimitServo>(
            ros_endpoints::PID_CONTROLLER_NODE::SUB::SHOW_SERVO_ANGLE_CMD, 10,
            std::bind(&PIDControllerNode::showLimitServoCallback, this, std::placeholders::_1));

        feedback_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            ros_endpoints::SharedTopics::CURRENT_HEIGHT_EST, 10,
            std::bind(&PIDControllerNode::heightCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            ros_endpoints::SharedTopics::IMU_DATA, 10, 
            std::bind(&PIDControllerNode::imuCallback, this, std::placeholders::_1));

        sog_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            ros_endpoints::SharedTopics::GPS_VEL_DATA, 10, 
            std::bind(&PIDControllerNode::speedCallback, this, std::placeholders::_1));

        control_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            ros_endpoints::PID_CONTROLLER_NODE::PUB::CONTROL_VALUE, 10);

        indicators_pub_ = this->create_publisher<sail_msgs::msg::Indicators>(
            ros_endpoints::PID_CONTROLLER_NODE::PUB::PID_INDICATORS, 10);

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_controller", std::bind(&PIDControllerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

        int period_ms = static_cast<int>(1000.0 * dt_);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&PIDControllerNode::timerCallback, this));

        publishIndicators();
    }

    // =================================================================================
    // LOGICA DI CONTROLLO PRINCIPALE (Il Timer)
    // =================================================================================
    void PIDControllerNode::timerCallback() {
        state_machine_->updateSensors(current_sog_, current_roll_, current_pitch_);
        
        if (state_machine_->evaluate(dt_)) {
            if (state_machine_->getState() == ControllerState::ON) {
                pid_math_->reset(); 
                RCLCPP_INFO(this->get_logger(), "PID ON: Velocita' e assetto OK.");
            } else {
                RCLCPP_INFO(this->get_logger(), "PID OFF: Condizioni non ottimali. Flap neutro.");
            }
        }

        double target_flap_angle_deg = neutral_flap_angle_;

        auto now_time = this->now();

        // LOGICA DI OVERRIDE
        if(override_active_) {
            if (now_time > override_end_time_) {
                // allo scadere del tempo disattiviamo l'override e torniamo al controllo automatico
                override_active_ = false;
                RCLCPP_INFO(this->get_logger(), "OVERRIDE SCADUTO: Tornando al controllo automatico.");
            } else {
                if (override_type_ == sail_msgs::msg::ShowLimitServo::SERVO_MIN) {
                    target_flap_angle_deg = flap_angle_limit_min_;
                    // RCLCPP_INFO(this->get_logger(), "OVERRIDE ATTIVO: Flap al limite minimo (%.1f deg)", target_flap_angle_deg);
                } else if (override_type_ == sail_msgs::msg::ShowLimitServo::SERVO_MAX) {
                    target_flap_angle_deg = flap_angle_limit_max_;
                    // RCLCPP_INFO(this->get_logger(), "OVERRIDE ATTIVO: Flap al limite massimo (%.1f deg)", target_flap_angle_deg);
                }
            }
        }

        // CALCOLO PID solo se non c'è override e la macchina a stati è ON
        if (!override_active_ && state_machine_->getState() == ControllerState::ON) {
            double correction_deg = pid_math_->compute(setpoint_, feedback_);
            target_flap_angle_deg = neutral_flap_angle_ + correction_deg;
        }

        target_flap_angle_deg = std::clamp(target_flap_angle_deg, flap_angle_limit_min_, flap_angle_limit_max_);
        double servo_angle_output_deg = flapToServoAngle(target_flap_angle_deg);

        auto msg = std_msgs::msg::Float64();
        msg.data = static_cast<double>(servo_angle_output_deg);
        control_pub_->publish(msg);

        publishIndicators();
    }

    // =================================================================================
    // CALLBACKS SENSORI
    // =================================================================================
    void PIDControllerNode::heightCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        feedback_ = msg->pose.pose.position.z;
    }

    void PIDControllerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto& q_msg = msg->orientation;
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_roll_ = roll * 180.0 / M_PI; 
        current_pitch_ = pitch * 180.0 / M_PI; 
    }

    void PIDControllerNode::speedCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_sog_ = std::sqrt(vx*vx + vy*vy) * 1.94384;          // Convertiamo da m/s a nodi
    }

    // callback override manuale del servo
    void PIDControllerNode::showLimitServoCallback(const sail_msgs::msg::ShowLimitServo::SharedPtr msg) {
        // il comando di override viene accettato solo se non c'è già un override attivo o se il nuovo comando è diverso da quello attivo
        if (!override_active_ || (override_active_ && msg->type != override_type_)) {
            override_active_ = true;

            override_end_time_ = this->now() + rclcpp::Duration(20, 0);
            override_type_ = msg->type;

            RCLCPP_INFO(this->get_logger(), "OVERRIDE ATTIVATO (20s): Forzatura a %s",
                        (override_type_ == sail_msgs::msg::ShowLimitServo::SERVO_MIN)  ? "SERVO_MIN" : "SERVO_MAX");
        }
    }
        
    // =================================================================================
    // GESTIONE GUI
    // =================================================================================
    void PIDControllerNode::updateFromGui(const sail_msgs::msg::Update::SharedPtr msg) {
        // Deleghiamo tutta la logica al manager!
        bool changed = gui_manager_->processUpdate(
            msg, kp_, ki_, kd_, setpoint_, flap_angle_limit_min_, flap_angle_limit_max_);

        if (changed) {
            pid_math_->updateCoefficients(kp_, ki_, kd_, flap_angle_limit_min_, flap_angle_limit_max_, cutoff_freq_);
            publishIndicators();
        }
    }

    void PIDControllerNode::publishIndicators() {
        // Deleghiamo la creazione del messaggio
        auto msg = gui_manager_->generateIndicatorsMessage(
            kp_, ki_, kd_, setpoint_, flap_angle_limit_min_, flap_angle_limit_max_);
        indicators_pub_->publish(msg);
    }

    // =================================================================================
    // HELPERS & SERVICES
    // =================================================================================
    double PIDControllerNode::flapToServoAngle(double flap_angle) {
        return (flap_angle * gear_ratio_) + servo_offset_;
    }

    void PIDControllerNode::resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        pid_math_->reset();
        res->success = true;
        res->message = "PID resettato.";
    }

} // namespace flap_control

// === MAIN ===
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flap_control::PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}