#include "my_robot_interface.hpp"

namespace ros2_control_hardware
{
DCMotorHardwareInterface::DCMotorHardwareInterface()
    : motor_command_left_(0.0),
      motor_command_right_(0.0),
      motor_state_left_(0.0),
      motor_state_right_(0.0),
      linear_velocity_x_(0.0),
      angular_velocity_z_(0.0),
      max_pwm_(1023) // Assuming 10-bit PWM
{}
DCMotorHardwareInterface::~DCMotorHardwareInterface()  
{

}
hardware_interface::CallbackReturn DCMotorHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info) {
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }
    if (info_.joints.size() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("DCMotorHardwareInterface"), "Expected exactly 2 joints (left and right)");
        return hardware_interface::CallbackReturn::ERROR;
    }

    velocity_states_.reserve(info_.joints.size());
    velocity_commands_.reserve(info_.joints.size());

    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
    

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCMotorHardwareInterface::on_configure(const rclcpp_lifecycle::State&) {

}

hardware_interface::CallbackReturn DCMotorHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
    wiringPiSetupGpio();
    pinMode(pwm_pin_left_, PWM_OUTPUT);
    pinMode(dir1_pin_left_, OUTPUT);
    pinMode(dir2_pin_left_, OUTPUT);

    pinMode(pwm_pin_right_, PWM_OUTPUT);
    pinMode(dir1_pin_right_, OUTPUT);
    pinMode(dir2_pin_right_, OUTPUT);

    velocity_commands_ = {0.0, 0.0};
    velocity_states_ = {0.0, 0.0};

    try
    {    
        auto node = rclcpp::Node::make_shared("dc_motor_hardware_interface");
        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DCMotorHardwareInterface::odomCallback, this, std::placeholders::_1));
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while interacting with port " << port_);
        return CallbackReturn::FAILURE;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCMotorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
    setMotorSpeed(pwm_pin_left_, 1, 0.0);
    setMotorSpeed(pwm_pin_right_, 0, 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DCMotorHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    // Compute wheel velocities using odometry
    motor_state_left_ = linear_velocity_x_ - (angular_velocity_z_ * wheel_separation_ / 2.0);
    motor_state_right_ = linear_velocity_x_ + (angular_velocity_z_ * wheel_separation_ / 2.0);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DCMotorHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    setMotorSpeed(pwm_pin_left_, dir1_pin_left_, dir2_pin_left_, motor_command_left_);
    setMotorSpeed(pwm_pin_right_, dir1_pin_right_, dir2_pin_right_, motor_command_right_);
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DCMotorHardwareInterface::export_state_interfaces() {
    return {
        hardware_interface::StateInterface("left_wheel", "velocity", &motor_state_left_),
        hardware_interface::StateInterface("right_wheel", "velocity", &motor_state_right_)
    };
}

std::vector<hardware_interface::CommandInterface> DCMotorHardwareInterface::export_command_interfaces() {
    return {
        hardware_interface::CommandInterface("left_wheel", "velocity", &motor_command_left_),
        hardware_interface::CommandInterface("right_wheel", "velocity", &motor_command_right_)
    };
}

void DCMotorHardwareInterface::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    linear_velocity_x_ = msg->twist.twist.linear.x;
    angular_velocity_z_ = msg->twist.twist.angular.z;
}

void DCMotorHardwareInterface::setMotorSpeed(int pwm_pin, int dir1_pin, int dir2_pin, double speed) {
    int pwm_value = static_cast<int>(max_pwm_ * fabs(speed));
    pwm_value = std::min(max_pwm_, pwm_value);

    if (speed >= 0) {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
    } else {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
    }
    pwmWrite(pwm_pin, pwm_value);
}
} // namespace ros2_control_hardware
