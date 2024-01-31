#pragma once
#include "rclcpp/rclcpp.hpp"
#include "auto_aim_msg/msg/gimbal_pose.hpp"
#include "auto_aim_msg/msg/aiming_ctrl.hpp"
#include "auto_aim_msg/msg/aiming_status.hpp"
#include "auto_aim_msg/msg/gimbal_ctrl.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class serialPort;

class serial_node: public rclcpp::Node {
public:
    serial_node();
    rclcpp::Logger roslogger;

private:
    std::string port;
    serialPort *tools;

    bool is_ok=false;
    bool getRightPort();
    void timerCallback0();
    void timerCallback1();
    rclcpp::TimerBase::SharedPtr timer0_;
    void timerCallback2();

    void gimbalCallback(const auto_aim_msg::msg::GimbalCtrl::SharedPtr msg);
    void autoaimCallback(const auto_aim_msg::msg::AimingStatus::SharedPtr msg);

    std::shared_ptr<auto_aim_msg::msg::GimbalPose> msgGimbalPose;
    std::shared_ptr<auto_aim_msg::msg::AimingCtrl> msgAimingCtrl;

    rclcpp::Publisher<auto_aim_msg::msg::GimbalPose>::SharedPtr publisher1_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::Publisher<auto_aim_msg::msg::AimingCtrl>::SharedPtr publisher2_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Subscription<auto_aim_msg::msg::GimbalCtrl>::SharedPtr gimbal_subscriber;
    rclcpp::Subscription<auto_aim_msg::msg::AimingStatus>::SharedPtr autoaim_subscriber;
};