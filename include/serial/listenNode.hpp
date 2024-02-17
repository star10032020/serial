#pragma once
#include "rclcpp/rclcpp.hpp"
#include "auto_aim_msg/msg/gimbal_pose.hpp"
#include "auto_aim_msg/msg/aiming_ctrl.hpp"
#include "auto_aim_msg/msg/aiming_status.hpp"
#include "auto_aim_msg/msg/gimbal_ctrl.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class serialPort;

class listenNode: public rclcpp::Node {
public:
    listenNode();
    rclcpp::Logger roslogger;

private:
    rclcpp::CallbackGroup::SharedPtr m_callback_group_;
    rclcpp::SubscriptionOptions sub_opt;
    std::string port;
    serialPort *tools;
    void translate();
    void WhilePublishing();

    bool is_ok=false;
    bool getRightPort();
    void timerCallback0();
    rclcpp::TimerBase::SharedPtr timer0_;

    std::shared_ptr<auto_aim_msg::msg::GimbalPose> msgGimbalPose;
    std::shared_ptr<auto_aim_msg::msg::AimingCtrl> msgAimingCtrl;

    rclcpp::Publisher<auto_aim_msg::msg::GimbalPose>::SharedPtr publisher1_;
    rclcpp::Publisher<auto_aim_msg::msg::AimingCtrl>::SharedPtr publisher2_;
};