#pragma once
#include "rclcpp/rclcpp.hpp"
#include "auto_aim_msg/msg/gimbal_pose.hpp"
#include "auto_aim_msg/msg/aiming_ctrl.hpp"
#include "auto_aim_msg/msg/aiming_status.hpp"
#include "auto_aim_msg/msg/gimbal_ctrl.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class serialPort;

class talkerNode: public rclcpp::Node {
public:
    talkerNode();
    rclcpp::Logger roslogger;

private:
    rclcpp::CallbackGroup::SharedPtr m_callback_group_;
    rclcpp::SubscriptionOptions sub_opt;
    struct GimCtrl{
        bool sub_tag;
        int state;
        int not_update_timing;
        int cold_thresh;

        double delta_yaw;
        double delta_pitch;

        double failed_yaw=0.00;
        double failed_pitch=0.00;
    }gimctrl;
    struct AimStatu{
        bool sub_tag;
        int state;
        int not_update_timing;
        int cold_thresh;
        int want_to_shoot_timing;
        int want_to_shoot_thresh;

        int not_want_to_shoot_thresh;
        int not_want_to_shoot_timing;

        float confidence_thresh;

        
        float confidence;
    }aimstatu;

    void updateData();

    std::string port;
    serialPort *tools;


    bool is_ok=false;
    bool getRightPort();
    void timerCallback0();
    rclcpp::TimerBase::SharedPtr timer0_;

    void gimbalCallback(const auto_aim_msg::msg::GimbalCtrl::SharedPtr msg);
    void autoaimCallback(const auto_aim_msg::msg::AimingStatus::SharedPtr msg);

    rclcpp::Subscription<auto_aim_msg::msg::GimbalCtrl>::SharedPtr gimbal_subscriber;
    rclcpp::Subscription<auto_aim_msg::msg::AimingStatus>::SharedPtr autoaim_subscriber;
    rclcpp::TimerBase::SharedPtr timer3_;
};