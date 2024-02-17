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
        bool sub_tag=false;
        int state=0;
        int not_update_timing=0;
        int cold_thresh=1000.0/80.0*3+10;//1000:定时器帧率 80：理论上的gimctrl话题帧率.

        double delta_yaw=0.00;
        double delta_pitch=0.00;

        double failed_yaw=0.00;
        double failed_pitch=0.00;
    }gimctrl;
    struct AimStatu{
        bool sub_tag=false;
        int state=0;
        int not_update_timing=0;
        int cold_thresh=1000.0/80.0*3+100;
        int want_to_shoot_timing=0;
        int want_to_shoot_thresh=1000.0*0.7;//0.7s持续想要开火

        int not_want_to_shoot_thresh=1000.0/80.0*6;
        int not_want_to_shoot_timing=0;

        float confidence_thresh=0.75;

        
        float confidence=0.00;
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