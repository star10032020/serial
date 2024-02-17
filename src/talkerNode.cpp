#include "serial/talkerNode.hpp"
#include "serial/serialtools.hpp"
//talkerNode负责向下位机写消息
bool talkerNode::getRightPort()
{
    return is_ok;
}
void talkerNode::timerCallback0()
{
    if (this->getRightPort())
        return;
    if (port == "Not Set" || !tools || !tools->ok())
    {
        RCLCPP_INFO(roslogger, "Not Found the PORT:%s!", port.c_str());
        if (tools)
        {
            tools->~serialPort();
            tools = nullptr;
        }
        this->get_parameter("ChuanKou", port);
        if (port != "Not Set")
            tools = new serialPort(this, port.c_str());
        // rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 睡眠1000毫秒
        if (tools && tools->ok())
        {
            RCLCPP_INFO(roslogger, "the port is %s,and successfully get it", port.c_str());
            is_ok = true;
            return;
        }

        return;
    }
    is_ok = true;
    return;
}

talkerNode::talkerNode() : Node("serial_communication_down"), roslogger(this->get_logger())
{
    tools = nullptr;
    port = "Not Set";
    // tools=new serialPort(this,port.c_str());
    this->declare_parameter<std::string>("ChuanKou", "Not Set");
    m_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group_;

    // 用来确定串口名称
    timer0_ = this->create_wall_timer(1000ms, std::bind(&talkerNode::timerCallback0, this),m_callback_group_);


    // 订阅云台偏移量 与 自瞄自信度
    gimbal_subscriber = this->create_subscription<auto_aim_msg::msg::GimbalCtrl>("/gimbal_ctrl", 1, std::bind(&talkerNode::gimbalCallback, this, _1),sub_opt);
    autoaim_subscriber = this->create_subscription<auto_aim_msg::msg::AimingStatus>("/aiming_status", 1, std::bind(&talkerNode::autoaimCallback, this, _1),sub_opt);

    //持续向下位机发信
    timer3_ = this->create_wall_timer(200us, std::bind(&talkerNode::updateData, this),m_callback_group_);
}

void talkerNode::updateData()
{
    if (!this->getRightPort())
        return;
    double gim_pitch_want, gim_yaw_want;
    if (gimctrl.sub_tag)
    { // 若有了更新，冷却重置
        gimctrl.sub_tag = false;
        gimctrl.not_update_timing = 0;
    }
    if (gimctrl.not_update_timing >= gimctrl.cold_thresh)
    {
        // 过长时间没接受新云台请求，视为更新请求过期，不再更新云台
        gim_pitch_want = gimctrl.failed_pitch;
        gim_yaw_want = gimctrl.failed_yaw;
    }
    else
    {
        // 请求没有失效
        gim_pitch_want = gimctrl.delta_pitch;
        gim_yaw_want = gimctrl.delta_yaw;

        // 冷却加1
        gimctrl.not_update_timing++;
    }

    unsigned char fire_want;
    float confidence_want;
    // 注意线程冲突
    if (aimstatu.sub_tag)
    { // 若有了更新，冷却重置
        aimstatu.sub_tag = false;
        aimstatu.not_update_timing = 0;
    }
    if (aimstatu.not_update_timing >= aimstatu.cold_thresh)
    {
        // 过长时间没接受新自瞄追踪请求，视为目标已脱靶，停火
        fire_want = 0x00;
        confidence_want = 0.00;

        // 直接重置冷却
        aimstatu.want_to_shoot_timing = 0;
        aimstatu.not_want_to_shoot_timing = 0;
    }
    else
    {
        // 追踪没有失效
        aimstatu.not_update_timing++;
        confidence_want = aimstatu.confidence;

        if (aimstatu.confidence > aimstatu.confidence_thresh)
        {
            // 仍要继续开火
            // 冷却机制刷新
            aimstatu.not_want_to_shoot_timing = 0;
            if (aimstatu.want_to_shoot_timing >= aimstatu.want_to_shoot_thresh)
            {
                // 处于连射区间
                fire_want = 0x02;
            }
            else
            {
                fire_want = 0x01;
                // 开火继续升温
                aimstatu.want_to_shoot_timing++;
            }
        }
        else
        {
            // 当前还未继续开火
            // 判断阈值，超出阈值缓慢冷却,这个阈值应该设的大一点
            if (aimstatu.not_want_to_shoot_timing >= aimstatu.not_want_to_shoot_thresh)
            {
                if (aimstatu.want_to_shoot_timing >= 1)
                {
                    aimstatu.want_to_shoot_timing--;
                }
                else
                {
                    aimstatu.want_to_shoot_timing = 0;
                }
                // 重置冷却槽，准备进入下一次冷却判断
                aimstatu.not_want_to_shoot_timing = 0;
            }
            else
            {
                aimstatu.not_want_to_shoot_timing++;
            }
            // 忘了设置FIREMODE了
            if (aimstatu.want_to_shoot_timing >= aimstatu.want_to_shoot_thresh)
            {
                // 处于连射区间
                fire_want = 0x02;
            }
            else
            {
                fire_want = 0x01;
            }
        }
    }   

    this->tools->setRosData(gim_yaw_want, gim_pitch_want, fire_want, confidence_want);
}

void talkerNode::gimbalCallback(const auto_aim_msg::msg::GimbalCtrl::SharedPtr msg)
{
    if (!this->getRightPort())
        return;
    double delta_yaw = -20, delta_pitch = -20;
    delta_yaw = msg->delta_yaw;
    delta_pitch = msg->delta_pitch;

    gimctrl.delta_yaw=delta_yaw;
    gimctrl.delta_pitch=delta_pitch;
    gimctrl.sub_tag=true;
}
void talkerNode::autoaimCallback(const auto_aim_msg::msg::AimingStatus::SharedPtr msg)
{
    if (!this->getRightPort())
        return;
    bool is_tracking = msg->is_tracking;
    float confidence = msg->confidence;
    // what to do

    aimstatu.confidence=confidence;
    aimstatu.sub_tag=true;
}