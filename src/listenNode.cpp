#include "serial/listenNode.hpp"
#include "serial/serialtools.hpp"

//listenNode负责从下位机获取信息并进行后续操作
bool listenNode::getRightPort()
{
    return is_ok;
}
void listenNode::timerCallback0()
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

void listenNode::translate()
{
    if (!this->getRightPort())
    {
        return;
    }
    int cmd_id = this->tools->newGetData();
    // 云台数据
    if (cmd_id == 1)
    {
        double yaw, pitch, roll;
        yaw = tools->readYaw;
        pitch = tools->readPitch;
        roll = tools->readRoll;

        msgGimbalPose->roll = roll;
        msgGimbalPose->yaw = yaw;
        msgGimbalPose->pitch = pitch;
        msgGimbalPose->header.frame_id = "serial";
        publisher1_->publish(*msgGimbalPose);
    }
    // 自瞄数据
    else if (cmd_id == 2)
    {
        unsigned char ID, Color, attackMode, detectMode;
        ID = tools->readSelfID;
        Color = tools->readSelfColor;
        attackMode = tools->readAttackMode;
        detectMode = tools->readDetectMode;

        char targetColor;
        if (Color == 0x00) // 蓝
        {
            if (detectMode == 0x02) // 是能量机关
                targetColor = 'B';
            else // 禁用 普通 吊射
                targetColor = 'R';
        }
        else if (Color == 0x01) // 红
        {
            if (detectMode == 0x02) // 是能量机关
                targetColor = 'R';
            else // 禁用 普通 吊射
                targetColor = 'B';
        }
        // targetColor = 'U'; // 测试阶段

        msgAimingCtrl->target_color = targetColor;
        msgAimingCtrl->mode = detectMode;        // 该mode为识别模式
        msgAimingCtrl->target_robot_type = 0x00; // 测试阶段默认值
        msgAimingCtrl->header.frame_id = "serial";
        publisher2_->publish(*msgAimingCtrl);
    }
    else
    {
        RCLCPP_INFO(roslogger, "Wrong cmd_id when translate!");
    }
}
void listenNode::WhilePublishing(){
    while(rclcpp::ok()){
        this->translate();
    }
}

listenNode::listenNode() : Node("serial_communication_up"), roslogger(this->get_logger())
{
    tools = nullptr;
    port = "Not Set";
    // tools=new serialPort(this,port.c_str());
    this->declare_parameter<std::string>("ChuanKou", "Not Set");
    msgGimbalPose = std::make_shared<auto_aim_msg::msg::GimbalPose>();
    msgAimingCtrl = std::make_shared<auto_aim_msg::msg::AimingCtrl>();
    m_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group_;

    // 用来确定串口名称
    timer0_ = this->create_wall_timer(1000ms, std::bind(&listenNode::timerCallback0, this),m_callback_group_);

    // 下位机->上位机 云台数据
    publisher1_ = this->create_publisher<auto_aim_msg::msg::GimbalPose>("/gimbal_pose", 1);

    // 下位机->上位机 自瞄信息
    publisher2_ = this->create_publisher<auto_aim_msg::msg::AimingCtrl>("/aiming_ctrl", 1);

    //使用while发送消息
    WhilePublishing();
}