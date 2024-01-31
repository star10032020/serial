#include "serial/serial_node.hpp"
#include "serial/serialtools.hpp"

serial_node::serial_node() : Node("serial_communication"),roslogger(this->get_logger())
{
    tools=nullptr;
    port="Not Set";
    //tools=new serialPort(this,port.c_str());
    this->declare_parameter<std::string>("ChuanKou","Not Set");
    while(!tools||!tools->ok()){
        RCLCPP_INFO(roslogger,"Not Found the PORT:%s!",port.c_str());
        if(tools)
        {
            tools->~serialPort();
            tools=nullptr;
        }
        this->get_parameter("ChuanKou",port);
        tools=new serialPort(this,port.c_str());
        rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 睡眠1000毫秒
    }
    msgGimbalPose = std::make_shared<auto_aim_msg::msg::GimbalPose>();
    msgAimingCtrl = std::make_shared<auto_aim_msg::msg::AimingCtrl>();
    publisher1_ = this->create_publisher<auto_aim_msg::msg::GimbalPose>("/gimbal_pose", 1);
    timer1_ = this->create_wall_timer(200us, std::bind(&serial_node::timerCallback1, this));

    publisher2_ = this->create_publisher<auto_aim_msg::msg::AimingCtrl>("/aiming_ctrl", 1);
    timer2_ = this->create_wall_timer(200us, std::bind(&serial_node::timerCallback2, this));

    gimbal_subscriber = this->create_subscription<auto_aim_msg::msg::GimbalCtrl>("/gimbal_ctrl", 1, std::bind(&serial_node::gimbalCallback, this, _1));
    autoaim_subscriber = this->create_subscription<auto_aim_msg::msg::AimingStatus>("/aiming_status", 1, std::bind(&serial_node::autoaimCallback, this, _1));
}
void serial_node::timerCallback1()
{
   double pitch=-100,roll=-100,yaw=-100;
   tools->getGimbalPose(roll,pitch,yaw);
    msgGimbalPose->roll=roll;
    msgGimbalPose->yaw=yaw;
    msgGimbalPose->pitch=pitch;
    msgGimbalPose->header.frame_id="serial";
    publisher1_->publish(*msgGimbalPose);
}

void serial_node::timerCallback2()
{
 char targetColor='F';
 unsigned char target_robot_type=0,mode=0;
 targetColor=tools->getTargetColor();
 target_robot_type=tools->getTargetRobotType();
 mode=tools->getMode();

 msgAimingCtrl->target_color=targetColor;
 msgAimingCtrl->mode=mode;
 msgAimingCtrl->target_robot_type=target_robot_type;
 msgAimingCtrl->header.frame_id="serial";
 publisher2_->publish(*msgAimingCtrl);

}

void serial_node::gimbalCallback(const auto_aim_msg::msg::GimbalCtrl::SharedPtr msg)
{
    double delta_yaw=-20,delta_pitch=-20;
    delta_yaw=msg->delta_yaw;
    delta_pitch=msg->delta_pitch;
    double gimbal_yaw=-100,gimbal_pitch=-100,gimbal_roll=-100;
    tools->getGimbalPose(gimbal_roll,gimbal_pitch,gimbal_yaw);
    tools->setGimbalPose(gimbal_yaw+delta_yaw,gimbal_pitch+delta_pitch);
}
void serial_node::autoaimCallback(const auto_aim_msg::msg::AimingStatus::SharedPtr msg)
{
    bool is_tracking=msg->is_tracking;
    float confidence=msg->confidence;
    //what to do
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serial_node>());
    rclcpp::shutdown();
    return 0;
}
