#ifndef __CHUANKOU__WENJIAN__
#define __CHUANKOU__WENJIAN__

#include <stdio.h>  /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/

#include <unistd.h> /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <rclcpp/rclcpp.hpp>
#include <stdbool.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <stdint.h>
#include <map>

#define SOF1 0xE7
#define SOF2 0x7E
#define RC_CH_VALUE_OFFSET ((unsigned short)1024)

// #include "ros/ros.h"
// using namespace std;
enum Color
{
    UNKNOWN_COLOR = 0,
    RED = 1,
    BLUE = 2
};
enum CtrlKey
{
    KEY_Q = 1,
    KEY_E = 2,
    KEY_SHIFT = 3,
    KEY_CTRL = 4,
    MOUSE_LEFT = 5,
    MOUSE_RIGHT = 6
};
enum RobotID
{
    UNKNOWN_ID_ROBOT = 0,
    HERO_ID_1 = 1,
    ENGINEER_ID_2 = 2,
    STANDARD_ID_3 = 3,
    STANDARD_ID_4 = 4,
    STANDARD_ID_5 = 5,
    SENTRY_ID_7 = 7
};

class serial_node;
class serialPort
{
private:
    typedef union
    {
        float data;
        unsigned char bytes[4];
    } RxFP32Data;

    typedef union
    {
        short data;
        unsigned char bytes[2];
    } RxInt16Data;

    typedef union
    {
        bool data[8];
        unsigned char bytes[2];
    } RxRCData;

    typedef struct
    {
        struct
        {
            short ch[5];
            char s[2];
        } rc;
        struct
        {
            short x;
            short y;
            short z;
            unsigned char press_l;
            unsigned char press_r;
        } mouse;
        struct
        {
            unsigned short v;
        } key;

    } RC_ctrl_t;
    typedef struct
    {
        RxFP32Data yaw;
        RxFP32Data pitch;
        RxFP32Data roll;
        RxFP32Data BulletSpeed;
        unsigned char ID;
        unsigned char LightColor;

        unsigned char target_color;
        unsigned char mode;
        unsigned char target_robot_type; 
        RC_ctrl_t RemoteControl;
    } GimbalRx_t;
    GimbalRx_t RxMsg;
    bool Decode(const unsigned char RawData[], int You_need_cmd);
    void Encode(unsigned char RawData[], float YawData, float PitchData);

    // auto_aim::chuankou msg;
    int fd;
    bool very_ok = false;
    bool OpenPort(const char *dev);
    int setup(int speed, int flow_ctrl, int databits, int stopbits, int parity);
    struct termios Opt;
    int speed_arr[15] = {B38400, B19200, B9600, B4800, B2400, B1200, B300,
                         B38400, B19200, B9600, B4800, B2400, B1200, B300, B115200};
    int name_arr[15] = {38400, 19200, 9600, 4800, 2400, 1200, 300,
                        38400, 19200, 9600, 4800, 2400, 1200, 300, 115200};

    float serial_speed;
    float serial_yam, serial_pitch,serial_roll;

    char serial_target_color;
    unsigned char serial_mode,serial_robot_type;

    Color serial_clour;

    RobotID serial_ID;

    std::map<CtrlKey, bool> serial_map;
    void sbus_to_rc(const unsigned char *sbus_buf, RC_ctrl_t *rc_ctrl);

public:
    rclcpp::Logger roslogger;
    serialPort(serial_node* father,const char *dev);
    ~serialPort();
    serialPort(serial_node* father);
    void initPath(const char *dev);
    // void set_speed(int speed);
    // int set_Parity(int databits,int stopbits,int parity);
    void readBuffer(uint8_t *buffer, int size);
    int writeBuffer(uint8_t *buffer, int size);
    void getGimbalPose(double &gimbal_roll,double &gimbal_pitch,double &gimbal_yaw);
    char getTargetColor();
    unsigned char getMode();
    unsigned char getTargetRobotType();
    RobotID getSelfId();
    Color getSelfColor();
    float getBalletSpeed();
    std::map<CtrlKey, bool> getCtrlKeys();
    void setGimbalPose(float gimbal_yaw, float gimbal_pitch);
    uint8_t getchar();
    void getData(int You_need_cmd);//command1为寻找roll yaw pitch command2为寻找远程控制,command3为ID、灯条颜色、弹速,command4 为目标颜色、自瞄模式、待击打编号
    void ClosePort();
    bool ok();
};
#endif