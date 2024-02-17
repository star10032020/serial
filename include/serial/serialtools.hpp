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

#include<queue>

#define down_to_up_SOF1 0xE7
#define down_to_up_SOF2 0x7E


#define up_to_down_SOF1 0xA5
#define up_to_down_SOF2 0x5A

class listenNode;
class talkerNode;
class serialPort
{
private:
    
    int dataLenth[3] = {0, 27, 7};

    unsigned char Data[1024];
    unsigned char WriteData[1024];
    int WriteDataLenth=23;

    typedef union
    {
        float data;
        unsigned char bytes[4];
    } RxFP32Data;
    
    typedef union
    {
        double data;
        unsigned char bytes[8];
    } RxFP64Data;

    typedef union
    {
        short data;
        unsigned char bytes[2];
    } RxInt16Data;

    typedef struct
    {
        RxFP64Data yaw;
        RxFP64Data pitch;
        RxFP64Data roll;

        unsigned char self_ID;

        unsigned char self_color;
        unsigned char DetectMode;
        unsigned char AttackMode; 
    } GimbalRx_t;
    GimbalRx_t RxMsg;
    bool Decode(const unsigned char RawData[]);
    void Encode(unsigned char RawData[], double YawData, double PitchData,unsigned char FireMode,float confidence);

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
public:
    double readYaw,readPitch,readRoll;
    unsigned char readSelfColor,readSelfID,readDetectMode,readAttackMode;

        
    rclcpp::Logger roslogger;
    serialPort(listenNode* father,const char *dev);
    serialPort(talkerNode* father,const char *dev);
    ~serialPort();
    void initPath(const char *dev);
    // void set_speed(int speed);
    // int set_Parity(int databits,int stopbits,int parity);
    void readBuffer(uint8_t *buffer, int size);
    int writeBuffer(uint8_t *buffer, int size);

    //一个字节一个字节的读，直到得到规范数据后交给Decode解码,返回命令码
    int newGetData();
 
    void setRosData(double relate_yaw, double relate_pitch,unsigned char FireMode,float confidence);
    uint8_t getchar();
    void ClosePort();
    bool ok();
};
#endif