#include "serial/serialtools.hpp"
#include <stdexcept>
#include <string.h>
#include <sys/ioctl.h>
// using namespace std;
#include "serial/listenNode.hpp"
#include "serial/talkerNode.hpp"
int serialPort::newGetData()
{
    tcflush(fd,TCIFLUSH);
    int state = 0;
    unsigned char cmd_id = 0;

    int readDataLenth = 0;
    while (true)
    {
        if (state == 100)
            break;
        unsigned char ch = this->getchar();
        switch (state)
        {
        case 0:
            if (ch == down_to_up_SOF1)
            {
                state = 1;
                Data[readDataLenth++] = ch;
            }
            else
            {
                state = 0;
                cmd_id = 0x00;
                readDataLenth = 0;
            }
            break;
        case 1:
            if (ch == down_to_up_SOF2)
            {
                state = 2;
                Data[readDataLenth++] = ch;
            }
            else
            {
                state = 0;
                cmd_id = 0x00;
                readDataLenth = 0;
            }
            break;
        case 2:
            if (ch == 0x01 || ch == 0x02)
            {
                cmd_id = ch;
                state = 3;
                Data[readDataLenth++] = ch;
            }
            else
            {
                state = 0;
                cmd_id = 0x00;
                readDataLenth = 0;
            }
            break;
        case 3:
            if (dataLenth[int(cmd_id)] == readDataLenth)
            {
                state = 100;
            }
            else
            {
                Data[readDataLenth++] = ch;
            }
            break;
        default:
            state = 100;
            cmd_id = 0x00;
            readDataLenth = 0;
            break;
        }
    }
    if (cmd_id = 0x00)
    {
        RCLCPP_INFO(roslogger, "erro happened in the newGetData");
        return newGetData();
    }
    if(!this->Decode(Data))
    {
        RCLCPP_INFO(roslogger, "erro happened in the time when we want decode");
        return newGetData();
    }
    if(cmd_id==0x01){
        readYaw=RxMsg.yaw.data;
        readPitch=RxMsg.pitch.data;
        readRoll=RxMsg.roll.data;
    }else if(cmd_id==0x02){
        readSelfColor=RxMsg.self_color;
        readSelfID=RxMsg.self_ID;
        readAttackMode=RxMsg.AttackMode;
        readDetectMode=RxMsg.DetectMode;
    }else{
        RCLCPP_INFO(roslogger,"Wrong cmd_id in the newGetData!");
    }
    return (int)cmd_id;
}
serialPort::~serialPort()
{
    this->ClosePort();
}
serialPort::serialPort(listenNode *father, const char *dev) : roslogger(father->roslogger)
{
    this->fd = -1;
    this->very_ok = false;
    RCLCPP_INFO(roslogger, "new dev:%s", dev);
    if (!OpenPort(dev))
    {

        // printf("open failed;\n serial BAD!\n");
        RCLCPP_INFO(roslogger, "open failed; \t serial BAD!");
    }
    else
    {
        if (!setup(115200, 0, 8, 1, 'N'))
        {
            // printf("setup failed;\n serial BAD!\n");
            RCLCPP_INFO(roslogger, "setup failed; \t serial BAD!");
        }
        else
        {
            // printf("setup is ok");
            // printf("open and setup ok.let us read.\n");
            RCLCPP_INFO(roslogger, "setup is ok...");
            RCLCPP_INFO(roslogger, "open and setup ok. \t let us read.");
            very_ok = true;
        }
    }
}
serialPort::serialPort(talkerNode *father, const char *dev) : roslogger(father->roslogger)
{
    this->fd = -1;
    this->very_ok = false;
    RCLCPP_INFO(roslogger, "new dev:%s", dev);
    if (!OpenPort(dev))
    {

        // printf("open failed;\n serial BAD!\n");
        RCLCPP_INFO(roslogger, "open failed; \t serial BAD!");
    }
    else
    {
        if (!setup(115200, 0, 8, 1, 'N'))
        {
            // printf("setup failed;\n serial BAD!\n");
            RCLCPP_INFO(roslogger, "setup failed; \t serial BAD!");
        }
        else
        {
            // printf("setup is ok");
            // printf("open and setup ok.let us read.\n");
            RCLCPP_INFO(roslogger, "setup is ok...");
            RCLCPP_INFO(roslogger, "open and setup ok. \t let us read.");
            very_ok = true;
        }
    }
}
void serialPort::initPath(const char *dev)
{
    // this->fd = -1;
    // this->very_ok = false;
    if (this->very_ok)
    {
        // printf("You have make it.Do not do it again!\n");
        RCLCPP_INFO(roslogger, "You have read it.Do not do it again!");
        return;
    }

    if (!OpenPort(dev))
    {

        // printf("open failed;\n serial BAD!\n");
        RCLCPP_INFO(roslogger, "open failed; \t serial BAD!");
    }
    else
    {
        // printf("Openport is ok.\n");
        RCLCPP_INFO(roslogger, "Openport is ok.");
        if (!setup(115200, 0, 8, 1, 'N'))
        {
            // printf("setup failed;\n serial BAD!\n");
            RCLCPP_INFO(roslogger, "setup failed \t serial BAD!");
        }
        else
        {
            // printf("setup is ok.\n");
            // printf("open and setup ok.let us read.\n");
            RCLCPP_INFO(roslogger, "setup is ok...");
            RCLCPP_INFO(roslogger, "open and setup ok. \t let us read.");
            very_ok = true;
        }
    }
}

void serialPort::setRosData(double relate_yaw, double relate_pitch,unsigned char FireMode,float confidence)
{
    tcflush(fd, TCOFLUSH);
    if (!this->very_ok)
    {
        RCLCPP_INFO(roslogger, "Oh NO! \t serialPort may not been open or setup correctly!");
        // printf("Oh NO! serialPort may not been open or setup correctly!\n");
        return;
    }
    Encode(this->WriteData, relate_yaw, relate_pitch,FireMode,confidence);
    writeBuffer(this->WriteData, this->WriteDataLenth);
    // printf("we have write.\n");
}

bool serialPort::OpenPort(const char *dev)
{

    char *_dev = new char[64];
    strcpy(_dev, dev);
    fd = open(_dev, O_RDWR); //| O_NOCTTY | O_NDELAY
    if (-1 == fd)
    {
        printf("dev is %s,", _dev);
        perror("Can't Open the serialPOrt");
        return false;
    }

    int DTR_flag;
    DTR_flag = TIOCM_DTR;
    ioctl(fd, TIOCMBIS, &DTR_flag); // Set RTS pin
    return true;
}
int serialPort::setup(int speed, int flow_ctrl, int databits, int stopbits, int parity)
{

    int i;
    int status;
    struct termios options;
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
     */
    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return (false);
    }

    // 设置串口输入波特率和输出波特率
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    // 修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    // 修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    // 设置数据流控制
    switch (flow_ctrl)
    {

    case 0: // 不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1: // 使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2: // 使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    // 设置数据位
    // 屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return (false);
    }
    // 设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': // 无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O': // 设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': // 设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': // 设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return (false);
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return (false);
    }

    // 修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // options.c_lflag &= ~(ISrIG | ICANON);

    // 设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 80; /* 读取字符的最少个数为1 */
    // 如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd, TCIFLUSH);

    // 激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("com set error!\n");
        return (false);
    }
    return (true);
}

void serialPort::readBuffer(uint8_t *buffer, int size)
{
    read(fd, buffer, size);
}
int serialPort::writeBuffer(uint8_t *buffer, int size)
{
    return write(fd, buffer, size);
}
uint8_t serialPort::getchar()
{
    uint8_t t;
    read(fd, &t, 1);
    return t;
}
void serialPort::ClosePort()
{
    close(fd);
}

bool serialPort::Decode(const unsigned char RawData[])
{
    bool are_you_ok = true;
    int state = 0;
    int I_get_cmd = 0;
    while (100 != state)
    {

        switch (state)
        {
        case 0:
            if (down_to_up_SOF1 == RawData[0])
            {
                state = 1;
            }
            else
            {
                are_you_ok = false;
                state = 100;
            }
            break;
        case 1:
            if (down_to_up_SOF2 == RawData[1])
            {
                state = 2;
            }
            else
            {
                are_you_ok = false;
                state = 100;
            }
            break;
        case 2:
            if (RawData[2] == 0x01 || RawData[2] == 0x02)
            {
                state += (int)RawData[2];
                I_get_cmd = RawData[2];
                break;
            }
            else
            {
                are_you_ok = false;
                state=100;
                break;
            }
            break;
        case 3:
            for (int i = 0; i < 8; i++)
            {
                RxMsg.yaw.bytes[i] = RawData[i + 3];
                RxMsg.pitch.bytes[i] = RawData[i + 11];
                RxMsg.roll.bytes[i] = RawData[i + 19]; // 串口还没写
            }
            state = 100;
            break;
        case 4:
            
            if(RawData[3]>0x01){
                are_you_ok = false;
                state=100;
                break;
            }
            if(RawData[4]==0x00||RawData[4]>0x08){
                are_you_ok = false;
                state=100;
                break;
            }
            if(RawData[5]>0x03){
                are_you_ok = false;
                state=100;
                break;
            }
            if(RawData[6]>0x01){
                are_you_ok = false;
                state=100;
                break;
            }
            RxMsg.self_color = RawData[3];      
            RxMsg.self_ID = RawData[4];
            RxMsg.DetectMode = RawData[5]; 
            RxMsg.AttackMode = RawData[6]; 

            state = 100;
            break;
        default:
            are_you_ok = false;
            state=100;
            break;
        }

        if (!are_you_ok)
        {
            break;
        }
    }
    return are_you_ok;
}

void serialPort::Encode(unsigned char RawData[], double YawData, double PitchData,unsigned char FireMode,float confidence)
{
    RxFP64Data yaw, pitch;
    yaw.data = YawData;
    pitch.data = PitchData;
    RawData[0] = up_to_down_SOF1;
    RawData[1] = up_to_down_SOF2;
    for (int i = 0; i < 8; i++)
    {
        RawData[i + 2] = yaw.bytes[i];
        RawData[i + 10] = pitch.bytes[i];
    }
    RawData[18]=FireMode;
  
    RxFP32Data conf;
    conf.data=confidence;
    for(int i=0;i<4;i++){
        RawData[i+19]=conf.bytes[i];
    }
}
bool serialPort::ok()
{
    return very_ok;
}