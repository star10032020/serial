#include "serial/serialtools.hpp"
#include <stdexcept>
#include <string.h>
#include <sys/ioctl.h>
// using namespace std;
#include"serial/serial_node.hpp"
serialPort::serialPort(serial_node *father): roslogger(father->roslogger)
{
    this->fd = -1;
    this->very_ok = false;
    
}
serialPort::~serialPort(){
this->ClosePort();
}
serialPort::serialPort(serial_node *father,const char *dev): roslogger(father->roslogger)
{
    this->fd = -1;
    this->very_ok = false;
    RCLCPP_INFO(roslogger,"new dev:%s",dev);
    if (!OpenPort(dev))
    {

        //printf("open failed;\n serial BAD!\n");
        RCLCPP_INFO(roslogger,"open failed; \t serial BAD!");
    }
    else
    {
        if (!setup(115200, 0, 8, 1, 'N'))
        {
            //printf("setup failed;\n serial BAD!\n");
            RCLCPP_INFO(roslogger,"setup failed; \t serial BAD!");
        }
        else
        {
            //printf("setup is ok");
            //printf("open and setup ok.let us read.\n");
            RCLCPP_INFO(roslogger,"setup is ok...");
            RCLCPP_INFO(roslogger,"open and setup ok. \t let us read.");
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
        //printf("You have make it.Do not do it again!\n");
        RCLCPP_INFO(roslogger,"You have read it.Do not do it again!");
        return;
    }

    if (!OpenPort(dev))
    {

        //printf("open failed;\n serial BAD!\n");
        RCLCPP_INFO(roslogger,"open failed; \t serial BAD!");
    }
    else
    {
        //printf("Openport is ok.\n");
        RCLCPP_INFO(roslogger,"Openport is ok.");
        if (!setup(115200, 0, 8, 1, 'N'))
        {
            //printf("setup failed;\n serial BAD!\n");
        RCLCPP_INFO(roslogger,"setup failed \t serial BAD!");
        }
        else
        {
            //printf("setup is ok.\n");
            //printf("open and setup ok.let us read.\n");
            RCLCPP_INFO(roslogger,"setup is ok...");
            RCLCPP_INFO(roslogger,"open and setup ok. \t let us read.");
            very_ok = true;
        }
    }
}

char serialPort::getTargetColor(){
     if (!this->very_ok)
    {
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        return 'F';
    }
    getData(4);
    return this->serial_target_color;
}
unsigned char serialPort::getMode(){
     if (!this->very_ok)
    {
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        return 100;
    }
    getData(4);
    return this->serial_mode;
}
unsigned char serialPort::getTargetRobotType(){
     if (!this->very_ok)
    {
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        return 100;
    }
    getData(4);
    return this->serial_robot_type;
}
void serialPort::getGimbalPose(double &gimbal_roll,double &gimbal_pitch,double &gimbal_yaw)
{
    // tcflush(fd,TCIOFLUSH);
    if (!this->very_ok)
    {
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        return;
    }
    getData(1);
    gimbal_roll =serial_roll;
    gimbal_yaw = serial_yam, gimbal_pitch = serial_pitch;
    return;
}

void serialPort::setGimbalPose(float gimbal_yaw, float gimbal_pitch)
{
    unsigned char WriData[30];
    tcflush(fd, TCIOFLUSH);
    if (!this->very_ok)
    {
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        return;
    }
    Encode(WriData, gimbal_yaw, gimbal_pitch);
    writeBuffer(WriData, 10);
    // printf("we have write.\n");
}
float serialPort::getBalletSpeed()
{
    if (!this->very_ok)
    {
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        return -1;
    }
    getData(3);
    return RxMsg.BulletSpeed.data;
}

RobotID serialPort::getSelfId()
{

    if (!this->very_ok)
    {
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        return UNKNOWN_ID_ROBOT;
    }
    getData(3);
    return this->serial_ID;
}
Color serialPort::getSelfColor()
{
    if (!this->very_ok)
    {
         RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
        //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        return UNKNOWN_COLOR;
    }

    getData(3);
    return this->serial_clour;
}
std::map<CtrlKey, bool> serialPort::getCtrlKeys()
{
    if (!this->very_ok)
    {
        RCLCPP_INFO(roslogger,"Oh NO! \t serialPort may not been open or setup correctly!");
       //printf("Oh NO! serialPort may not been open or setup correctly!\n");
        return this->serial_map;
    }
    getData(2);
    return this->serial_map;
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

void serialPort::sbus_to_rc(const unsigned char *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

bool serialPort::Decode(const unsigned char RawData[], int You_need_cmd)
{
    bool are_you_ok = true;
    int state = 0;
    int I_get_cmd = 0;
    while (100 != state)
    {

        switch (state)
        {
        case 0:
            if (SOF1 == RawData[0])
            {
                state = 1;
            }
            else
            {
                are_you_ok = false;
                state = 0;
            }
            break;
        case 1:
            if (SOF2 == RawData[1])
            {
                state = 2;
            }
            else
            {
                are_you_ok = false;
                state = 0;
            }
            break;
        case 2:
            if (RawData[2] > 3 || RawData[2] < 1)
            {
                are_you_ok = false;
                break;
            }
            state += RawData[2];
            I_get_cmd = RawData[2];
            if (I_get_cmd != You_need_cmd)
                are_you_ok = false;
            break;
        case 3:
            for (int i = 0; i < 4; i++)
            {
                RxMsg.yaw.bytes[i] = RawData[i + 3];
                RxMsg.pitch.bytes[i] = RawData[i + 7];
                RxMsg.roll.bytes[i]=0;//串口还没写
            }
            state = 100;
            break;
        case 4:
            sbus_to_rc(RawData + 3, &RxMsg.RemoteControl);
            state = 100;
            break;
        case 5:
            RxMsg.ID = RawData[3];
            RxMsg.LightColor = RawData[4];
            for (int i = 0; i < 4; i++)
            {
                RxMsg.BulletSpeed.bytes[i] = RawData[i + 5];
            }
            state = 100;
            break;
        case 6:
            RxMsg.target_color=0;//串口还没写
            RxMsg.mode=0;//串口还没写
            RxMsg.target_robot_type=0;//串口还没写
            
            state=100;
            break;
        default:
            are_you_ok=false;
            break;
        }

        if (!are_you_ok)
        {
            break;
        }
    }
    return are_you_ok;
}

void serialPort::Encode(unsigned char RawData[], float YawData, float PitchData)
{
    RxFP32Data yaw, pitch;
    yaw.data = YawData;
    pitch.data = PitchData;
    RawData[0] = SOF1;
    RawData[1] = SOF2;
    for (int i = 0; i < 4; i++)
    {
        RawData[i + 2] = yaw.bytes[i];
        RawData[i + 6] = pitch.bytes[i];
    }
}
void serialPort::getData(int You_need_cmd)
{
    static unsigned char MyRowData[300] = {};
    bool I_ok = false;
    int this_have_run_time;
    for (this_have_run_time = 1; this_have_run_time <= 100; ++this_have_run_time)
    {
        tcflush(fd, TCIOFLUSH);

        // printf("we are deleting the huancun now .\n");
        readBuffer(MyRowData, 80);
        // printf("we are getting data now.\n");

        /* if(You_need_cmd==0){

                             for(int i=0;i<100;i++)
                             printf("%X ",MyRowData[i]);
                         printf("\n");
                         }*/

        for (int i = 0; i + 22 < 78; i++)
        {
            if (MyRowData[i] == SOF1 && MyRowData[i + 1] == SOF2)
                if (MyRowData[i + 2] >= 1 && MyRowData[i + 2] <= 3)
                {
                    if (You_need_cmd == 0)
                    {

                        for (int j = i; j < 22 + i && j < 80; j++)
                            printf("%X ", MyRowData[j]);
                        printf("\n");
                    }
                    I_ok = Decode(MyRowData + i, You_need_cmd);
                    if (I_ok)
                    {
                        break;
                    }
                }
        }
        if (I_ok)
        {
            break;
        }
        else
            RCLCPP_INFO(roslogger,"this time we dont get the frame_header!\n");
    }
    if (this_have_run_time > 100)
    {
       RCLCPP_INFO(roslogger,"100 times try to get the serial message,but it is not ok. we end it,and you should notice it.\n");
        //this->very_ok = false;
        return;
    }
    if (You_need_cmd == 0)
        return;

    if (You_need_cmd == 1)
    {
        this->serial_yam = RxMsg.yaw.data;
        this->serial_pitch = RxMsg.pitch.data;
        this->serial_roll = RxMsg.roll.data;
        if (abs(this->serial_yam) > 4 || abs(this->serial_pitch) > 4 / 2)
        {
            return getData(1);
        }
    }
    else if (You_need_cmd == 2)
    {
        unsigned short tool = RxMsg.RemoteControl.key.v;
        bool key_ctrl = tool & 1;
        tool >>= 1;
        bool key_shift = tool & 1;
        tool >>= 1;
        bool key_e = tool & 1;
        tool >>= 1;
        bool key_q = tool & 1;
        tool >>= 1;

        /*bool key_d = tool&1;
        tool>>=1;
        bool key_a = tool&1;
        tool>>=1;
        bool key_s = tool&1;
        tool>>=1;
        bool key_w = tool&1;
        tool>>=1;*/

        bool mouse_left = RxMsg.RemoteControl.mouse.press_l;
        bool mouse_right = RxMsg.RemoteControl.mouse.press_r;

        this->serial_map[KEY_Q] = key_q;
        this->serial_map[KEY_CTRL] = key_ctrl;
        this->serial_map[KEY_SHIFT] = key_shift;
        this->serial_map[KEY_E] = key_e;
        this->serial_map[MOUSE_LEFT] = mouse_left;
        this->serial_map[MOUSE_RIGHT] = mouse_right;
    }
    else if (You_need_cmd == 3)
    {

        int tem_ID = RxMsg.ID, tem_LightCOlor = RxMsg.LightColor;

        switch (tem_ID)
        {

        case 1:
            this->serial_ID = HERO_ID_1;
            break;
        case 2:
            this->serial_ID = ENGINEER_ID_2;
            break;
        case 3:
            this->serial_ID = STANDARD_ID_3;
            break;
        case 4:
            this->serial_ID = STANDARD_ID_4;
            break;
        case 5:
            this->serial_ID = STANDARD_ID_5;
            break;
        case 7:
            this->serial_ID = SENTRY_ID_7;
            break;
        default:
            this->serial_ID = UNKNOWN_ID_ROBOT;
            break;
        }

        switch (tem_LightCOlor)
        {
        case 1:
            this->serial_clour = RED;
            break;
        case 2:
            this->serial_clour = BLUE;
            break;
        default:
            this->serial_clour = UNKNOWN_COLOR;
            break;
        }

        this->serial_speed = RxMsg.BulletSpeed.data;
    }else if (You_need_cmd==4)
    {
        switch(RxMsg.target_color){
            case 0:
                this->serial_target_color='R';
                break;
            case 1:
                this->serial_target_color='B';
                break;
            case 2:
                this->serial_target_color='U';
                break;
            default:
                this->serial_target_color='N';
                break;
        }

        this->serial_mode=RxMsg.mode;
        this->serial_robot_type=RxMsg.target_robot_type;
    }
    

    return;
}

bool serialPort::ok() {
    return very_ok;
}