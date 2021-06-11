#include<iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>



#include"serial.h"

auto Serial::prepareNrt() -> void {
    //imu
    static const u_int8_t start[6] = {0xA5,0x5A,0x04,0x01,0x05,0xAA};
    static const char stop[6] = {'0','1','2','3','4','5'};
    static int data_length = 40;
    static uint8_t imu_data[40] = { 0 };
    m_and_s_ = doubleParam("motor_and_switch");
    //第一部分代码/
    //根据具体的设备修改
    const char default_path[] = "/dev/ttyACM2";
}

auto MoveTest::executeRT() -> int {
        int fd;
        char *path;
        uint8_t buf[100] = {0};

        //第二部分代码/

        //若无输入参数则使用默认终端设备
        if (argc > 1)
            path = argv[1];
        else
            path = (char *)default_path;

        //获取串口设备描述符
        printf("This is tty/usart demo.\n");
        fd = open(path, O_RDWR | O_NOCTTY);
      //  fcntl(fd,F_SETFL,0);
        if (fd < 0)
        {
            printf("Fail to Open %s device\n", path);
            return 0;
        }

        //第三部分代码/
        struct termios opt;

        //清空串口接收缓冲区
        tcflush(fd, TCIOFLUSH);
        // 获取串口参数opt
        tcgetattr(fd, &opt);


        //设置串口输出波特率
        cfsetospeed(&opt, B9600);
        //设置串口输入波特率
        cfsetispeed(&opt, B9600);
        //设置数据位数
        opt.c_cflag &= ~CSIZE;
        opt.c_cflag |= CS8;
        //校验位
        opt.c_cflag &= ~PARENB;
        opt.c_iflag &= ~INPCK;
        //设置停止位
        opt.c_cflag &= ~CSTOPB;

        //更新配置
        tcsetattr(fd, TCSANOW, &opt);

        std::cout << "Device ttyUSB0 is set to 9600bps,8N1\n" << std::endl;

        //第四部分代码/
     //   write(fd,stop,1);
        usleep(1000*1000);

        tcflush(fd,TCIFLUSH);
        tcflush(fd,TCOFLUSH);
        tcflush(fd, TCIOFLUSH);
        tcsetattr(fd, TCSANOW, &opt);
        usleep(100*100);
        //write(fd,start,6);
        usleep(100*100);

        if (m_and_s_ == 0){
            write(fd,stop,1);
        }
        else if(m_and_s_ == 1){
            write(fd,stop+1,1);
        }
        else if(m_and_s_ == 2){
            write(fd,stop+2,1);
        }
        else if(m_and_s_ == 3){
            write(fd,stop+3,1);
        }
//        while (1)
//        {
//            write(fd,stop+2,1);
//            usleep(1000*1000);
//            write(fd,stop,1);
//            usleep(1000*1000);
//            //---------------------------------------------
//           read(fd,buf,20*sizeof (uint8_t) * data_length);
//           std::copy(buf,buf+40,imu_data);
//           std::cout<<imu_data<<std::endl;
//        }

        return 0;
    }

Serial::Serial(const std::string &name) : Plan(name) {
    command().loadXmlStr(
            "<Command name=\"sucker_ctrl\">"
            "	<Param name=\"motor\" default=\"Off\" abbreviation=\"mands\"/>"
            "	<Param name=\"airSwitch\" default=\"Off\" abbreviation=\"mands\"/>"
            "</Command>");
}


