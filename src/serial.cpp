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
    //��һ���ִ���/
    //���ݾ�����豸�޸�
    const char default_path[] = "/dev/ttyACM2";
}

auto MoveTest::executeRT() -> int {
        int fd;
        char *path;
        uint8_t buf[100] = {0};

        //�ڶ����ִ���/

        //�������������ʹ��Ĭ���ն��豸
        if (argc > 1)
            path = argv[1];
        else
            path = (char *)default_path;

        //��ȡ�����豸������
        printf("This is tty/usart demo.\n");
        fd = open(path, O_RDWR | O_NOCTTY);
      //  fcntl(fd,F_SETFL,0);
        if (fd < 0)
        {
            printf("Fail to Open %s device\n", path);
            return 0;
        }

        //�������ִ���/
        struct termios opt;

        //��մ��ڽ��ջ�����
        tcflush(fd, TCIOFLUSH);
        // ��ȡ���ڲ���opt
        tcgetattr(fd, &opt);


        //���ô������������
        cfsetospeed(&opt, B9600);
        //���ô������벨����
        cfsetispeed(&opt, B9600);
        //��������λ��
        opt.c_cflag &= ~CSIZE;
        opt.c_cflag |= CS8;
        //У��λ
        opt.c_cflag &= ~PARENB;
        opt.c_iflag &= ~INPCK;
        //����ֹͣλ
        opt.c_cflag &= ~CSTOPB;

        //��������
        tcsetattr(fd, TCSANOW, &opt);

        std::cout << "Device ttyUSB0 is set to 9600bps,8N1\n" << std::endl;

        //���Ĳ��ִ���/
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


