#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include "kaanh/kaanhconfig.h"
#include<atomic>
#include<string>
#include<filesystem>
#include "robot.h"

#include"plan.h"

using namespace aris::dynamic;
auto static xmlpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
auto static logpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
const std::string xmlfile = "kaanh.xml";		//控制配置文件名称
const std::string logfolder = "log";			//log文件夹名称


int main(int argc, char *argv[])
{

    xmlpath = xmlpath / xmlfile;				//拼接控制器配置文件路径
    logpath = logpath / logfolder;				//拼接log文件夹路径

    auto&cs = aris::server::ControlServer::instance();

    //cs.loadXmlFile(xmlpath.string().c_str());
    aris::core::fromXmlFile(cs, xmlpath);//加载kaanh.xml配置
    cs.resetPlanRoot(robot::createPlanRoot().release());//加载cmd配置
    //cs.saveXmlFile(xmlpath.string().c_str());	//save kaanh.xml配置
    cs.init();									//初始化
    aris::core::logDirectory(logpath);			//设置log路径
    std::cout << aris::core::toXmlString(cs) << std::endl; //print the control server state


    auto &cal = cs.model().calculator();		//UI变量求解器
    kaanhconfig::createUserDataType(cal, 6);		//预定义UI界面变量集
//    kaanhconfig::createPauseTimeSpeed();		//初始化UI停止暂停功能参数
    //cs.start();   // 不注释，则程序运行时开启控制器服务

    //实时回调函数，每个实时周期调用一次//
    cs.setRtPlanPostCallback(kaanh::update_state);
 //   g_model = cs.model();

    //开启windows下虚拟轴功能
#ifdef WIN32
    for (auto &m : cs.controller().slavePool())
	{
		dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);
	}
#endif


    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "1001", aris::core::Socket::WEB);
    //开启WebSocket/socket服务器//
    cs.open();

    //等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
    cs.runCmdLine();

    return 0;
}
