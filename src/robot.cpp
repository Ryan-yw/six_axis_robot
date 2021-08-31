#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include "kaanh.h"
#include "robot.h"
#include<aris.hpp>

#include"plan.h"
#include"planRT.h"

using namespace aris::dynamic;
using namespace aris::plan;
//------------------
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
//------------------
namespace robot {
//MoveJS的指令参数结构体，长度单位是m，角度单位是rad
//指令的执行顺序
//1、先执行prepareNrt，每条指令只执行一次
//2、然后执行executeRT,executeRT每一个循环周期(默认1ms)会被实时核调用一次，执行的总时间由用户给定
//3、执行结束后，本指令会被析构
//指令功能：某一电机或者所有电机1-cos(theta)轨迹运行，幅值为pos，周期为time，周期数为timenum

/*-----------MoveS------------*/
    struct MoveSParam {
        double pos;
        double time;
        uint32_t timenum;
        std::vector<bool> active_motor;            //目标电机
        std::vector<double> begin_pjs;            //起始位置
        std::vector<double> step_pjs;            //目标位置
    };
    auto MoveS::prepareNrt() -> void {
        //初始化这个结构体中的每个成员
        MoveSParam param;
        param.active_motor.clear();
        param.active_motor.resize(controller()->motionPool().size(), false);  //开辟一段空间，有motionPool().size()这么大，每个元素用false进行填充
        param.begin_pjs.resize(controller()->motionPool().size(), 0.0);
        param.step_pjs.resize(controller()->motionPool().size(), 0.0);
        param.pos = 0.0;
        param.time = 0.0;
        param.timenum = 0;

        // 根据发送的指令，填充结构体里面对应成员的值
        //解析指令参数
        for (auto &p : cmdParams()) {
            if (p.first == "all") {
                std::fill(param.active_motor.begin(), param.active_motor.end(), true);
            } else if (p.first == "motion_id") {
                param.active_motor.at(int32Param(p.first)) = true;
            } else if (p.first == "pos") {
                if (p.second == "current_pos") {
                    param.pos = 0;
                } else {
                    param.pos = doubleParam(p.first);
                }

            } else if (p.first == "time") {
                param.time = doubleParam(p.first);
            } else if (p.first == "timenum") {
                param.timenum = int32Param(p.first);
            }
        }

        this->param() = param;
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto MoveS::executeRT() -> int {
        auto &param = std::any_cast<MoveSParam &>(this->param());
        auto time = static_cast<int32_t>(param.time * 1000);        //运行周期
        auto totaltime = static_cast<int32_t>(param.timenum * time);//运行总时间
        double accelerationtime = 100;
        double k;

        //第一个周期设置log文件名称，获取当前电机所在位置
        if (count() == 1) {
            controller()->logFileRawName("motion_replay");//设置log文件名称

            for (Size i = 0; i < param.active_motor.size(); ++i) {
                if (param.active_motor[i]) {
                    param.begin_pjs[i] = controller()->motionPool()[i].targetPos();
                }
            }
        }

        k = param.pos / (2 * accelerationtime * (time - accelerationtime));

        //1-cos(theta)轨迹运行
        for (std::size_t i = 0; i < param.active_motor.size(); ++i) {
            if (param.active_motor[i]) {
                if (time / 2 < accelerationtime) {
                    param.step_pjs[i] =
                            param.begin_pjs[i] + param.pos * (std::cos(2.0 * PI * count() / time - PI) + 1) / 2;
                    controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
                } else if (time / 2 > accelerationtime) {
                    if (count() < accelerationtime) {
                        param.step_pjs[i] = param.begin_pjs[i] + k * count() * count() / 2;
//                        controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
                    } else if (accelerationtime < count() < (time - accelerationtime)) {
                        param.step_pjs[i] = param.begin_pjs[i] + k * accelerationtime * count() -
                                            k * accelerationtime * accelerationtime / 2;
//                        controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
                    } else if (count() > (time - accelerationtime)) {
                        param.step_pjs[i] = param.begin_pjs[i] + k * (count() * (count() / 2 + time) -
                                                                      accelerationtime * (accelerationtime - time) -
                                                                      time * time / 2);
//                        controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
                    }
                    controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);

                }

//                param.step_pjs[i] = param.begin_pjs[i] + param.pos*(1.0 - std::cos(2.0 * PI*count() / time));
//                controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
            }

        }

        //打印
        auto &cout = controller()->mout();
        if (count() % 100 == 0) {
            for (std::size_t i = 0; i < param.active_motor.size(); i++) {
                cout << std::setprecision(10) << "targetpos:" << param.step_pjs[i] << "  ";
                cout << std::setprecision(10) << "actualpos:" << controller()->motionPool()[i].actualPos() << "  ";
            }
            cout << std::endl;

        }

        //记录
        auto &lout = controller()->lout();
        for (std::size_t i = 0; i < param.active_motor.size(); i++) {
            lout << controller()->motionPool()[i].targetPos() << "  ";
            lout << controller()->motionPool()[i].actualPos() << "  ";
        }
        lout << std::endl;

        //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
        return totaltime - count();
    }
    auto MoveS::collectNrt() -> void {}
    MoveS::~MoveS() = default;
    MoveS::MoveS(const std::string &name) {
        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        //command().loadXmlStr(
        aris::core::fromXmlString(command(),
                "<Command name=\"mvs\">"
                "	<GroupParam>"
                "		<Param name=\"pos\" default=\"current_pos\"/>"
                "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
                "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
                "		<UniqueParam default=\"all\">"\
            "			<Param name=\"all\" abbreviation=\"a\"/>"\
            "			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
                "		</UniqueParam>"
                "	</GroupParam>"
                "</Command>");
    }

/*-----------test------------*/
//    auto MoveTest::prepareNrt() -> void {
//        dir_ = doubleParam("direction");
//    }
//    auto MoveTest::executeRT() -> int {
//        double s_tcp[6];
//        static double begin_pe[6];
//        char eu_type[4]{'1', '2', '3', '\0'};
//        std::size_t motionNum = controller()->motionPool().size();
//        // end-effector //
//        auto &ee = model()->generalMotionPool()[0];
//        auto forwardPos = [&](){
//            for(std::size_t i =0; i <motionNum; ++i){
//                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
//            }
//            if(model()->solverPool()[1].kinPos()){
//                std::cout<<"forward kinematic failed"<<std::endl;
//                return false;
//            }
//            ee.updMpm();
//            return true;
//        };

//        if(!forwardPos()){
//            return -1;
//        }
//        if(count()==1){
//            ee.getMpe(begin_pe, eu_type);
//        }
//        ee.getMpe(s_tcp, eu_type);
//        s_tcp[2] += 0.000005;
//        ee.setMpe(s_tcp, eu_type);
//        model()->solverPool()[0].kinPos();
//        double x_joint[6];
//        for(std::size_t i = 0; i<motionNum; ++i)
//        {
//            model()->motionPool()[i].updMp();
//            x_joint[i] = model()->motionPool()[i].mp();
//        }
//        for(std::size_t i = 0; i<motionNum; ++i)
//        {
//            controller()->motionPool()[i].setTargetPos(x_joint[i]);
//        }
//        std::cout<<controller()->motionPool()[2].targetPos()<<std::endl;
//        if(abs(s_tcp[2]-begin_pe[2])>0.03){
//            return 0;
//        }
//        return 1;
//    }
//    auto MoveTest::collectNrt() ->void{};
//    MoveTest::MoveTest(const std::string &name){
//        //command().loadXmlStr(
//        aris::core::fromXmlString(command(),
//                "<Command name=\"mv\">"
//                "	<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
//                "</Command>");
//    }
//    MoveTest &MoveTest::operator=(const MoveTest &other) = default;
//    MoveTest &MoveTest::operator=(MoveTest &&other) = default;
//    MoveTest::~MoveTest() = default;
//    MoveTest::MoveTest(const MoveTest &other) = default;
auto MoveTest::prepareNrt() -> void {
   dir_ = doubleParam("direction");
}
auto MoveTest::executeRT() -> int {
   static double begin_pe[6];

   // end-effector //
   auto &ee = model()->generalMotionPool()[0];
   double pe[6] = {0};
   double pre_step[] = {0};

   // get begin pe //
   if (count() == 1)
   {
       // get position from controller //
       for (std::size_t i = 0; i < model()->motionPool().size(); ++i)
       {
           model()->motionPool()[i].setMp(controller()->motionPool()[i].actualPos());
       }

       // forward kinematic //
       model()->solverPool()[1].kinPos();

       // update ee //
       ee.updMpm();

       // get ee pe //
       ee.getMpe(begin_pe);

   }
   std::copy(begin_pe, begin_pe + 6, pe);
   // trajectory generation //

   TCurve s1(5, 2);
   s1.getCurveParam();
   EllipseTrajectory e1(-1, 0.8, 0, s1);
   e1.getEllipseTrajectory(count());

   //   mout() << "step:" << dir_*0.1 * e1.z_ << " " << dir_*0.1 * e1.x_ << " " << 0.1 * e1.y_ << std::endl;
   //    mout() << "pe:" << pe[0] << " " << pe[1] << " " << pe[2] << std::endl;
//    pe[0] = begin_pe[0] + dir_*0.1 * e1.z_;
//    pe[1] = begin_pe[1] + dir_*0.1 * e1.x_;
//    pe[2] = begin_pe[2] + 0.1 * e1.y_;
    pe[0] = begin_pe[0] + dir_*0.1 * s1.getTCurve(count());
   // inverse kinematic //
   ee.setMpe(pe);
   model()->solverPool()[0].kinPos();

   // get input //
//    double input[6];
   for (std::size_t i = 0; i < model()->motionPool().size(); ++i)
   {
       model()->motionPool()[i].updMp();
       controller()->motionPool()[i].setTargetPos(model()->motionPool()[i].mp());
   }

   return s1.Tc_ * 1000 - count();
}

MoveTest::MoveTest(const std::string &name)
{
           //command().loadXmlStr(
           aris::core::fromXmlString(command(),
                   "<Command name=\"mv\">"
                   "	<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
                   "</Command>");
}
/*-----------Move Joint------------*/
    static std::atomic_bool enable_mvJoint = true;
    struct MoveJointParam {
        aris::dynamic::Marker *tool, *wobj;
        float realdata_init[6], realdata[6];//the initial force data, and the force data of eacch cycle
        double theta;
        double vel_limit[6], k[6], damping[6], K[6], force_delete[6];
        double fs2tpm[16], t2bpm[16];
        double force_offset[6], force_target[6];
        double force_damp[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double theta_setup = -90;//力传感器安装位置相对tcp偏移角
        double pos_setup = 0.061;//力传感器安装位置相对tcp偏移距离
        double threshold = 1e-6, a1, a0, KK = 10, force_aim = -1;
        double B = 1;// the damping constance of robot
        double M = 1;// the mass of robot
        double pm_init[16];//the position matrix of end effector when program starts
        double pm_real[16];
        double R01[16];
        double target_displacement = 0;
        double s_temp = 0, ss = 0;
        double ke = 220000;// the spring constance of the environment surface
        double x_r;// the reference position of trajactory, calculated
        double x_e;// the environment position, known
        double loop_period = 1e-3;// the period of every loop
        bool contacted = false;
//    const double M=1,B=1,KK=10,force_aim=-1;
//    double a1, a0;

    };
    struct MoveJoint::Imp : public MoveJointParam {
    };
    auto MoveJoint::prepareNrt() -> void {
        std::cout << "prepare begin" << std::endl;

        enable_mvJoint.store(true);
        imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
        imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));

        imp_->x_e = 0;

        for (auto cmd_param : cmdParams()) {
            if (cmd_param.first == "vellimit") {
                auto a = matrixParam(cmd_param.first);
                if (a.size() == 6) {
                    std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                } else {
                    THROW_FILE_LINE("");
                }
            } else if (cmd_param.first == "kd") {
                auto temp = matrixParam(cmd_param.first);
                if (temp.size() == 6) {
                    std::copy(temp.data(), temp.data() + 6, imp_->k);
                } else {
                    THROW_FILE_LINE("");
                }
            } else if (cmd_param.first == "damping") {
                auto temp = matrixParam(cmd_param.first);
                if (temp.size() == 1) {
                    std::fill(imp_->damping, imp_->damping + 6, temp.toDouble());
                } else if (temp.size() == 6) {
                    std::copy(temp.data(), temp.data() + 6, imp_->damping);
                } else {
                    THROW_FILE_LINE("");
                }
            }
        }

        for (auto &option : motorOptions()){
            option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
        }
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::cout << "prepare finished" << std::endl;
    }
    auto MoveJoint::executeRT() -> int {

        const int FS_NUM = 7;

        // end-effector //
        auto &ee = model()->generalMotionPool()[0];

        // Function Pointer
        auto &cout = controller()->mout();
        auto &lout = controller()->lout();
        char eu_type[4]{'1', '2', '3', '\0'};

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double v_tcp[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, v_joint[6];
        static double s_tcp[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        static float realdata[6];

        // use lambda function to realize sub-function //
        auto get_force_data = [&](float *data) {
            for (int i = 0; i < 6; i++) {
                 this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
            }
        };


        if (count() == 1) {
            //get the current end effector position
            for(std::size_t i = 0; i < 6; ++i){
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
            }
            model()->solverPool()[1].kinPos();
            ee.updMpm();
            ee.getMpm(imp_->pm_init);
            //设置log文件名称//
            controller()->logFileRawName("motion_replay");
            //获取力传感器相对tcp的旋转矩阵//
            imp_->theta = (-imp_->theta_setup) * PI / 180;
            double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(imp_->theta / 2.0), cos(imp_->theta / 2.0)};
            s_pq2pm(pq_setup, imp_->fs2tpm);
            get_force_data(imp_->realdata_init);
        }

        //减去力传感器初始偏置
        get_force_data(realdata);
        for (int i = 0; i < 6; i++)
            realdata[i] -= imp_->realdata_init[i];
        double real_data_double[6];
        for (int i = 0; i < 6; ++i)real_data_double[i] = static_cast<double>(realdata[i]);

        //获取每个周期末端所受的力
        double xyz_temp[3]{real_data_double[0], real_data_double[1], real_data_double[2]}, abc_temp[3]{
                real_data_double[3], real_data_double[4], real_data_double[5]};
        double pm_begin[16];
        //transform the force to world frame, store in imp_->force_target
        model()->generalMotionPool().at(0).updMpm();
        imp_->tool->getPm(*imp_->wobj, pm_begin);
        s_pm_dot_pm(pm_begin, imp_->fs2tpm, imp_->t2bpm);
        s_mm(3, 1, 3, imp_->t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, imp_->force_target, 1);
        s_mm(3, 1, 3, imp_->t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, imp_->force_target + 3, 1);
        //print the force of z
        if (count() % 500 == 0) {
            std::cout << "fz:" << imp_->force_target[2] << " ";
            std::cout << std::endl;
        }
        if (abs(imp_->force_target[2]) > 0.1) {
            /*
            * if判断语句的作用：
            * abs(imp_->force_target[2])>0.1 表示已经与目标物体接触；
            * else表示还没接触的时候；
            * 如果没有接触就以指定速度下降，如果产生接触则执行阻抗控制命令；
            */
            //get the velocity of tool center
            model()->generalMotionPool().at(0).getMve(v_tcp,eu_type);
            //get the position of tool center
            model()->generalMotionPool().at(0).getMpe(s_tcp,eu_type);
            //get the x_environment as soon as contact happens
            if(!imp_->contacted){
                imp_->contacted = true;
                imp_->x_e = s_tcp[2];
            }
            //get the x_reference of
            imp_->x_r = imp_->x_e - (imp_->force_aim / imp_->ke);
            //calculate the desired acceleration and velocity
            double a = (imp_->force_target[2] - imp_->force_aim - imp_->damping[2]* v_tcp[2] - imp_->K[2] * (s_tcp[2] - imp_->x_r))/imp_->M;
            double v = v_tcp[2] + a * imp_-> loop_period;
            double next_tcp_v[6] = {0,0,v,0,0,0};
            //inverse kinematic to calculate the acceleration of every motor
                //get the jacobian first
                // 获取雅可比矩阵，并对过奇异点的雅可比矩阵进行特殊处理
            double pinv[36];
            {
                auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
                fwd.cptJacobiWrtEE();
                    //QR分解求方程的解
                double U[36], tau[6], tau2[6];
                aris::Size p[6];
                Size rank;
                    //auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
                    //A为输入,根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
                s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-3);
                    //对奇异点进行特殊处理,对U进行处理
                if (rank < 6)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        if (U[7 * i] >= 0)
                        {
                            U[7 * i] = U[7 * i] + 0.1;
                        }
                        else
                        {
                            U[7 * i] = U[7 * i] - 0.1;
                        }
                    }
                }
                    // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
                s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);
            }
                //calculate the desired accelaration of every motor
            s_mm(6,1,6,pinv,next_tcp_v,v_joint);
            //move the joint
            for(std::size_t i = 0; i<controller()->motionPool().size(); ++i){
                controller()->motionPool()[i].setTargetVel(v_joint[i]);
            }
//            if (count()== 1 | flag == 0) {
//                flag=0;
//                //求阻尼力
//                model()->generalMotionPool()[0].getMve(v_now, eu_type);
//                model()->generalMotionPool()[0].getMve(v_tcp, eu_type);
                
//                // 读力
//                imp_->a1 = (imp_->force_target[2] - imp_->force_aim - imp_->damping[2] * v_tcp[2] -
//                            imp_->K[2] * s_tcp[2]) / 0.4;
//                v_tmp[2] = v_tcp[2];
//                v_tcp[2] += ((imp_->a1 + imp_->a0) * 1e-3) / 2;
//                imp_->a1 = imp_->a0;
//                // 计算阻抗控制笛卡尔空间下的目标相对位移s
//                // s_tcp[2]指末端z轴方向上的总相对位移
//                s_tcp[2] += ((v_tmp[2] + v_tcp[2]) * 1e-3) / 2;

//                // 打印并查验计算得出的目标速度与目标位移
//                if (count() % 50 == 0) {
//                    std::cout << "V:" << v_tcp[2] << std::endl;
//                    std::cout << "S:" << s_tcp[2] << std::endl;
//                    std::cout << "fce tar:" << imp_->force_target[2] << std::endl;
//                }
//            }

//            // 获取并设置当前关节空间
//            for (std::size_t i = 0; i < model()->motionPool().size(); ++i) {
//                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
//            }

//            // 根据模型与当前关节空间计算笛卡尔坐标系下的坐标
//            // forward kinematic //
//            model()->solverPool()[1].kinPos();

//            // 更新末端位置的位置坐标
//            // update ee //
//            ee.updMpm();
//            // 获取当前欧拉角位姿并赋值到begin_pe，方便操作
//            ee.getMpe(begin_pe);


//            // 初始化新的笛卡尔坐标pe（初始赋值为begin_pe）
//            std::copy(begin_pe, begin_pe + 6, pe);
//            double step = s_tcp[2];
//            mout() << "s_tcp[2]:" << s_tcp[2] << std::endl;
//--------------------------------------------------

//            // get position from controller //
//            for(int i=0;i<model()->motionPool().size();++i)
//            {
//                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
//            }


//            for (int i = 0; i < 6; i++)
//            {
//                p_next[i] = controller()->motionPool().at(i).actualPos();
//                p_next[i] += v_joint[i] * 1e-3;
//                p_next[i] = std::min(std::max(p_next[i], controller()->motionPool().at(i).minPos() + 0.1), controller()->motionPool().at(i).maxPos() - 0.1);

//                controller()->motionPool().at(i).setTargetPos(p_next[i]);
//                model()->motionPool().at(i).setMp(p_next[i]);
//            }


//            {
//            imp_->a1 = (imp_->force_target[2]-imp_->force_aim-imp_->damping[2] * v_tcp[2])/0.4;
//            v_tcp[2] += ((imp_->a1+imp_->a0)*1e-3)/2;
//            imp_->a1 = imp_->a0;
//            }
        } else {
            for (std::size_t i = 0; i < model()->motionPool().size(); ++i) {
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
            }
            // forward kinematic //
            model()->solverPool()[1].kinPos();
            ee.updMpm();
            // get position and eular of end effector
            double pe_now[6];
            ee.getMpe(pe_now);
            //give a velocity of the end effector
            double v_now[6]{0,0,-0.001,0,0,0};
            ee.setMva(v_now);
            //inverse kinematics
            model()->solverPool()[0].kinVel();
            //excute
            for(std::size_t i = 0; i< 6; ++i){
                controller()->motionPool()[i].setTargetVel(model()->motionPool()[i].mv());
                if (count() % 500 == 0) {
                    std::cout << "vi:" << model()->motionPool()[i].mv() <<std::endl;

                }
            }



//            std::copy(begin_pe, begin_pe + 6, pe);
//            Curve curve;
// //           cout << "T" << curve.T_ << std::endl;
//            double step = 0.1;
//            imp_->ss = curve.getCurve(count()) - imp_->s_temp;

////            if (count()%1000 == 0){
////               mout() << "getcurve:" << curve.getCurve(count()) << std::endl;
////               mout() << " imp_->s_temp:" <<  imp_->s_temp << std::endl;
////            }
//            pe[2] -= step * imp_->ss;
//            imp_->s_temp = curve.getCurve(count());
////            mout() << "3" << pe[2] << std::endl;

//            // 给ee设置新的笛卡尔坐标pe
//            // inverse kinematic //
//            ee.setMpe(pe);

//            // 进行反解，计算关节空间
//            model()->solverPool()[0].kinPos();
////            mout() << "4" << pe[2] << std::endl;
//            // 执行
//            for(std::size_t i = 0; i <model()->motionPool().size(); ++i){
//                model()->motionPool()[i].updMp();
//                controller()->motionPool()[i].setTargetPos(model()->motionPool()[i].mp());
//            }
        }
//            // 根据力传感器受力和阻尼力计算v_tcp //
//            for (int i = 0; i < 6; i++)
//            {
//                if (std::abs(imp_->force_target[i]) > imp_->threshold)  //ending actual force
//                {
//                    v_tcp[i] += imp_->k[i] * imp_->force_target[i] * 1e-3;
//                }
//                v_tcp[i] = std::min(std::max(v_tcp[i], -imp_->vel_limit[i]), imp_->vel_limit[i]);
//            }

        // 获取雅可比矩阵，并对过奇异点的雅可比矩阵进行特殊处理
//    double pinv[36];
//    {
//        auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
//        fwd.cptJacobiWrtEE();
//        //QR分解求方程的解
//        double U[36], tau[6], tau2[6];
//        aris::Size p[6];
//        Size rank;
//        //auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
//        //A为输入,根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
//        s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-3);
//        //对奇异点进行特殊处理,对U进行处理
//        if (rank < 6)
//        {
//            for (int i = 0; i < 6; i++)
//            {
//                if (U[7 * i] >= 0)
//                {
//                    U[7 * i] = U[7 * i] + 0.1;
//                }
//                else
//                {
//                    U[7 * i] = U[7 * i] - 0.1;
//                }
//            }
//        }
//        // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
//        s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);
//    }

//    // 根据v_tcp以及雅可比矩阵反算到关节v_joint
//    s_mm(6, 1, 6, pinv, v_tcp, v_joint);

        // 根据关节速度v_joint规划每个关节的运动角度 //
//    for (int i = 0; i < 6; i++)
//    {
//#ifdef UNIX
//        p_next[i] = controller()->motionPool().at(i).actualPos();
//#endif
//#ifdef WIN32
//        p_next[i] = controller()->motionPool().at(i).targetPos();
//#endif
//        p_next[i] += v_joint[i] * 1e-3;
//        p_next[i] = std::min(std::max(p_next[i], controller()->motionPool().at(i).minPos() + 0.1), controller()->motionPool().at(i).maxPos() - 0.1);

//        controller()->motionPool().at(i).setTargetPos(p_next[i]);
//        model()->motionPool().at(i).setMp(p_next[i]);
//    }

        // 运动学正解 //
        if (model()->solverPool().at(1).kinPos()){
            std::cout<<"forward kinematic failed";
            return -1;
        }

        return 1;
    }
    auto MoveJoint::collectNrt() -> void {}
    MoveJoint::~MoveJoint() = default;
    MoveJoint::MoveJoint(const MoveJoint &other) = default;
    MoveJoint::MoveJoint(MoveJoint &other) = default;
    MoveJoint &MoveJoint::operator=(const MoveJoint &other) = default;
    MoveJoint &MoveJoint::operator=(MoveJoint &&other) = default;
    MoveJoint::MoveJoint(const std::string &name){
        aris::core::fromXmlString(command(),
                "<Command name=\"movejoint\">"
                "	<GroupParam>"
                "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                "		<Param name=\"damping\" default=\"{0.02,0.02,0.02,0.01,0.01,0.01}\"/>"
                "		<Param name=\"kd\" default=\"{120,120,120,3,3,3}\"/>"
                "       <Param name=\"K\" default=\"{1,1,1,3,3,3}\"/>"
                "		<Param name=\"tool\" default=\"tool0\"/>"
                "		<Param name=\"wobj\" default=\"wobj0\"/>"
                "	</GroupParam>"
                "</Command>");
    }

/*-----------ImpedPos------------*/
    ImpedPos::ImpedPos(const std::string &name){
        aris::core::fromXmlString(command(),
                                  "<Command name=\"impedpos\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                  "		<Param name=\"tool\" default=\"tool0\"/>"
                                  "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                  "     <Param name=\"Mass\" default=\"0.05\" abbreviation=\"m\"/>"
                                  "     <Param name=\"Damp\" default=\"0.025\" abbreviation=\"b\"/>"
                                  "	</GroupParam>"
                                  "</Command>"
                                  );
    }
    struct ImpedPosParam{
        aris::dynamic::Marker * tool, * wobj;
        //parameters in prepareNT
        double x_e; //environment position
        double vel_limit[6];//the velocity limitation of motors
        //parameters in excuteRT
        double pm_init[16];//the position matrix of the tool center in world frame
        double theta_setup = -90;// the install angle of force sensor
        double pos_setup = 0.061;// the install position of force sensor
        double fs2tpm[16]; //
        float init_force[6]; // compensate the gravity of tool
        bool contacted = false;// flag to show if the end effector contact with the surface
        double desired_force = 1;
        double ke = 220000;
        double B = 0.7;//the damping factor of end effector0.7 has a better performance
        double M = 0.1;//0.1 has a better performance
        double K = 1;//
        double loop_period = 1e-3;
    };
    struct ImpedPos::Imp : public ImpedPosParam{};
    auto ImpedPos::prepareNrt() -> void
    {
        std::cout<<"prepare begin"<<std::endl;
        imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
        imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
        imp_-> x_e = 0;
        for (auto cmd_param : cmdParams()) {
            if (cmd_param.first == "vellimit") {
                auto a = matrixParam(cmd_param.first);
                if (a.size() == 6) {
                    std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                } else {
                    THROW_FILE_LINE("");
                }
            }else if (cmd_param.first == "Mass"){
                auto m = doubleParam(cmd_param.first);
                imp_->M = m;
            }else if(cmd_param.first == "Damp"){
                auto d = doubleParam(cmd_param.first);
                imp_->B = d;
            }
        }
        for (auto &option : motorOptions())//???
        {
            option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
        }
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::cout<<"prepare finished"<<std::endl;
    }
    auto ImpedPos::executeRT() ->int
    {
        const int FS_NUM = 7;
        static std::size_t motionNum = controller()->motionPool().size();
        // end-effector //
        auto &ee = model()->generalMotionPool()[0];
        // Function Pointer
//        auto &cout = controller()->mout();
//        auto &lout = controller()->lout();
        char eu_type[4]{'1', '2', '3', '\0'};
        // the lambda function to get the force
        auto get_force_data = [&](float *data){
            for (std::size_t i =0; i< motionNum;++i)
            {
                this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
            }
        };
        //the function to update model according to real motor and excecute one forward kinematic
        auto forwardPos = [&](){
            for(std::size_t i =0; i<motionNum; ++i)
            {
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
            }
            if(model()->solverPool()[1].kinPos())
            {
                std::cout<<"forward kinematic failed, exit"<<std::endl;
            }
            ee.updMpm();
        };
        //safty check the position change
        auto checkPos = [&](double * data)
        {
            for(std::size_t i = 0; i< motionNum; i++)
            {
                if(abs(data[i] - controller()->motionPool()[i].targetPos()) > imp_->vel_limit[i]){
                    std::cout<<"joint "<<i<<" move too fast"<<std::endl;
                    std::cout<<"pi-1 = "<<controller()->motionPool()[i].targetPos()<<std::endl;
                    std::cout<<"pi = "<<data[i]<<std::endl;
                    std::cout<<"vellimit = "<<imp_->vel_limit[i]<<std::endl;
                    return false;
                }
            }
            return true;
        };
        //first loop record
        if(count() ==1)
        {
            //get the current end effector position
            forwardPos();
            ee.getMpm(imp_->pm_init);
            //set the log file
            controller()->logFileRawName("motion_replay");
            //get the force transformation matrix in tool frame
            double theta = (-imp_->theta_setup) * PI / 180;
            double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
            s_pq2pm(pq_setup, imp_->fs2tpm);
            get_force_data(imp_->init_force);
        }
        //减去力传感器初始偏置
        float force_data[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double force_data_double[6];
        get_force_data(force_data);
        for (int i = 0; i < 6; i++)
            force_data[i] -= imp_->init_force[i];
        for (int i = 0; i < 6; ++i)force_data_double[i] = static_cast<double>(force_data[i]);
        //获取每个周期末端所受的力
        double xyz_temp[3]{force_data_double[0], force_data_double[1], force_data_double[2]}, abc_temp[3]{
                force_data_double[3], force_data_double[4], force_data_double[5]};
        //get the position of tcp
        double pm_begin[16];
        //transform the force to world frame, store in imp_->force_target
        model()->generalMotionPool().at(0).updMpm();
        imp_->tool->getPm(*imp_->wobj, pm_begin);
        double t2bpm[16];
        s_pm_dot_pm(pm_begin, imp_->fs2tpm, t2bpm);
        double net_force[6];
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, net_force, 1);
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, net_force + 3, 1);
        //print the force of z
        double v_tcp[6];
        if (count() % 500 == 0) {
            model()->generalMotionPool().at(0).getMve(v_tcp, eu_type);
            std::cout << "fz:" << net_force[2] << std::endl;//<< " v_tcp[2] : "<<v_tcp[2];

        }
        if(abs(net_force[2]) > 0.1 || imp_->contacted)
        {
            /*
            * if判断语句的作用：
            * abs(imp_->force_target[2])>0.1 表示已经与目标物体接触；
            * else表示还没接触的时候；
            * 如果没有接触就以指定速度下降，如果产生接触则执行阻抗控制命令；
            */

            double s_tcp[6];
            //get the velocity of tool center
            model()->generalMotionPool().at(0).getMve(v_tcp,eu_type);
            //get the position of tool center
            model()->generalMotionPool().at(0).getMpe(s_tcp,eu_type);
            //get the x_environment as soon as contact happens
            if(!imp_->contacted){
                imp_->contacted = true;
                imp_->x_e = s_tcp[2];
                std::cout << "x_e: " << imp_->x_e << std::endl;
            }
            //get the x_reference of
            double x_r = imp_->x_e - (imp_->desired_force / imp_->ke );
            //calculate the desired acceleration and velocity
            double a = (net_force[2] - imp_->desired_force- imp_->B* v_tcp[2] )/imp_->M;//- imp_->K * (s_tcp[2] - x_r))/imp_->M;
            double x = s_tcp[2] + 0.5 * a * imp_->loop_period* imp_->loop_period + v_tcp[2] * imp_->loop_period;
            double v = (x - s_tcp[2]);
            if (count() % 100 == 0) {
                model()->generalMotionPool().at(0).getMve(v_tcp, eu_type);
                std::cout << " a: " <<a << " v : "<<v<< " x : "<<x;
                std::cout << std::endl;
            }
            double ee_v [6]{0.0,0.0,v,0.0,0.0,0.0};
            s_tcp[2] = x;
            ee.setMpe(s_tcp, eu_type);
            model()->solverPool()[0].kinPos();
            double x_joint[6];
            for(std::size_t i = 0; i<motionNum; ++i)
            {
                model()->motionPool()[i].updMp();
                x_joint[i] = model()->motionPool()[i].mp();
            }
            if(checkPos(x_joint)){
                for(std::size_t i = 0; i<motionNum; ++i)
                {
                    controller()->motionPool()[i].setTargetPos(x_joint[i]);
                }
                ee.setMve(ee_v,eu_type);
            }
            if(count() > 15000){
                std::cout<<"finished"<<std::endl;
                return 0;
            }
        }else{
            forwardPos();
            double s_tcp[6];
            ee.getMpe(s_tcp, eu_type);
            s_tcp[2] -= 0.00001;
            ee.setMpe(s_tcp, eu_type);
            double ee_v[6]{0.0, 0.0, -0.00001, 0.0, 0.0, 0.0};
            model()->solverPool()[0].kinPos();
            double x_joint[6];
            for(std::size_t i = 0; i<motionNum; ++i)
            {
                model()->motionPool()[i].updMp();
                x_joint[i] = model()->motionPool()[i].mp();
            }
            if(checkPos(x_joint)){
                for(std::size_t i = 0; i<motionNum; ++i)
                {
                    controller()->motionPool()[i].setTargetPos(x_joint[i]);
                }
                model()->generalMotionPool()[0].setMve(ee_v, eu_type);
            }
        }
        // 运动学正解 //
        if (model()->solverPool().at(1).kinPos()){
            std::cout<<"forward kinematic failed";
            return -1;
        }

        return 1;
    }
    auto ImpedPos::collectNrt() -> void{}
    ImpedPos::~ImpedPos() = default;
    ImpedPos::ImpedPos(const ImpedPos &other) = default;

    /*-----------Drag------------*/
    Drag::Drag(const std::string &name){
        aris::core::fromXmlString(command(),
                                  "<Command name=\"drag\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                  "		<Param name=\"tool\" default=\"tool0\"/>"
                                  "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                  "     <Param name=\"Mass\" default=\"{0.3,0.3,0.3,0.0004,0.0004,0.0004}\" abbreviation=\"m\"/>"
                                  "     <Param name=\"Damp\" default=\"{0.9,0.9,0.9,0.0,0.0,0.0}\" abbreviation=\"b\"/>"
                                  "	</GroupParam>"
                                  "</Command>"
                                  );
    }
    struct DragParam{
        aris::dynamic::Marker * tool, * wobj;
        //parameters in prepareNT
        double x_e; //environment position
        double vel_limit[6];//the velocity limitation of motors
        //parameters in excuteRT
        double pm_init[16];//the position matrix of the tool center in world frame
        double theta_setup = -90;// the install angle of force sensor
        double pos_setup = 0.061;// the install position of force sensor
        double fs2tpm[16]; //
        float init_force[6]; // compensate the gravity of tool
        double ke = 220000;
        double B[6]{0.25,0.25,0.25,0.0,0.0,0.0};//the damping factor of end effector0.7 has a better performance
        double M[6]{0.25,0.25,0.25,1,1,1};//0.1 has a better performance
        double K = 1;//
        double loop_period = 1e-3;
        double last_v[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    };
    struct Drag::Imp : public DragParam{};
    auto Drag::prepareNrt() -> void
    {
        std::cout<<"prepare begin"<<std::endl;
        imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
        imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
        imp_-> x_e = 0;
        for (auto cmd_param : cmdParams()) {
            if (cmd_param.first == "vellimit") {
                auto a = matrixParam(cmd_param.first);
                if (a.size() == 6) {
                    std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                } else {
                    THROW_FILE_LINE("");
                }
            }else if (cmd_param.first == "Mass"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 6) {
                    std::copy(m.data(), m.data() + 6, imp_->M);
                }else if (m.size() == 1) {
                    std::copy(m.data(), m.data()+1, imp_->M+3);
                    std::copy(m.data(), m.data() + 1, imp_->M+4);
                    std::copy(m.data(), m.data() + 1, imp_->M+5);
                }else{
                    THROW_FILE_LINE("");
                }
            }else if(cmd_param.first == "Damp"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 6) {
                    std::copy(m.data(), m.data() + 6, imp_->B);
                }else if (m.size() == 1) {
                    std::copy(m.data(), m.data()+1, imp_->B+3);
                    std::copy(m.data(), m.data() + 1, imp_->B+4);
                    std::copy(m.data(), m.data() + 1, imp_->B+5);
                }else{
                    THROW_FILE_LINE("");
                }
            }
        }
        for (auto &option : motorOptions())//???
        {
            option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
        }
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::cout<<"prepare finished"<<std::endl;
    }
    auto Drag::executeRT() ->int
    {
        const int FS_NUM = 7;
        static std::size_t motionNum = controller()->motionPool().size();
        // end-effector //
        auto &ee = model()->generalMotionPool()[0];
        // Function Pointer
//        auto &cout = controller()->mout();
//        auto &lout = controller()->lout();
        char eu_type[4]{'1', '2', '3', '\0'};
        // the lambda function to get the force
        auto get_force_data = [&](float *data){
            for (std::size_t i =0; i< motionNum;++i)
            {
                this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
            }
        };


        for(std::size_t i =0; i<motionNum; ++i)
        {
            model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
        }
        if (model()->solverPool().at(1).kinPos()){
            std::cout<<"forward kinematic failed";
            return -1;
        }else{
            ee.updMpm();
        }
        //first loop record
        if(count() ==1)
        {
            //get the current end effector position
            ee.getMpm(imp_->pm_init);
            //set the log file
            controller()->logFileRawName("motion_replay");
            //get the force transformation matrix in tool frame
            double theta = (-imp_->theta_setup) * PI / 180;
            double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
            s_pq2pm(pq_setup, imp_->fs2tpm);
            get_force_data(imp_->init_force);
        }
        //减去力传感器初始偏置
        float force_data[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double force_data_double[6];
        get_force_data(force_data);
        for (int i = 0; i < 6; i++)
            force_data[i] -= imp_->init_force[i];
        for (int i = 0; i < 6; ++i)force_data_double[i] = static_cast<double>(force_data[i]);
        //获取每个周期末端所受的力
        double xyz_temp[3]{force_data_double[0], force_data_double[1], force_data_double[2]}, abc_temp[3]{
                force_data_double[3], force_data_double[4], force_data_double[5]};
        //get the position of tcp
        double pm_begin[16];
        //transform the force to world frame, store in imp_->force_target
        model()->generalMotionPool().at(0).updMpm();
        imp_->tool->getPm(*imp_->wobj, pm_begin);
        double t2bpm[16];
        s_pm_dot_pm(pm_begin, imp_->fs2tpm, t2bpm);
        double net_force[6];
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, net_force, 1);
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, net_force + 3, 1);
        //print the force of z
        if (count() % 100 == 0) {
            std::cout << "fz:" << net_force[0] ;//<< " v_tcp[2] : "<<v_tcp[2];
            std::cout<<std::endl;
        }
        double s_tcp[6];
        //get the position of tool center
        ee.getMpe(s_tcp,eu_type);
        double a[6]{0,0,0,0,0,0};
        double v[6]{0,0,0,0,0,0};
        double x[6]{0,0,0,0,0,0};
        for(std::size_t i =0 ;i < 3 ; i++){
            a[i] = (net_force[i])/imp_->M[i];//- imp_->K * (s_tcp[2] - x_r))/imp_->M;
            v[i] = a[i] * imp_->loop_period * imp_->loop_period + imp_->B[i]*imp_->last_v[i];
            imp_->last_v[i] = v[i];
            x[i] = s_tcp[i] + v[i];
            double max =0.00010;
            //prevent the abrupt change
            if(abs(v[i]) > max){
                x[i]=v[i]>0?(s_tcp[i]+max):(s_tcp[i]-max);
                v[i]=v[i]>0?max:-max;
                imp_->last_v[i] = v[i];
                {std::cout<<"speed reaches the maximum value"<<std::endl;}
            }
            s_tcp[i] = x[i];
        }
        //rotation part
        for(std::size_t i=3 ; i < 6; i++){
            double lowerbound = 0.003;
            if(abs(net_force[i])>lowerbound){
                net_force[i] -= lowerbound;
            }else{
                net_force[i] = 0;
            }
            //get the desired angular acceleration, get the desired angular velocity
            a[i] = (net_force[i])/imp_->M[i];//- imp_->K * (s_tcp[2] - x_r))/imp_->M;
            v[i] = a[i] * imp_->loop_period * imp_->loop_period + imp_->B[i]*imp_->last_v[i];
            imp_->last_v[i] = v[i];
        }
        double re[3]{0,0,0};
        std::copy(s_tcp+3,s_tcp+6,re);
        double rm_init[9];
        s_re2rm(re,rm_init,eu_type);
        double rm[9]{0,0,0,0,0,0,0,0,0};
        double omega[3]{v[3],v[4],v[5]};

        //get the scale of angular velocity
        double scale = s_norm(3,omega);
        //scale upper limit
        double rot_max = 0.0005;
        if(scale >= rot_max){
            scale = rot_max;
            std::cout<<"rot reach upper bound"<<std::endl;
        }
        if(scale != 0.0){
            //get the unit vector of the angular velocity
            s_nv(3,1/scale,omega);
            double skew[9]{0,0,0,0,0,0,0,0,0};
            s_cm3(omega,skew);
            s_eye(3,rm);
            s_mma(3,3,3,(1.0-std::cos(scale)),skew,skew,rm);
            s_ma(3,3,std::sin(scale),skew,rm);
            double result[9]{0,0,0,0,0,0,0,0,0};
            s_mma(3,3,3,rm,rm_init,result);
            //avoid zero torque
            if(!std::isnan(result[0])){
                s_rm2re(result,re,eu_type);
                std::copy(re,re+3,s_tcp+3);
               // dsp(3,3,result);
            }
        }
        if(count()%100 == 0){
            std::cout<<net_force[3]<<"   "<<net_force[4]<<"   "<<net_force[5]<<"   "<<std::endl;
        }

        //inverse kinematic calculation
        ee.setMpe(s_tcp, eu_type);
        model()->solverPool()[0].kinPos();
        double x_joint[6];
        for(std::size_t i = 0; i<motionNum; ++i)
        {
            model()->motionPool()[i].updMp();
            x_joint[i] = model()->motionPool()[i].mp();
            controller()->motionPool()[i].setTargetPos(x_joint[i]);
        }

        if(count() > 20000){
            std::cout<<"finished"<<std::endl;
            return 0;
        }

        return 1;
    }
    auto Drag::collectNrt() -> void{}
    Drag::~Drag() = default;
    Drag::Drag(const Drag &other) = default;


    /*-----------DragTrans------------*/
    DragTrans::DragTrans(const std::string &name){
        aris::core::fromXmlString(command(),
                                  "<Command name=\"dragtrans\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                  "		<Param name=\"tool\" default=\"tool0\"/>"
                                  "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                  "     <Param name=\"Mass\" default=\"{0.3,0.3,0.3,1.0,1.0,1.0}\" abbreviation=\"m\"/>"
                                  "     <Param name=\"Damp\" default=\"{0.9,0.9,0.9,0.0,0.0,0.0}\" abbreviation=\"b\"/>"
                                  "	</GroupParam>"
                                  "</Command>"
                                  );
    }
    struct DragTransParam{
        aris::dynamic::Marker * tool, * wobj;
        //parameters in prepareNT
        double x_e; //environment position
        double vel_limit[6];//the velocity limitation of motors
        //parameters in excuteRT
        double pm_init[16];//the position matrix of the tool center in world frame
        double theta_setup = -90;// the install angle of force sensor
        double pos_setup = 0.061;// the install position of force sensor
        double fs2tpm[16]; //
        float init_force[6]; // compensate the gravity of tool
        double ke = 220000;
        double B[6]{0.25,0.25,0.25,0.0,0.0,0.0};//the damping factor of end effector0.7 has a better performance
        double M[6]{0.25,0.25,0.25,1,1,1};//0.1 has a better performance
        double K = 1;//
        double loop_period = 1e-3;
        double last_v[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double maxTrans =0.00010;
    };
    struct DragTrans::Imp : public DragTransParam{};
    auto DragTrans::prepareNrt() -> void
    {
        std::cout<<"prepare begin"<<std::endl;
        imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
        imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
        imp_-> x_e = 0;
        for (auto cmd_param : cmdParams()) {
            if (cmd_param.first == "vellimit") {
                auto a = matrixParam(cmd_param.first);
                if (a.size() == 6) {
                    std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                } else {
                    THROW_FILE_LINE("");
                }
            }else if (cmd_param.first == "Mass"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 6) {
                    std::copy(m.data(), m.data() + 6, imp_->M);
                }else if (m.size() == 1) {
                    std::copy(m.data(), m.data()+1, imp_->M);
                    std::copy(m.data(), m.data() + 1, imp_->M+1);
                    std::copy(m.data(), m.data() + 1, imp_->M+2);
                }else{
                    THROW_FILE_LINE("");
                }
            }else if(cmd_param.first == "Damp"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 6) {
                    std::copy(m.data(), m.data() + 6, imp_->B);
                }else if (m.size() == 1) {
                    std::copy(m.data(), m.data()+1, imp_->B);
                    std::copy(m.data(), m.data() + 1, imp_->B+1);
                    std::copy(m.data(), m.data() + 1, imp_->B+2);
                }else{
                    THROW_FILE_LINE("");
                }
            }
        }
        for (auto &option : motorOptions())//???
        {
            option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
        }
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::cout<<"prepare finished"<<std::endl;
    }
    auto DragTrans::executeRT() ->int
    {
        const int FS_NUM = 7;
        static std::size_t motionNum = controller()->motionPool().size();
        // end-effector //
        auto &ee = model()->generalMotionPool()[0];
        // Function Pointer
//        auto &cout = controller()->mout();
//        auto &lout = controller()->lout();
        char eu_type[4]{'1', '2', '3', '\0'};
        // the lambda function to get the force
        auto get_force_data = [&](float *data){
            for (std::size_t i =0; i< motionNum;++i)
            {
                this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
            }
        };
        for(std::size_t i =0; i<motionNum; ++i)
        {
            model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
        }
        if (model()->solverPool().at(1).kinPos()){
            std::cout<<"forward kinematic failed";
            return -1;
        }else{
            ee.updMpm();
        }
        //first loop record
        if(count() ==1)
        {
            //get the current end effector position
             ee.getMpm(imp_->pm_init);
            //set the log file
            controller()->logFileRawName("motion_replay");
            //get the force transformation matrix in tool frame
            double theta = (-imp_->theta_setup) * PI / 180;
            double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
            s_pq2pm(pq_setup, imp_->fs2tpm);
            get_force_data(imp_->init_force);
        }
        //减去力传感器初始偏置
        float force_data[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double force_data_double[6];
        get_force_data(force_data);
        for (int i = 0; i < 6; i++)
            force_data[i] -= imp_->init_force[i];
        for (int i = 0; i < 6; ++i)force_data_double[i] = static_cast<double>(force_data[i]);
        //获取每个周期末端所受的力
        double xyz_temp[3]{force_data_double[0], force_data_double[1], force_data_double[2]}, abc_temp[3]{
                force_data_double[3], force_data_double[4], force_data_double[5]};
        //get the position of tcp
        double pm_begin[16];
        //transform the force to world frame, store in imp_->force_target
        ee.updMpm();
        imp_->tool->getPm(*imp_->wobj, pm_begin);
        double t2bpm[16];
        s_pm_dot_pm(pm_begin, imp_->fs2tpm, t2bpm);
        double net_force[6];
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, net_force, 1);
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, net_force + 3, 1);
        //print the force of z
        if (count() % 100 == 0) {
            std::cout << "fz:" << net_force[0] ;//<< " v_tcp[2] : "<<v_tcp[2];
            std::cout<<std::endl;
        }
        double s_tcp[6];
        //get the position of tool center
        model()->generalMotionPool().at(0).getMpe(s_tcp,eu_type);
        double a[6]{0,0,0,0,0,0};
        double v[6]{0,0,0,0,0,0};
        double x[6]{0,0,0,0,0,0};

        for(std::size_t i =0 ;i < 3 ; i++){
            a[i] = (net_force[i])/imp_->M[i];//- imp_->K * (s_tcp[2] - x_r))/imp_->M;
            v[i] = a[i] * imp_->loop_period * imp_->loop_period + imp_->B[i]*imp_->last_v[i];
            imp_->last_v[i] = v[i];
            x[i] = s_tcp[i] + v[i];
            //prevent the abrupt change
            if(abs(v[i]) > imp_->maxTrans){
                x[i]=v[i]>0?(s_tcp[i]+imp_->maxTrans):(s_tcp[i]-imp_->maxTrans);
                v[i]=v[i]>0?imp_->maxTrans:-imp_->maxTrans;
                imp_->last_v[i] = v[i];
                {std::cout<<"speed reaches the maximum value"<<std::endl;}
            }
            s_tcp[i] = x[i];
        }

        //inverse kinematic calculation
        ee.setMpe(s_tcp, eu_type);
        model()->solverPool()[0].kinPos();
        double x_joint[6];
        for(std::size_t i = 0; i<motionNum; ++i)
        {
            model()->motionPool()[i].updMp();
            x_joint[i] = model()->motionPool()[i].mp();
            controller()->motionPool()[i].setTargetPos(x_joint[i]);
        }

        if(count() > 20000){
            std::cout<<"finished"<<std::endl;
            return 0;
        }

        return 1;
    }
    auto DragTrans::collectNrt() -> void{}
    DragTrans::~DragTrans() = default;
    DragTrans::DragTrans(const DragTrans &other) = default;

    /*-----------DragRot------------*/
    DragRot::DragRot(const std::string &name){
        aris::core::fromXmlString(command(),
                                  "<Command name=\"dragrot\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                  "		<Param name=\"tool\" default=\"tool0\"/>"
                                  "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                  "     <Param name=\"Mass\" default=\"{0.3,0.3,0.3,0.0005,0.0005,0.0005}\" abbreviation=\"m\"/>"
                                  "     <Param name=\"Damp\" default=\"{0.9,0.9,0.9,0.2,0.2,0.2}\" abbreviation=\"b\"/>"
                                  "	</GroupParam>"
                                  "</Command>"
                                  );
    }
    struct DragRotParam{
        aris::dynamic::Marker * tool, * wobj;
        //parameters in prepareNT
        double x_e; //environment position
        double vel_limit[6];//the velocity limitation of motors
        //parameters in excuteRT
        double pm_init[16];//the position matrix of the tool center in world frame
        double theta_setup = -90;// the install angle of force sensor
        double pos_setup = 0.061;// the install position of force sensor
        double fs2tpm[16]; // force sensor to tool center position matrix
        float init_force[6]; // compensate the gravity of tool
        double B[6]{0.25,0.25,0.25,0.0,0.0,0.0};//the damping factor of end effector0.7 has a better performance
        double M[6]{0.25,0.25,0.25,1,1,1};//0.1 has a better performance
        double loop_period = 1e-3;
        double last_v[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    };
    struct DragRot::Imp : public DragRotParam{};
    auto DragRot::prepareNrt() -> void
    {
        std::cout<<"prepare begin"<<std::endl;
        imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
        imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
        imp_-> x_e = 0;
        for (auto cmd_param : cmdParams()) {
            if (cmd_param.first == "vellimit") {
                auto a = matrixParam(cmd_param.first);
                if (a.size() == 6) {
                    std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                } else {
                    THROW_FILE_LINE("");
                }
            }else if (cmd_param.first == "Mass"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 6) {
                    std::copy(m.data(), m.data() + 6, imp_->M);
                }else if (m.size() == 1) {
                    std::copy(m.data(), m.data()+1, imp_->M+3);
                    std::copy(m.data(), m.data() + 1, imp_->M+4);
                    std::copy(m.data(), m.data() + 1, imp_->M+5);
                }else{
                    THROW_FILE_LINE("");
                }
            }else if(cmd_param.first == "Damp"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 6) {
                    std::copy(m.data(), m.data() + 6, imp_->B);
                }else if (m.size() == 1) {
                    std::copy(m.data(), m.data()+1, imp_->B+3);
                    std::copy(m.data(), m.data() + 1, imp_->B+4);
                    std::copy(m.data(), m.data() + 1, imp_->B+5);
                }else{
                    THROW_FILE_LINE("");
                }
            }
        }
        for (auto &option : motorOptions())//???
        {
            option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
        }
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::cout<<"prepare finished"<<std::endl;
    }
    auto DragRot::executeRT() ->int
    {
        const int FS_NUM = 7;
        static std::size_t motionNum = controller()->motionPool().size();
        // end-effector //
        auto &ee = model()->generalMotionPool()[0];
        // Function Pointer
//        auto &cout = controller()->mout();
//        auto &lout = controller()->lout();
        char eu_type[4]{'1', '2', '3', '\0'};
        // the lambda function to get the force
        auto get_force_data = [&](float *data){
            for (std::size_t i =0; i< motionNum;++i)
            {
                this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
            }
        };
        //the function to update model according to real motor and excecute one forward kinematic


        for(std::size_t i =0; i<motionNum; ++i)
        {
            model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
        }
        if (model()->solverPool().at(1).kinPos()){
            std::cout<<"forward kinematic failed";
            return -1;
        }else{
            ee.updMpm();
        }
        //first loop record
        if(count() ==1)
        {
            //get the current end effector position
            ee.getMpm(imp_->pm_init);
            //set the log file
            controller()->logFileRawName("motion_replay");
            //get the force transformation matrix in tool frame
            double theta = (-imp_->theta_setup) * PI / 180;
            double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
            s_pq2pm(pq_setup, imp_->fs2tpm);
            get_force_data(imp_->init_force);
        }
        //减去力传感器初始偏置
        float force_data[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double force_data_double[6];
        get_force_data(force_data);
        for (int i = 0; i < 6; i++)
            force_data[i] -= imp_->init_force[i];
        for (int i = 0; i < 6; ++i)force_data_double[i] = static_cast<double>(force_data[i]);
        //获取每个周期末端所受的力
        double xyz_temp[3]{force_data_double[0], force_data_double[1], force_data_double[2]}, abc_temp[3]{
                force_data_double[3], force_data_double[4], force_data_double[5]};
        //get the position of tcp
        double pm_begin[16];
        //transform the force to world frame, store in imp_->force_target
        model()->generalMotionPool().at(0).updMpm();
        imp_->tool->getPm(*imp_->wobj, pm_begin);
        double t2bpm[16];
        s_pm_dot_pm(pm_begin, imp_->fs2tpm, t2bpm);
        double net_force[6];
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, net_force, 1);
        s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, net_force + 3, 1);

        double s_tcp[6];
        //get the position of tool center
        model()->generalMotionPool().at(0).getMpe(s_tcp,eu_type);
        double a[6]{0,0,0,0,0,0};
        double v[6]{0,0,0,0,0,0};

        //rotation part
        //substrate the lower limit

        for(std::size_t i=3 ; i < 6; i++){
            double lowerbound = 0.003;
            if(abs(net_force[i])>lowerbound){
                net_force[i] -= lowerbound;
            }else{
                net_force[i] = 0;
            }
            //get the desired angular acceleration, get the desired angular velocity
            a[i] = (net_force[i])/imp_->M[i];//- imp_->K * (s_tcp[2] - x_r))/imp_->M;
            v[i] = a[i] * imp_->loop_period * imp_->loop_period + imp_->B[i]*imp_->last_v[i];
            imp_->last_v[i] = v[i];
        }
        double re[3]{0,0,0};
        std::copy(s_tcp+3,s_tcp+6,re);
        double rm_init[9];
        s_re2rm(re,rm_init,eu_type);
        double rm[9]{0,0,0,0,0,0,0,0,0};
        double omega[3]{v[3],v[4],v[5]};

        //get the scale of angular velocity
        double scale = s_norm(3,omega);
        //scale upper limit
        double rot_max = 0.0005;
        if(scale >= rot_max){
            scale = rot_max;
            std::cout<<"rot reach upper bound"<<std::endl;
        }
        if(scale != 0.0){
            //get the unit vector of the angular velocity
            s_nv(3,1/scale,omega);
            double skew[9]{0,0,0,0,0,0,0,0,0};
            s_cm3(omega,skew);
            s_eye(3,rm);
            s_mma(3,3,3,(1.0-std::cos(scale)),skew,skew,rm);
            s_ma(3,3,std::sin(scale),skew,rm);
            double result[9]{0,0,0,0,0,0,0,0,0};
            s_mma(3,3,3,rm,rm_init,result);
            //avoid zero torque
            if(!std::isnan(result[0])){
                s_rm2re(result,re,eu_type);
                std::copy(re,re+3,s_tcp+3);
               // dsp(3,3,result);
            }
        }
        if(count()%100 == 0){
            std::cout<<net_force[3]<<"   "<<net_force[4]<<"   "<<net_force[5]<<"   "<<std::endl;
        }
        //inverse kinematic calculation
        ee.setMpe(s_tcp, eu_type);
        model()->solverPool()[0].kinPos();
        double x_joint[6];
        for(std::size_t i = 0; i<motionNum; ++i)
        {
            model()->motionPool()[i].updMp();
            x_joint[i] = model()->motionPool()[i].mp();
            controller()->motionPool()[i].setTargetPos(x_joint[i]);

        }
        if(count() > 30000){
            std::cout<<"finished"<<std::endl;
            return 0;
        }
        return 1;
    }
    auto DragRot::collectNrt() -> void{}
    DragRot::~DragRot() = default;
    DragRot::DragRot(const DragRot &other) = default;

        const static double V = 0.1; //0.1m/s
        const static double A = 0.1;
        static double v = 0.1;
        static double a = 0.1;
        const static double vel_joint_limit[6]{0.3,0.3,0.3,0.6,0.6,1.0};
        const static double acc_joint_limit[6]{0.3,0.3,0.3,0.6,0.6,1.0};


        double moveto(double q0, double qf, double t0, double t){
            //normal case
            std::cout<<"q0 : "<<q0<<std::endl;
            std::cout<<"qf : "<<qf<<std::endl;
            if(qf - q0 < 0){
                v = - V;
                a = - A;
            }else{
                v = V;
                a = A;
            }
            t = (t - t0)* 0.001;
            if(abs(qf - q0) > (V * V / A)){
                double tf = v / a + (qf - q0) / v;
                std::cout<<"tf : "<<tf<<std::endl;
                double tb = tf - abs((qf - q0)/ v);
                std::cout<<"tb : "<<tb<<std::endl;
                double s = 0;
                std::cout<<"t : "<<t<<std::endl;
                if(t>=tf){
                    s = qf;
                    std::cout<<"stage 4 "<<std::endl;
                }else if((t) <= tb){
                    s = q0 + 0.5*a*t*t;
                    std::cout<<"stage 1 "<<std::endl;
                }else if((t)> tf - tb){
                    s = qf - 0.5*a*tf*tf + a*tf*t - 0.5*a*t*t;
                    std::cout<<"stage 3 "<<std::endl;
                }else{
                    s = 0.5*(qf + q0 - v*tf) + v*t;
                    std::cout<<"stage 2 "<<std::endl;
                }
                std::cout<<"s : "<<s<<std::endl;
                return s;
            }else{
                double tb = std::sqrt(abs((q0 - qf)/a));
                double vmax = (qf - q0)>0?std::sqrt(abs((q0 - qf ) * a)):-std::sqrt(abs((q0 - qf ) * a));
                double s = 0;
                std::cout<<"tb : "<<tb<<std::endl;
                std::cout<<"vmax : "<<vmax<<std::endl;
                std::cout<<"t : "<<t<<std::endl;
                if(t>=2*tb){
                    s = qf;
                    std::cout<<"stage 3 "<<std::endl;
                }else if( (t) < tb ){
                    s =  q0 + 0.5 * a * t * t;
                    std::cout<<"stage 1 "<<std::endl;
                }else{
                    s = 2*q0 - qf + 2 * vmax * t - 0.5 * a * t * t; //?
                    std::cout<<"stage 2 "<<std::endl;
                }
                std::cout<<"s : "<<s<<std::endl;
                return s;
            }
        }

        double moveto(double q0, double qf, double t0, double t, double alim, double vlim){
            //normal case
            alim = abs(alim);
            vlim = abs(vlim);
            std::cout<<"q0 : "<<q0<<std::endl;
            std::cout<<"qf : "<<qf<<std::endl;
            if(qf - q0 < 0){
                v = - vlim;
                a = - alim;
            }else{
                v = vlim;
                a = alim;
            }
            t = (t - t0)* 0.001;
            if(abs(qf - q0) > (vlim * vlim / alim)){
                double tf = v / a + (qf - q0) / v;
                std::cout<<"tf : "<<tf<<std::endl;
                double tb = tf - abs((qf - q0)/ v);
                std::cout<<"tb : "<<tb<<std::endl;
                double s = 0;
                std::cout<<"t : "<<t<<std::endl;
                if(t>=tf){
                    s = qf;
                    std::cout<<"stage 4 "<<std::endl;
                }else if((t) <= tb){
                    s = q0 + 0.5*a*t*t;
                    std::cout<<"stage 1 "<<std::endl;
                }else if((t)> tf - tb){
                    s = qf - 0.5*a*tf*tf + a*tf*t - 0.5*a*t*t;
                    std::cout<<"stage 3 "<<std::endl;
                }else{
                    s = 0.5*(qf + q0 - v*tf) + v*t;
                    std::cout<<"stage 2 "<<std::endl;
                }
                std::cout<<"s : "<<s<<std::endl;
                return s;
            }else{
                double tb = std::sqrt(abs((q0 - qf)/a));
                double vmax = (qf - q0)>0?std::sqrt(abs((q0 - qf ) * a)):-std::sqrt(abs((q0 - qf ) * a));
                double s = 0;
                std::cout<<"tb : "<<tb<<std::endl;
                std::cout<<"vmax : "<<vmax<<std::endl;
                std::cout<<"t : "<<t<<std::endl;
                if(t>=2*tb){
                    s = qf;
                    std::cout<<"stage 3 "<<std::endl;
                }else if( (t) < tb ){
                    s =  q0 + 0.5 * a * t * t;
                    std::cout<<"stage 1 "<<std::endl;
                }else{
                    s = 2*q0 - qf + 2 * vmax * t - 0.5 * a * t * t; //?
                    std::cout<<"stage 2 "<<std::endl;
                }
                std::cout<<"s : "<<s<<std::endl;
                return s;
            }
        }

        double moveto(const double* q0s, const double* qfs, double t0, double t, const double* alims, const double* vlims, double * result){
            //input check
            if(sizeof (q0s)!= 6|| sizeof(qfs)!=6 || sizeof(alims)!= 6|| sizeof(vlims) !=6|| sizeof(result) != 6){
                std::cout<<"input dimension should be six!!!"<<std::endl;
                return false;
            }
            double alim, vlim, q0, qf;
            double num_of_complete = 0;


            for(std::size_t i = 0; i < 6; i++){
                //normal case
                alim = abs(alims[i]);
                vlim = abs(vlims[i]);
                q0 = q0s[i];
                qf = qfs[i];
                if(qf - q0 < 0){
                    v = - vlim;
                    a = - alim;
                }else{
                    v = vlim;
                    a = alim;
                }
                t = (t - t0)* 0.001;
                if(abs(qf - q0) > (vlim * vlim / alim)){
                    double tf = v / a + (qf - q0) / v;
                    std::cout<<"tf : "<<tf<<std::endl;
                    double tb = tf - abs((qf - q0)/ v);
                    std::cout<<"tb : "<<tb<<std::endl;
                    double s = 0;
                    std::cout<<"t : "<<t<<std::endl;
                    if(t>=tf){
                        s = qf;
                        num_of_complete += 1;
                        std::cout<<"stage 4 "<<std::endl;
                    }else if((t) <= tb){
                        s = q0 + 0.5*a*t*t;
                        std::cout<<"stage 1 "<<std::endl;
                    }else if((t)> tf - tb){
                        s = qf - 0.5*a*tf*tf + a*tf*t - 0.5*a*t*t;
                        std::cout<<"stage 3 "<<std::endl;
                    }else{
                        s = 0.5*(qf + q0 - v*tf) + v*t;
                        std::cout<<"stage 2 "<<std::endl;
                    }
                    std::cout<<"s : "<<s<<std::endl;
                    result[i] = s;
                }else{
                    double tb = std::sqrt(abs((q0 - qf)/a));
                    double vmax = (qf - q0)>0?std::sqrt(abs((q0 - qf ) * a)):-std::sqrt(abs((q0 - qf ) * a));
                    double s = 0;
                    std::cout<<"tb : "<<tb<<std::endl;
                    std::cout<<"vmax : "<<vmax<<std::endl;
                    std::cout<<"t : "<<t<<std::endl;
                    if(t>=2*tb){
                        s = qf;
                        num_of_complete +=1;
                        std::cout<<"stage 3 "<<std::endl;
                    }else if( (t) < tb ){
                        s =  q0 + 0.5 * a * t * t;
                        std::cout<<"stage 1 "<<std::endl;
                    }else{
                        s = 2*q0 - qf + 2 * vmax * t - 0.5 * a * t * t; //?
                        std::cout<<"stage 2 "<<std::endl;
                    }
                    std::cout<<"s : "<<s<<std::endl;
                    result[i] = s;
                }
            }
            return num_of_complete;
        }

            /*-----------Move to ------------*/
        Moveto::Moveto(const std::string &name){
            aris::core::fromXmlString(command(),
                                      "<Command name=\"moveto\">"
                                      "	<GroupParam>"
                                      "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                      "		<Param name=\"tool\" default=\"tool0\"/>"
                                      "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                      "     <Param name=\"Mass\" default=\"{0.3,0.3,0.3,0.0005,0.0005,0.0005}\" abbreviation=\"m\"/>"
                                      "     <Param name=\"Damp\" default=\"{0.9,0.9,0.9,0.2,0.2,0.2}\" abbreviation=\"b\"/>"
                                      "	</GroupParam>"
                                      "</Command>"
                                      );
        }
//        for(int i =0; i< motionNum;++i)
//        {
//            std::cout  << "stcp:" << stcp[i] << std::endl;
//        }
        struct MovetoParam{
            aris::dynamic::Marker * tool, * wobj;
            //parameters in prepareNT
            double x_e; //environment position
            double vel_limit[6];//the velocity limitation of motors
            //parameters in excuteRT
            double pm_init[16];//the position matrix of the tool center in world frame
            double theta_setup = -90;// the install angle of force sensor
            double pos_setup = 0.061;// the install position of force sensor
            double fs2tpm[16]; // force sensor to tool center position matrix
            double start = 0;
            double start_pos[6];
        };
        struct Moveto::Imp : public MovetoParam{};
        auto Moveto::prepareNrt() -> void
        {
            std::cout<<"prepare begin"<<std::endl;
            imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
            imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
            imp_-> x_e = 0;
            for (auto cmd_param : cmdParams()) {
                if (cmd_param.first == "vellimit") {
                    auto a = matrixParam(cmd_param.first);
                    if (a.size() == 6) {
                        std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                    } else {
                        THROW_FILE_LINE("");
                    }
                }
            }
            for (auto &option : motorOptions())//???
            {
                option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
            }
            std::vector <std::pair<std::string, std::any>> ret_value;
            ret() = ret_value;
            std::cout<<"prepare finished"<<std::endl;
        }
        auto Moveto::executeRT() ->int
        {
            std::cout << "moveto!" << std::endl;
            static std::size_t motionNum = controller()->motionPool().size();
            // end-effector //
            auto &ee = model()->generalMotionPool()[0];
            // Function Pointer
    //        auto &cout = controller()->mout();
    //        auto &lout = controller()->lout();
            char eu_type[4]{'1', '2', '3', '\0'};
            // the lambda function to get the force
            //the function to update model according to real motor and excecute one forward kinematic
            for(std::size_t i =0; i<motionNum; ++i)
            {
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
            }
            if (model()->solverPool().at(1).kinPos()){
                std::cout<<"forward kinematic failed";
                return -1;
            }else{
                ee.updMpm();
            }
            //first loop record
            if(count() ==1)
            {
                //get the current end effector position
                ee.getMpm(imp_->pm_init);
                ee.getMpe(imp_->start_pos,eu_type);
                //set the log file
                controller()->logFileRawName("motion_replay");
                //get the force transformation matrix in tool frame
                double theta = (-imp_->theta_setup) * PI / 180;
                double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
                s_pq2pm(pq_setup, imp_->fs2tpm);
                imp_->start = count();
            }
            double s_tcp[6];
            //get the position of tool center
            std::copy(imp_->start_pos, imp_->start_pos+6, s_tcp);
            s_tcp[2] = moveto(imp_->start_pos[2], imp_->start_pos[2] + 0.05, imp_->start, count());
            //inverse kinematic calculation
            ee.setMpe(s_tcp, eu_type);
            model()->solverPool()[0].kinPos();
            double x_joint[6];
            for(std::size_t i = 0; i<motionNum; ++i)
            {
                model()->motionPool()[i].updMp();
                x_joint[i] = model()->motionPool()[i].mp();                
                if(x_joint[5]<0){x_joint[5] += 2*PI;std::cout<<"x_joint[5] : "<<x_joint[5];}
                if(x_joint[4]<0){x_joint[4] += 2*PI;std::cout<<"x_joint[4] : "<<x_joint[4];}
                controller()->motionPool()[i].setTargetPos(x_joint[i]);
            }
            if(count() > 5000){
                std::cout<<"finished"<<std::endl;
                return 0;
            }
            return 1;
        }
        auto Moveto::collectNrt() -> void{}
        Moveto::~Moveto() = default;
        Moveto::Moveto(const Moveto &other) = default;

        /*-----------Trapezoid ------------*/
    Trapezoid::Trapezoid(const std::string &name){
        aris::core::fromXmlString(command(),
                                  "<Command name=\"trap\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                  "		<Param name=\"tool\" default=\"tool0\"/>"
                                  "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                  "     <Param name=\"Motorindex\" default=\"{5}\" abbreviation=\"n\"/>"
                                  "     <Param name=\"offset\" default=\"{0.5}\" abbreviation=\"o\"/>"
                                  "     <Param name=\"lim\" default=\"{0.1,0.1}\" abbreviation=\"l\"/>"
                                  "	</GroupParam>"
                                  "</Command>"
                                  );
    }

    struct TrapezoidParam{
        aris::dynamic::Marker * tool, * wobj;
        //parameters in prepareNT
        double x_e; //environment position
        double vel_limit[6];//the velocity limitation of motors
        //parameters in excuteRT
        double pm_init[16];//the position matrix of the tool center in world frame
        double theta_setup = -90;// the install angle of force sensor
        double pos_setup = 0.061;// the install position of force sensor
        double fs2tpm[16]; // force sensor to tool center position matrix
        double start = 0;
        double start_pos_joint[6];
        double start_pos_cartesian[6];
        double motorindex = -1;
        double offset = -1;
        double lim[2];
    };
    struct Trapezoid::Imp : public TrapezoidParam{};
    auto Trapezoid::prepareNrt() -> void
    {
        std::cout<<"prepare begin"<<std::endl;
        imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
        imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
        imp_-> x_e = 0;
        for (auto cmd_param : cmdParams()) {
            if (cmd_param.first == "vellimit") {
                auto a = matrixParam(cmd_param.first);
                if (a.size() == 6) {
                    std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                } else {
                    THROW_FILE_LINE("");
                }
            }
            else if (cmd_param.first == "Motorindex"){
                    auto m = matrixParam(cmd_param.first);
                    if (m.size() == 1) {
                        imp_->motorindex = *m.data();
                    }else{
                        THROW_FILE_LINE("");
                    }
            }
            else if (cmd_param.first == "offset"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 1) {
                        imp_->offset = *m.data();
                }else{
                    THROW_FILE_LINE("");
                }
            }
            else if (cmd_param.first == "lim"){
                auto m = matrixParam(cmd_param.first);
                if (m.size() == 2) {
                    std::copy(m.data(), m.data() + 1, imp_->lim);
                    std::copy(m.data()+1, m.data() + 2, imp_->lim+1);
                }else{
                    THROW_FILE_LINE("");
                }
            }
        }
        for (auto &option : motorOptions())//???
        {
            option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
        }
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::cout<<"prepare finished"<<std::endl;
    }
    auto Trapezoid::executeRT() ->int
    {
        std::cout << "moveto!" << std::endl;
        static std::size_t motionNum = controller()->motionPool().size();
        // end-effector //
        auto &ee = model()->generalMotionPool()[0];
        // Function Pointer
//        auto &cout = controller()->mout();
//        auto &lout = controller()->lout();
        char eu_type[4]{'1', '2', '3', '\0'};
        //the lambda function to get the force
        //the function to update model according to real motor and excecute one forward kinematic


        for(std::size_t i =0; i<motionNum; ++i)
        {
            model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
        }
        if (model()->solverPool().at(1).kinPos()){
            std::cout<<"forward kinematic failed";
            return -1;
        }else{
            ee.updMpm();
        }
        //first loop record
        if(count() ==1)
        {
            for(std::size_t i =0; i<motionNum; ++i)
            {
                imp_->start_pos_joint[i] = model()->motionPool()[i].mp();
            }
            //get the current end effector position
            ee.getMpm(imp_->pm_init);
            ee.getMpe(imp_->start_pos_cartesian,eu_type);
            //set the log file
            controller()->logFileRawName("motion_replay");
            //get the force transformation matrix in tool frame
            double theta = (-imp_->theta_setup) * PI / 180;
            double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
            s_pq2pm(pq_setup, imp_->fs2tpm);
            imp_->start = count();
        }
        double target = imp_->start_pos_joint[int(imp_->motorindex)]+imp_->offset;
        double newPos = moveto(imp_->start_pos_joint[int(imp_->motorindex)],target,imp_->start, count(),imp_->lim[0], imp_->lim[1]);
        controller()->motionPool()[std::size_t(imp_->motorindex)].setTargetPos(newPos);
//        double s_tcp[6];
//        //get the position of tool center
//        std::copy(imp_->start_pos_cartesian, imp_->start_pos_cartesian+6, s_tcp);
//        s_tcp[2] = moveto(imp_->start_pos_cartesian[2], imp_->start_pos_cartesian[2] + 0.05, imp_->start, count());
//        //inverse kinematic calculation
//        ee.setMpe(s_tcp, eu_type);
//        model()->solverPool()[0].kinPos();
//        double x_joint[6];
//        for(std::size_t i = 0; i<motionNum; ++i)
//        {
//            model()->motionPool()[i].updMp();
//            x_joint[i] = model()->motionPool()[i].mp();
//            if(x_joint[5]<0){x_joint[5] += 2*PI;std::cout<<"x_joint[5] : "<<x_joint[5];}
//            if(x_joint[4]<0){x_joint[4] += 2*PI;std::cout<<"x_joint[4] : "<<x_joint[4];}
//            controller()->motionPool()[i].setTargetPos(x_joint[i]);
//        }
        if(count() > 4000 && newPos == target){
            std::cout<<"finished"<<std::endl;
            return 0;
        }
        return 1;
    }
    auto Trapezoid::collectNrt() -> void{}
    Trapezoid::~Trapezoid() = default;
    Trapezoid::Trapezoid(const Trapezoid &other) = default;



        /*-----------FitTog------------*/
        FitTog::FitTog(const std::string &name){
            aris::core::fromXmlString(command(),
                                      "<Command name=\"fit\">"
                                      "	<GroupParam>"
                                      "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                      "		<Param name=\"tool\" default=\"tool0\"/>"
                                      "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                      "     <Param name=\"Mass\" default=\"{0.3,0.3,0.3,1.0,1.0,1.0}\" abbreviation=\"m\"/>"
                                      "     <Param name=\"Damp\" default=\"{0.9,0.9,0.9,0.0,0.0,0.0}\" abbreviation=\"b\"/>"
                                      "	</GroupParam>"
                                      "</Command>"
                                      );
        }
        struct FitTogParam{
            aris::dynamic::Marker * tool, * wobj;
            //parameters in prepareNT
            double x_e; //environment position
            double vel_limit[6];//the velocity limitation of motors
            //parameters in excuteRT
            double pm_init[16];//the position matrix of the tool center in world frame
            double start_Pos[6];//ee eu_type postion and angle
            double theta_setup = -90;// the install angle of force sensor
            double pos_setup = 0.061;// the install position of force sensor
            double fs2tpm[16]; //
            float init_force[6]; // compensate the gravity of tool
            double ke = 220000;
//            double B = 0.7;//the damping factor of end effector0.7 has a better performance
//            double M = 0.1;//0.1 has a better performance
            double K = 1;//
            double loop_period = 1e-3;
            double B[6]{0.25,0.25,0.7,0.0,0.0,0.0};//the damping factor of end effector0.7 has a better performance
            double M[6]{0.1,0.1,0.1,0.1,0.1,0.1};//0.1 has a better performance
            double last_v[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double maxTrans =0.00010;
            int HOME = 0;
            double angle_offset = 1.5 * 3.1415926535 / 180;
            bool contacted = false;// flag to show if the end effector contact with the surface
            double desired_force = 0.3;
            double j6_start_angle;//when connect, remember j6 start angle
            long contacted_start_count;//before contact count()
            long get_init_fs_count;//when HOME = 1, remember fs data
            double contact_pos = 0.105406;//sun gear contact plant gear
            double final_pos = 0.100957;//fit together finish position
        };
        struct FitTog::Imp : public FitTogParam{};
        auto FitTog::prepareNrt() -> void
        {
            std::cout<<"prepare begin"<<std::endl;
            imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
            imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
            imp_-> x_e = 0;
            for (auto cmd_param : cmdParams()) {
                if (cmd_param.first == "vellimit") {
                    auto a = matrixParam(cmd_param.first);
                    if (a.size() == 6) {
                        std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                    } else {
                        THROW_FILE_LINE("");
                    }
                }else if (cmd_param.first == "Mass"){
                    auto m = matrixParam(cmd_param.first);
                    if (m.size() == 6) {
                        std::copy(m.data(), m.data() + 6, imp_->M);
                    }else if (m.size() == 1) {
                        std::copy(m.data(), m.data() + 1, imp_->M);
                        std::copy(m.data(), m.data() + 1, imp_->M+1);
                        std::copy(m.data(), m.data() + 1, imp_->M+2);
                    }else{
                        THROW_FILE_LINE("");
                    }
                }else if(cmd_param.first == "Damp"){
                    auto m = matrixParam(cmd_param.first);
                    if (m.size() == 6) {
                        std::copy(m.data(), m.data() + 6, imp_->B);
                    }else if (m.size() == 1) {
                        std::copy(m.data(), m.data() + 1, imp_->B);
                        std::copy(m.data(), m.data() + 1, imp_->B+1);
                        std::copy(m.data(), m.data() + 1, imp_->B+2);
                    }else{
                        THROW_FILE_LINE("");
                    }
                }
            }
            for (auto &option : motorOptions())//???
            {
                option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
            }
            std::vector <std::pair<std::string, std::any>> ret_value;
            ret() = ret_value;
            std::cout<<"prepare finished"<<std::endl;
        }
        auto FitTog::executeRT() ->int
        {
            const int FS_NUM = 7;
            static std::size_t motionNum = controller()->motionPool().size();
            // end-effector //
            auto &ee = model()->generalMotionPool()[0];
            // Function Pointer
    //        auto &cout = controller()->mout();
    //        auto &lout = controller()->lout();
            char eu_type[4]{'1', '2', '3', '\0'};
            // the lambda function to get the force
            auto get_force_data = [&](float *data){
                for (std::size_t i =0; i< motionNum;++i)
                {
                    this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
                }
            };
            //the function to update model according to real motor and excecute one forward kinematic
            auto forwardPos = [&](){
                for(std::size_t i =0; i<motionNum; ++i)
                {
                    model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
                }
                if(model()->solverPool()[1].kinPos())
                {
                    std::cout<<"forward kinematic failed, exit"<<std::endl;
                }
                ee.updMpm();
            };
            //safty check the position change
            auto checkPos = [&](double * data)
            {
                for(std::size_t i = 0; i< motionNum; i++)
                {
                    if(abs(data[i] - controller()->motionPool()[i].targetPos()) > imp_->vel_limit[i]){
                        std::cout<<"joint "<<i<<" move too fast"<<std::endl;
                        std::cout<<"pi-1 = "<<controller()->motionPool()[i].targetPos()<<std::endl;
                        std::cout<<"pi = "<<data[i]<<std::endl;
                        std::cout<<"vellimit = "<<imp_->vel_limit[i]<<std::endl;
                        return false;
                    }
                }
                return true;
            };
            for(std::size_t i =0; i<motionNum; ++i)
            {
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
            }
            if (model()->solverPool().at(1).kinPos()){
                std::cout<<"forward kinematic failed";
                return -1;
            }else{
                ee.updMpm();
            }
            //first loop record
            if(count() ==1)
            {
                double mp0[6];
                for(std::size_t i =0; i< motionNum;++i){
                    mp0[i] = controller()->motionPool()[i].actualPos();
                    std::cout << "mp0" << i << ": " << mp0[i] << std::endl;
                }
                model()->solverPool()[1].kinPos();
                //get the position of tcp
                double pm_begin[16];
                //transform the force to world frame, store in imp_->force_target
                ee.updMpm();
                imp_->tool->getPm(*imp_->wobj, pm_begin);
                double t2bpm[16];
                s_pm_dot_pm(pm_begin, imp_->fs2tpm, t2bpm);
                double s_tcp[6];
                model()->generalMotionPool().at(0).getMpe(s_tcp,eu_type);
                for(std::size_t i = 0; i <  motionNum; ++i){
                    std::cout << "stcp" << i << ": " << s_tcp[i] << std::endl;
                }



                //set the log file
                controller()->logFileRawName("motion_replay");
                //get the force transformation matrix in tool frame
                double theta = (-imp_->theta_setup) * PI / 180;
                double pq_setup[7]{0.0, 0.0, imp_->pos_setup, 0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)};
                s_pq2pm(pq_setup, imp_->fs2tpm);
//                std::cout << "mpstart:" << controller()->motionPool()[5].actualPos() << std::endl;
            }

//            controller()->motionPool()[0].setTargetVel(v);

            //HOME
            double mp[6]{0,0,0,0,0,0};
            if(imp_->HOME == 0)
            {
                for (std::size_t i =0; i< motionNum;++i)
                {
                    if(i != 4){
//                        std::cout << "Mpos:" << controller()->motionPool()[i].targetPos() << std::endl;
                        if(controller()->motionPool()[i].actualPos() >= 0.0001)
                        {

                            mp[i] = controller()->motionPool()[i].targetPos() - 0.0001;
                        }
                        else if(controller()->motionPool()[i].targetPos() <= -0.0001){

                            mp[i] = controller()->motionPool()[i].targetPos() + 0.0001;
                        }
                        else{
                            mp[i] = 0;

                        }
                    }
                    else{
//                        std::cout << "Mpos:" << controller()->motionPool()[i].targetPos() << std::endl;
                        if(controller()->motionPool()[i].actualPos() > 1.57087 - imp_->angle_offset)
                        {

                            mp[i] = controller()->motionPool()[i].targetPos() - 0.0001;
                        }
                        else if(controller()->motionPool()[i].targetPos() <= 1.57075 - imp_->angle_offset){

                            mp[i] = controller()->motionPool()[i].targetPos() + 0.0001;
                        }
                        else{
                            mp[i] = 1.57080 - imp_->angle_offset;

                        }
                    }

                    controller()->motionPool()[i].setTargetPos(mp[i]);
                }

            }

            if(imp_->HOME == 0){
                if(mp[0] == 0.0){
                    if (mp[1] == 0.0){
                        if(mp[2] == 0.0){
                            if(mp[3] == 0.0){
                                if(mp[4] == 1.57080 - imp_->angle_offset){
                                    if(mp[5] == 0.0){
                                        imp_->HOME = 1;
                                        imp_->get_init_fs_count = count();
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(imp_->HOME == 1)
            {
                if(count() == imp_->get_init_fs_count)get_force_data(imp_->init_force);
                //减去力传感器初始偏置
                float force_data[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                double force_data_double[6];
                get_force_data(force_data);
                for (int i = 0; i < 6; i++)
                    force_data[i] -= imp_->init_force[i];
                for (int i = 0; i < 6; ++i)force_data_double[i] = static_cast<double>(force_data[i]);
                //获取每个周期末端所受的力
                double xyz_temp[3]{force_data_double[0], force_data_double[1], force_data_double[2]}, abc_temp[3]{
                        force_data_double[3], force_data_double[4], force_data_double[5]};
                //get the position of tcp
                double pm_begin[16];
                //transform the force to world frame, store in imp_->force_target
                ee.updMpm();
                imp_->tool->getPm(*imp_->wobj, pm_begin);
                double t2bpm[16];
                s_pm_dot_pm(pm_begin, imp_->fs2tpm, t2bpm);
                double net_force[6];
                s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, net_force, 1);
                s_mm(3, 1, 3, t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, net_force + 3, 1);
                //print the force of z
                if (count() % 1000 == 0) {
                    std::cout << "fz:" << net_force[2] ;//<< " v_tcp[2] : "<<v_tcp[2];
                    std::cout<<std::endl;
                }

                if(abs(net_force[2]) > 0.2)
                {                    
                    if (count() % 1000 == 0)
                        {

                            std::cout << " force control start! " ;
                            std::cout << std::endl;
                        }
                    /*
                    * if判断语句的作用：
                    * abs(imp_->force_target[2])>0.1 表示已经与目标物体接触；
                    * else表示还没接触的时候；
                    * 如果没有接触就以指定速度下降，如果产生接触则执行阻抗控制命令；
                    */

                    double s_tcp[6];
                    //get the velocity of tool center
                    double v_tcp[6];
                    model()->generalMotionPool().at(0).getMve(v_tcp,eu_type);
                    //get the position of tool center
                    model()->generalMotionPool().at(0).getMpe(s_tcp,eu_type);
                    //get the x_environment as soon as contact happens
//                    if(!imp_->contacted)
//                        {
//                            imp_->contacted = true;
//                            imp_->x_e = s_tcp[2];
//                            std::cout << "x_e: " << imp_->x_e << std::endl;
//                        }
                    //get the x_reference of
                    double x_r = imp_->x_e - (imp_->desired_force / imp_->ke );
                    //calculate the desired acceleration and velocity
                    double a = (net_force[2] - imp_->desired_force- imp_->B[2] * v_tcp[2] )/imp_->M[2];//- imp_->K * (s_tcp[2] - x_r))/imp_->M;
                    double x = s_tcp[2] + 0.5 * a * imp_->loop_period* imp_->loop_period + v_tcp[2] * imp_->loop_period;
                    double v = (x - s_tcp[2]);
                    double ee_v [6]{0.0,0.0,v,0.0,0.0,0.0};
                    s_tcp[2] = x;
                    ee.setMpe(s_tcp, eu_type);
                    model()->solverPool()[0].kinPos();
                    double x_joint[6];
                    for(std::size_t i = 0; i<motionNum; ++i)
                        {
                            model()->motionPool()[i].updMp();
                            x_joint[i] = model()->motionPool()[i].mp();
                        }
                    if(checkPos(x_joint))
                        {
                            for(std::size_t i = 0; i<motionNum; ++i)
                            {
                                controller()->motionPool()[i].setTargetPos(x_joint[i]);
                            }
                            ee.setMve(ee_v,eu_type);
                        }
                    controller()->motionPool()[5].setTargetPos(imp_->j6_start_angle + 1 * sin((double((count() - imp_->contacted_start_count -1) % 15000))/15000 * PI * 2));
//                    std::cout << imp_->j6_start_angle << "j6pos:" << imp_->j6_start_angle + 0.1 * sin((double((count() - imp_->contacted_start_count - 1) % 15000))/15000 * PI * 2) << std::endl;
                }
                    else
                    {
                        forwardPos();
                        double s_tcp[6];
                        ee.getMpe(s_tcp, eu_type);
                        s_tcp[2] -= 0.00001;
                        //if contact deep enough ,stop
                        if(s_tcp[2] <= 0.101)return 0;
                        ee.setMpe(s_tcp, eu_type);
                        double ee_v[6]{0.0, 0.0, -0.00001, 0.0, 0.0, 0.0};
                        model()->solverPool()[0].kinPos();
                        double x_joint[6];
                        for(std::size_t i = 0; i<motionNum; ++i)
                        {
                            model()->motionPool()[i].updMp();
                            x_joint[i] = model()->motionPool()[i].mp();
                        }
                        if(checkPos(x_joint)){
                            for(std::size_t i = 0; i<motionNum; ++i)
                            {
                                controller()->motionPool()[i].setTargetPos(x_joint[i]);
                            }
                            model()->generalMotionPool()[0].setMve(ee_v, eu_type);
                        }
                        imp_->contacted_start_count = count();
                        imp_->j6_start_angle = controller()->motionPool()[5].actualPos();
                    }
            }
            double stcp[6]{0,0,0,0,0,0};
            ee.updMpm();
            ee.getMpe(stcp, eu_type);
            if(count() % 5000 == 0)
            {
                for (std::size_t i =0; i< motionNum;++i)
                {
                    std::cout  << "stcp" << i  << ":" << stcp[i] << std::endl;
                }
            }

//            if(count() > 20000){
//                std::cout<<"finished"<<std::endl;
//                return 0;
//            }
            return 1;
        }
        auto FitTog::collectNrt() -> void{}
        FitTog::~FitTog() = default;
        FitTog::FitTog(const FitTog &other) = default;

//------------Test--------------//
        Test::Test(const std::string &name){
            aris::core::fromXmlString(command(),
                                      "<Command name=\"test\">"
                                      "	<GroupParam>"
                                      "		<Param name=\"vellimit\" default=\"{0.2,0.2,0.1,0.5,0.5,0.5}\"/>"
                                      "		<Param name=\"tool\" default=\"tool0\"/>"
                                      "		<Param name=\"wobj\" default=\"wobj0\"/>"
                                      "     <Param name=\"Mass\" default=\"{0.3,0.3,0.3,0.0005,0.0005,0.0005}\" abbreviation=\"m\"/>"
                                      "     <Param name=\"Damp\" default=\"{0.9,0.9,0.9,0.2,0.2,0.2}\" abbreviation=\"b\"/>"
                                      "	</GroupParam>"
                                      "</Command>"
                                      );
        }
//        for(int i =0; i< motionNum;++i)
//        {
//            std::cout  << "stcp:" << stcp[i] << std::endl;
//        }
        struct TestParam{
            aris::dynamic::Marker * tool, * wobj;
            //parameters in prepareNT
            double x_e; //environment position
            double vel_limit[6];//the velocity limitation of motors
            //parameters in excuteRT
            double pm_init[16];//the position matrix of the tool center in world frame
            double theta_setup = -90;// the install angle of force sensor
            double pos_setup = 0.061;// the install position of force sensor
            double fs2tpm[16]; // force sensor to tool center position matrix
            double start = 0;
            double start_pos[6];
        };
        struct Test::Imp : public TestParam{};
        auto Test::prepareNrt() -> void
        {
            std::cout<<"prepare begin"<<std::endl;
            imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
            imp_->wobj = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));
            imp_-> x_e = 0;
            for (auto cmd_param : cmdParams()) {
                if (cmd_param.first == "vellimit") {
                    auto a = matrixParam(cmd_param.first);
                    if (a.size() == 6) {
                        std::copy(a.data(), a.data() + 6, imp_->vel_limit);
                    } else {
                        THROW_FILE_LINE("");
                    }
                }
//                else if (cmd_param.first == "Mass"){
//                    auto m = matrixParam(cmd_param.first);
//                    if (m.size() == 6) {
//                        std::copy(m.data(), m.data() + 6, imp_->M);
//                    }else if (m.size() == 1) {
//                        std::copy(m.data(), m.data()+1, imp_->M+3);
//                        std::copy(m.data(), m.data() + 1, imp_->M+4);
//                        std::copy(m.data(), m.data() + 1, imp_->M+5);
//                    }else{
//                        THROW_FILE_LINE("");
//                    }
//                }else if(cmd_param.first == "Damp"){
//                    auto m = matrixParam(cmd_param.first);
//                    if (m.size() == 6) {
//                        std::copy(m.data(), m.data() + 6, imp_->B);
//                    }else if (m.size() == 1) {
//                        std::copy(m.data(), m.data()+1, imp_->B+3);
//                        std::copy(m.data(), m.data() + 1, imp_->B+4);
//                        std::copy(m.data(), m.data() + 1, imp_->B+5);
//                    }else{
//                        THROW_FILE_LINE("");
//                    }
//                }
            }
            for (auto &option : motorOptions())//???
            {
                option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_VEL_CONTINUOUS;
            }
            std::vector <std::pair<std::string, std::any>> ret_value;
            ret() = ret_value;
            std::cout<<"prepare finished"<<std::endl;
        }
        auto Test::executeRT() ->int
        {
            const int FS_NUM = 7;
            static std::size_t motionNum = controller()->motionPool().size();
            // end-effector //
            auto &ee = model()->generalMotionPool()[0];
            char eu_type[4]{'1', '2', '3', '\0'};
            controller()->motionPool()[5].setTargetPos(0);
            if(count() > 5000){
                std::cout<<"finished"<<std::endl;
                return 0;
            }
            return 1;
        }
        auto Test::collectNrt() -> void{}
        Test::~Test() = default;
        Test::Test(const Test &other) = default;

    auto createPlanRoot() -> std::unique_ptr <aris::plan::PlanRoot> {
        std::unique_ptr <aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
        //用户自己开发指令集
//        plan_root->planPool().add<robot::MoveS>();
        plan_root->planPool().add<robot::MoveTest>();
        plan_root->planPool().add<robot::MoveJoint>();
        plan_root->planPool().add<robot::ImpedPos>();
        plan_root->planPool().add<robot::Trapezoid>();
        plan_root->planPool().add<robot::Drag>();
        plan_root->planPool().add<robot::DragTrans>();
        plan_root->planPool().add<robot::DragRot>();
        plan_root->planPool().add<robot::Moveto>();
//        plan_root->planPool().add<robot::Imu>();
        plan_root->planPool().add<robot::FitTog>();
        plan_root->planPool().add<robot::Test>();

        //aris库提供指令集
        plan_root->planPool().add<aris::plan::Enable>();//
        plan_root->planPool().add<aris::plan::Disable>();
        plan_root->planPool().add<aris::plan::Start>();
        plan_root->planPool().add<aris::plan::Stop>();
        plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Clear>();
//        plan_root->planPool().add<aris::server::GetInfo>();
        //kaanh库提供指令集
        plan_root->planPool().add<kaanh::Home>();
        plan_root->planPool().add<kaanh::Sleep>();
        plan_root->planPool().add<kaanh::Recover>();
        plan_root->planPool().add<kaanh::Reset>();
        plan_root->planPool().add<kaanh::MoveAbsJ>();
        plan_root->planPool().add<kaanh::MoveL>();
        plan_root->planPool().add<kaanh::MoveJ>();
        plan_root->planPool().add<kaanh::MoveC>();
        plan_root->planPool().add<kaanh::Get>();
        plan_root->planPool().add<kaanh::Var>();
        plan_root->planPool().add<kaanh::Evaluate>();
        plan_root->planPool().add<kaanh::JogJ1>();
        plan_root->planPool().add<kaanh::JogJ2>();
        plan_root->planPool().add<kaanh::JogJ3>();
        plan_root->planPool().add<kaanh::JogJ4>();
        plan_root->planPool().add<kaanh::JogJ5>();
        plan_root->planPool().add<kaanh::JogJ6>();
        plan_root->planPool().add<kaanh::JogJ7>();
        plan_root->planPool().add<kaanh::JX>();
        plan_root->planPool().add<kaanh::JY>();
        plan_root->planPool().add<kaanh::JZ>();
        plan_root->planPool().add<kaanh::JRX>();
        plan_root->planPool().add<kaanh::JRY>();
        plan_root->planPool().add<kaanh::JRZ>();
        plan_root->planPool().add<kaanh::SetDH>();
        plan_root->planPool().add<kaanh::SetPG>();
        plan_root->planPool().add<kaanh::SetPPath>();
        plan_root->planPool().add<kaanh::SetUI>();
        plan_root->planPool().add<kaanh::SetDriver>();
        plan_root->planPool().add<kaanh::SaveXml>();
        plan_root->planPool().add<kaanh::ScanSlave>();
        plan_root->planPool().add<kaanh::GetEsiPdoList>();
        plan_root->planPool().add<kaanh::SetEsiPath>();
        plan_root->planPool().add<kaanh::GetXml>();
        plan_root->planPool().add<kaanh::SetXml>();
        plan_root->planPool().add<kaanh::SetCT>();
        plan_root->planPool().add<kaanh::SetVel>();
        plan_root->planPool().add<kaanh::Run>();
        plan_root->planPool().add<kaanh::MoveF>();
        plan_root->planPool().add<kaanh::Switch>();
        plan_root->planPool().add<kaanh::CalibFZero>();
        plan_root->planPool().add<CalibT4P>();
        plan_root->planPool().add<CalibT5P>();
        plan_root->planPool().add<CalibT6P>();
        plan_root->planPool().add<CalibW3P>();
        plan_root->planPool().add<kaanh::FCStop>();
        plan_root->planPool().add<kaanh::MoveDJ>();
        plan_root->planPool().add<kaanh::AdmitInit>();
        plan_root->planPool().add<kaanh::Kunwei>();
        //plan_root->planPool().add<kaanh::MoveJoint>();
        return plan_root;
    }
}
