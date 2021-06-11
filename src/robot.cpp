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
        param.active_motor.resize(controller()->motionPool().size(),
                                  false);  //开辟一段空间，有motionPool().size()这么大，每个元素用false进行填充
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
        for (Size i = 0; i < param.active_motor.size(); ++i) {
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
            for (int i = 0; i < param.active_motor.size(); i++) {
                cout << std::setprecision(10) << "targetpos:" << param.step_pjs[i] << "  ";
                cout << std::setprecision(10) << "actualpos:" << controller()->motionPool()[i].actualPos() << "  ";
            }
            cout << std::endl;

        }

        //记录
        auto &lout = controller()->lout();
        for (int i = 0; i < param.active_motor.size(); i++) {
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
        if (count() == 1) {
            // get position from controller //
            for (int i = 0; i < model()->motionPool().size(); ++i) {
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
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

//        pe[0] += dir_ * 0.1 * (1.0 - std::cos(count() / 1000.0 * PI))/2.0;

        // trajectory generation ellipse//
//        TCurve s1(5,2);
//        s1.getCurveParam();
//        pe[0] += dir_*0.1*s1.getTCurve(count());


        TCurve s1(5, 2);
        s1.getCurveParam();
        EllipseTrajectory e1(-1, 0.8, 0, s1);
        e1.getEllipseTrajectory(count());

        //   mout() << "step:" << dir_*0.1 * e1.z_ << " " << dir_*0.1 * e1.x_ << " " << 0.1 * e1.y_ << std::endl;
        //    mout() << "pe:" << pe[0] << " " << pe[1] << " " << pe[2] << std::endl;
//    pe[0] = begin_pe[0] + dir_*0.1 * e1.z_;
//    pe[1] = begin_pe[0] + dir_*0.1 * e1.x_;
//    pe[2] = begin_pe[0] + 0.1 * e1.y_;

        pe[0] += dir_ * s1.getTCurve(count());
        // pe[1] += 0.000005;
        //  pe[2] += 0.000005;

//mout() << "pe:" << pe[0] << " " << pe[1] << " " << pe[2] << std::endl;
//mout()<<std::endl;
        //aris::dynamic::dsp(1,3,pe);




        // inverse kinematic //
        ee.setMpe(pe);
        model()->solverPool()[0].kinPos();

        // get input //
//    double input[6];
        for (int i = 0; i < model()->motionPool().size(); ++i) {
            model()->motionPool()[i].updMp();
            controller()->motionPool()[i].setTargetPos(model()->motionPool()[i].mp());
        }

        // print //
        // if(count() % 100 == 0)
        //    mout() << pe[0] << "  "<< pe[1]<<"  " << pe[2]<<"  "<< pe[3] << "  "<< pe[4]<<"  " << pe[5] << std::endl;

        // log //
        lout() << pe[0] << "  " << pe[1] << "  " << pe[2] << "  " << pe[3] << "  " << pe[4] << "  " << pe[5]
               << std::endl;

        return s1.Tc_ * 1000 - count();

    }

    MoveTest::MoveTest(const std::string &name) : Plan(name) {
        //command().loadXmlStr(
        aris::core::fromXmlString(command(),
                "<Command name=\"mv\">"
                "	<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
                "</Command>");
    }


// 导纳控制 //
    std::atomic_int x = 0;
    std::atomic_int y = 0;
    std::atomic_int z = 0;
    std::atomic_int rx = 0;
    std::atomic_int ry = 0;
    std::atomic_int rz = 0;
    std::atomic_bool enable_mvJoint = true;
    struct MoveJointParam {
        aris::dynamic::Marker *tool, *wobj;
        float realdata_init[6], realdata[6];
        double theta;
        double vel_limit[6], k[6], damping[6], K[6], force_delete[6];
        double fs2tpm[16], t2bpm[16];
        double force_offset[6], force_target[6];
        double force_damp[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double theta_setup = -90;//力传感器安装位置相对tcp偏移角
        double pos_setup = 0.061;//力传感器安装位置相对tcp偏移距离
        double threshold = 1e-6, a1, a0, M = 1, B = 1, KK = 10, force_aim = -1;
        double pm_begin[16];
        double pm_real[16];
        double R01[16];
        double target_displacement = 0;
        double s_temp = 0, ss = 0;
//    const double M=1,B=1,KK=10,force_aim=-1;
//    double a1, a0;

    };
    struct MoveJoint::Imp : public MoveJointParam {
    };

    auto MoveJoint::prepareNrt() -> void {
        std::cout << "prepare begin" << std::endl;

        enable_mvJoint.store(true);
//        imp_->tool = &*model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(
//                std::string(cmdParams().at("tool")));
//        imp_->wobj = &*model()->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(
//                std::string(cmdParams().at("wobj")));

        imp_->tool = &*model()->generalMotionPool()[0].makI()->fatherPart().findMarker(cmdParams().at("tool"));
        imp_->tool = &*model()->generalMotionPool()[0].makJ()->fatherPart().findMarker(cmdParams().at("wobj"));

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

        for (auto &option : motorOptions()) option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                                                      NOT_CHECK_VEL_CONTINUOUS;
        std::vector <std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::cout << "prepare finished" << std::endl;
    }

    auto MoveJoint::executeRT() -> int {
        const int FS_NUM = 6;

        static double begin_pe[6];
        static double pe[6];

        // end-effector //
        auto &ee = model()->generalMotionPool()[0];

        // Function Pointer
        auto &cout = controller()->mout();
        auto &lout = controller()->lout();
        char eu_type[4]{'1', '2', '3', '\0'};

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double p_next[6], v_now[6], v_tcp[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, v_joint[6];
        static double s_next[6], v_tmp[6], s_now[6], s_tcp[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        static float realdata[6];

        // use lambda function to realize sub-function //
        auto get_force_data = [&](float *data) {
            //auto slave7 = dynamic_cast<aris::control::EthercatSlave &>(controller()->slavePool().at(FS_NUM));
            for (int i = 0; i < 6; i++) {
            //    slave7.readPdo(0x6020, i + 11, data + i, 32);
                 this->ecController()->slavePool()[FS_NUM].readPdo(0x6020, i + 11, data + i, 32);
            }
        };

        if (count() == 1) {
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
        model()->generalMotionPool().at(0).updMpm();
        imp_->tool->getPm(*imp_->wobj, pm_begin);
        s_pm_dot_pm(pm_begin, imp_->fs2tpm, imp_->t2bpm);
        s_mm(3, 1, 3, imp_->t2bpm, aris::dynamic::RowMajor{4}, xyz_temp, 1, imp_->force_target, 1);
        s_mm(3, 1, 3, imp_->t2bpm, aris::dynamic::RowMajor{4}, abc_temp, 1, imp_->force_target + 3, 1);

        if (count() % 500 == 0) {
            for (int i = 0; i < 6; i++) {
                std::cout << "f:" << imp_->force_target[i] << " ";
            }
            std::cout << std::endl;
        }



        if (abs(imp_->force_target[2]) > 0.1) {
            /*
            * if判断语句的作用：
            * abs(imp_->force_target[2])>0.1 表示已经与目标物体接触；
            * else表示还没接触的时候；
            * 如果没有接触就以指定速度下降，如果产生接触则执行阻抗控制命令；
            */

            if (count()== 1 | flag == 0) {
                flag=0;
                //求阻尼力
                model()->generalMotionPool()[0].getMve(v_now, eu_type);
                model()->generalMotionPool()[0].getMve(v_tcp, eu_type);

                // 读力
                imp_->a1 = (imp_->force_target[2] - imp_->force_aim - imp_->damping[2] * v_tcp[2] -
                            imp_->K[2] * s_tcp[2]) / 0.4;
                v_tmp[2] = v_tcp[2];
                v_tcp[2] += ((imp_->a1 + imp_->a0) * 1e-3) / 2;
                imp_->a1 = imp_->a0;
                // 计算阻抗控制笛卡尔空间下的目标相对位移s
                // s_tcp[2]指末端z轴方向上的总相对位移
                s_tcp[2] += ((v_tmp[2] + v_tcp[2]) * 1e-3) / 2;

                // 打印并查验计算得出的目标速度与目标位移
                if (count() % 50 == 0) {
                    std::cout << "V:" << v_tcp[2] << std::endl;
                    std::cout << "S:" << s_tcp[2] << std::endl;
                    std::cout << "fce tar:" << imp_->force_target[2] << std::endl;
                }
            }

            // 获取并设置当前关节空间
            for (int i = 0; i < model()->motionPool().size(); ++i) {
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
            }

            // 根据模型与当前关节空间计算笛卡尔坐标系下的坐标
            // forward kinematic //
            model()->solverPool()[1].kinPos();

            // 更新末端位置的位置坐标
            // update ee //
            ee.updMpm();
            // ee.updMpm与getMpe的区别是什么？ee的物理意义是什么？
            // get ee pe
            // 获取当前欧拉角位姿并赋值到begin_pe，方便操作
            ee.getMpe(begin_pe);


            // 初始化新的笛卡尔坐标pe（初始赋值为begin_pe）
            std::copy(begin_pe, begin_pe + 6, pe);
            double step = s_tcp[2];
            mout() << "s_tcp[2]:" << s_tcp[2] << std::endl;

            // Plan Real Time Trajectory
            Curve curve;

            /* 给pe加上规划好的小位移 */
            imp_->ss = curve.getCurve(count()) - imp_->s_temp;
            mout() << "getcurve:" << curve.getCurve(count()) << std::endl;
            mout() << " imp_->s_temp:" <<  imp_->s_temp << std::endl;
            pe[2] -= step * imp_->ss;
            imp_->s_temp = curve.getCurve(count());

            mout() << "pe[2]" << pe[2] << std::endl;
            mout() << "s_tcp[2]]" << s_tcp[2] << std::endl;

            if (count() % 500 == 0 & s_tcp[2] >= 0.001) {
                mout() << "pe[2]" << pe[2] << std::endl;
                mout() << "s_tcp[2]]" << s_tcp[2] << std::endl;
                return 0;
            }


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


            {
//            imp_->a1 = (imp_->force_target[2]-imp_->force_aim-imp_->damping[2] * v_tcp[2])/0.4;
//            v_tcp[2] += ((imp_->a1+imp_->a0)*1e-3)/2;
//            imp_->a1 = imp_->a0;
            }
        } else {
            for (int i = 0; i < model()->motionPool().size(); ++i) {
                model()->motionPool()[i].setMp(controller()->motionPool()[i].targetPos());
            }

            // forward kinematic //
            model()->solverPool()[1].kinPos();
            mout() << "1" << pe[2] << std::endl;
            // update ee //
            ee.updMpm();

            // get ee pe //
            ee.getMpe(begin_pe);

            std::copy(begin_pe, begin_pe + 6, pe);
            Curve curve;
            cout << "T" << curve.T_ << std::endl;
            double step = 0.1;
            imp_->ss = curve.getCurve(count()) - imp_->s_temp;
            mout() << "getcurve:" << curve.getCurve(count()) << std::endl;
            mout() << " imp_->s_temp:" <<  imp_->s_temp << std::endl;
            pe[2] -= step * imp_->ss;
            imp_->s_temp = curve.getCurve(count());
            mout() << "3" << pe[2] << std::endl;

        }

        {
//            // 根据力传感器受力和阻尼力计算v_tcp //
//            for (int i = 0; i < 6; i++)
//            {
//                if (std::abs(imp_->force_target[i]) > imp_->threshold)  //ending actual force
//                {
//                    v_tcp[i] += imp_->k[i] * imp_->force_target[i] * 1e-3;
//                }
//                v_tcp[i] = std::min(std::max(v_tcp[i], -imp_->vel_limit[i]), imp_->vel_limit[i]);
//            }
        }

        // 给ee设置新的笛卡尔坐标pe
        // inverse kinematic //
        ee.setMpe(pe);

        // 进行反解，计算关节空间
        model()->solverPool()[0].kinPos();
        mout() << "4" << pe[2] << std::endl;

        // 执行
        for (int i = 0; i < model()->motionPool().size(); ++i) {
//            model()->motionPool()[i].updMp();
//            controller()->motionPool()[i].setTargetPos(model()->motionPool()[i].mp());
        }

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
        if (model()->solverPool().at(1).kinPos())return -1;

        static aris::Size total_count = 1;
        if (enable_mvJoint.load()) {
            total_count = count() + 1000;
            return 1;
        } else {
            return total_count - count();
        }
    }

    auto MoveJoint::collectNrt() -> void {}

    MoveJoint::~MoveJoint() = default;

    MoveJoint::MoveJoint(const MoveJoint &other) = default;

    MoveJoint::MoveJoint(MoveJoint &other) = default;

    MoveJoint &MoveJoint::operator=(const MoveJoint &other) = default;

    MoveJoint &MoveJoint::operator=(MoveJoint &&other) = default;

    MoveJoint::MoveJoint(const std::string &name) : Plan(name), imp_(new Imp) {
        //command().loadXmlStr(
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



//------------------------//

    ARIS_REGISTRATION
            {
                aris::core::class_<MoveS>("MoveS")
                        .inherit<Plan>();

                aris::core::class_<MoveTest>("MoveTest")
                        .inherit<Plan>();


            }
    auto createPlanRoot() -> std::unique_ptr <aris::plan::PlanRoot> {
        std::unique_ptr <aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
        //用户自己开发指令集
        plan_root->planPool().add<robot::MoveS>();
        plan_root->planPool().add<robot::MoveTest>();
        plan_root->planPool().add<robot::MoveJoint>();


        //aris库提供指令集
        plan_root->planPool().add<aris::plan::Enable>();
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
