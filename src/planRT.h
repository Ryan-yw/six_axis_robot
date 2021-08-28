//
// Created by Capoo on 2021/2/6.
//

#ifndef SIXAXISROBOT_PLANRT_H
#define SIXAXISROBOT_PLANRT_H

///功能：生成实时曲线。可根据输入的时间与位移判断曲线为梯形还是三角形
//   ##参数定义##
//  T: Period Time of Force Measurement 计算阻尼力或重新规划一条曲线的周期时间，由用户输入 单位ms
//  tr: Rising Time 加速段所需的时间，由速度上限v和加速度上限a计算v/a得到 单位ms
//  T: Period Time of Force Measurement 力传感器读取并计算阻尼力或重新规划一条曲线的周期时间，由用户输入 单位ms
//  v: Velocity Limit of the Manipulator End Effector 机械臂末端固有或用户预设绝对速度上限 单位m/s
//  a: Velocity Limit of the Manipulator End Effector 机械臂末端固有或用户预设绝对加速度上限 单位m/s^2
//  FLAG: 0 for T-curve; T形曲线
//        1 for Triangle-curve; 三角形曲线
//        FLAG决定生成曲线的类型
static int member_count;
static int flag;

class Curve {
public:
    double tr_;
    double T_;
    double v_;
    double a_;
    int FLAG_;


    double curve_[];

    auto getCurve(int count) -> double;

    Curve( double v = 2, double a = 5)
    {
        v_ = v;
        a_ = a;
        tr_ = v / a;
        T_ = (a_ + v_ * v_) / v_ / a_;
    }

    ~Curve() {}
};

class Speed {
public:
    double acc_;
    double v_rch_;
    double acc_time;

    double speed_[];

    auto getVnow(int count) -> double;

    Speed( double v = 0.4, double a =0.1){
        v_rch_ = v;
        acc_ = a;
        acc_time = v / a;
    }

    ~Speed() {}
};


#endif //SIXAXISROBOT_PLANRT_H
