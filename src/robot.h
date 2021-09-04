#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{
    //其他参数和函数声明

    void moveTo(double* , double *);
    using Size = std::size_t;
    constexpr double PI = 3.141592653589793;
    class MoveS :public aris::core::CloneObject<MoveS, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MoveS();
        explicit MoveS(const std::string &name = "mvs");
  //      ARIS_REGISTER_TYPE(MoveS);
    };

    class MoveJoint : public aris::core::CloneObject<MoveJoint, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~MoveJoint();
        explicit MoveJoint(const std::string &name = "MoveJoint_plan");
  //      ARIS_REGISTER_TYPE(robot::MoveJoint);
        MoveJoint(const MoveJoint &);
        MoveJoint(MoveJoint &);
        MoveJoint& operator=(const MoveJoint &);
        MoveJoint& operator=(MoveJoint &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

//    class MoveTest :public aris::core::CloneObject<MoveTest, aris::plan::Plan>
//    {
//    public:
//        auto virtual prepareNrt()->void;
//        auto virtual executeRT()->int;
//        auto virtual collectNrt()->void;
//        virtual ~MoveTest();
//        explicit MoveTest(const std::string &name = "mv_test");
//    //    ARIS_REGISTER_TYPE(MoveTest);
//        MoveTest(const MoveTest &);
//        MoveTest(MoveTest &);
//        MoveTest& operator=(const MoveTest &);
//        MoveTest& operator=(MoveTest &&);
//        double dir_{1};
//    };
    class MoveTest :public aris::core::CloneObject<MoveTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        explicit MoveTest(const std::string &name = "mv_test");
    //    ARIS_REGISTER_TYPE(MoveTest);

        double dir_;
    };

    class ImpedPos : public aris::core::CloneObject<ImpedPos, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~ImpedPos();
        explicit ImpedPos(const std::string &name = "Impedence control");
        ImpedPos(const ImpedPos &);
        ImpedPos(ImpedPos &);
        ImpedPos& operator=(const ImpedPos &);
        ImpedPos& operator=(ImpedPos &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };
    /**
     * @brief The Drag class. If a force is applied on the end effector, the end-effctor will both translational and rotational move in world coordinate by admittance control.
     * @note The admittance control law and the rodrigous functions haven't verified correct.
     */
    class Drag : public aris::core::CloneObject<Drag, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~Drag();
        explicit Drag(const std::string &name = "Drag");
        Drag(const Drag &);
        Drag(Drag &);
        Drag& operator=(const Drag &);
        Drag& operator=(Drag &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };
    /**
     * @brief The DragTrans class. If a force is applied on the end effector, the end-effctor will translational move in world coordinate by admittance control.
     * @note The admittance control law hasn't verified correct.
     */

    class DragTrans : public aris::core::CloneObject<DragTrans, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~DragTrans();
        explicit DragTrans(const std::string &name = "DragTrans");
        DragTrans(const DragTrans &);
        DragTrans(DragTrans &);
        DragTrans& operator=(const DragTrans &);
        DragTrans& operator=(DragTrans &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };
    /**
     * @brief The DragRot class. If a force is applied on the end effector, the end-effctor will rotational move in world coordinate by admittance control.
     * @note The admittance control law and the rodrigous functions haven't verified correct.
     */


    class DragRot : public aris::core::CloneObject<DragRot, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~DragRot();
        explicit DragRot(const std::string &name = "DragRot");
        DragRot(const DragRot &);
        DragRot(DragRot &);
        DragRot& operator=(const DragRot &);
        DragRot& operator=(DragRot &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

    double moveto(double q0, double qf, double t, double t0);
    double moveto(double q0, double qf, double t, double t0, double alim, double vlim);
    double moveto(const double *q0s,const double *qfs, double t, double t0,const double *alims,const double *vlims, double *result);

    /**
     * @brief The Moveto class  The end effector moves z+0.5 in world coordinate
     */
    class Moveto : public aris::core::CloneObject<Moveto, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~Moveto();
        explicit Moveto(const std::string &name = "Moveto");
        Moveto(const Moveto &);
        Moveto(Moveto &);
        Moveto& operator=(const Moveto &);
        Moveto& operator=(Moveto &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

    /**
     * @brief The Trapezoid class
     */

    class Trapezoid : public aris::core::CloneObject<Trapezoid, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~Trapezoid();
        explicit Trapezoid(const std::string &name = "Trapezoid");
        Trapezoid(const Trapezoid &);
        Trapezoid(Trapezoid &);
        Trapezoid& operator=(const Trapezoid &);
        Trapezoid& operator=(Trapezoid &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

    class FitTog : public aris::core::CloneObject<FitTog, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~FitTog();
        explicit FitTog(const std::string &name = "FitTog");
        FitTog(const FitTog &);
        FitTog(FitTog &);
        FitTog& operator=(const FitTog &);
        FitTog& operator=(FitTog &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

    class Test : public aris::core::CloneObject<Test, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~Test();
        explicit Test(const std::string &name = "Test");
        Test(const Test &);
        Test(Test &);
        Test& operator=(const Test &);
        Test& operator=(Test &&);

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

//

    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
