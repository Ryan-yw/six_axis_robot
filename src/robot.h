#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{
    //其他参数和函数声明
    using Size = std::size_t;
    constexpr double PI = 3.141592653589793;
    class MoveS :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MoveS();
        explicit MoveS(const std::string &name = "mvs");
  //      ARIS_REGISTER_TYPE(MoveS);
    };

    class MoveJoint : public aris::plan::Plan
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

    class MoveTest :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        explicit MoveTest(const std::string &name = "mv_test");
    //    ARIS_REGISTER_TYPE(MoveTest);

        double dir_{1};
    };

//    class Serial : public aris::plan::Plan
    class Serial : public aris::core::CloneObject<Serial, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void override;
        auto virtual executeRT()->int override;

        virtual ~Serial();
        explicit Serial(const std::string &name = "sucker_ctrl");
  //      ARIS_REGISTER_TYPE(Serial);

//        char motor{"Off"};
//        char airSwitch{"Off"};
    };

    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
