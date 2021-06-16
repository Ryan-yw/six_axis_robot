#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{
    //其他参数和函数声明
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

    class MoveTest :public aris::core::CloneObject<MoveTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;
        virtual ~MoveTest();
        explicit MoveTest(const std::string &name = "mv_test");
    //    ARIS_REGISTER_TYPE(MoveTest);
        MoveTest(const MoveTest &);
        MoveTest(MoveTest &);
        MoveTest& operator=(const MoveTest &);
        MoveTest& operator=(MoveTest &&);
        double dir_{1};
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

//

    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
