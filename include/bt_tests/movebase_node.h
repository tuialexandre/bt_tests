#ifndef MOVEBASE_BT_NODE_H
#define MOVEBASE_BT_NODE_H

#include "behaviortree_cpp_v3/bt_factory.h"

// Custom type example
struct Pose2D { double x, y, theta; };

inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

namespace BT
{
// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Position2D.
template <> inline Pose2D convertFromString(StringView str)
{
    //printf("Converting string: \"%s\"\n", str.data() );

    // real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 3)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        Pose2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
}
}

using namespace BT;

// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".
class MoveBaseAction : public BT::AsyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Pose2D>("goal") };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};

#endif