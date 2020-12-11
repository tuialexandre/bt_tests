#include "dummy_nodes.h"

BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckTemperature::tick()
{
    Optional<double> msg = getInput<double>("threshold");
    double  fake_temperature_reading = 50.0;

    if(msg) {
        if (fake_temperature_reading < msg.value())
        {
            std::cout << "[ Temperature: OK ]" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }else {
            std::cerr << "[ Temperature: Critical! ]" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
    else{
        std::cerr << "missing parameter [threshold_distance]" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ThinkWhatToSay::tick()
{
    // the output may change at each tick(). Here we keep it simple.
    setOutput("text", "The answer is 42" );
    return NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomething::tick()
{
    Optional<std::string> msg = getInput<std::string>("message");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", 
                                msg.error() );
    }

    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

void DummyNodes::RegisterNodes(BehaviorTreeFactory& factory)
{
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<CheckTemperature>("TemperatureOK");
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
}