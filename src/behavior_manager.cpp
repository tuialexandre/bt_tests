#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>

#include "dummy_nodes.h"
#include "movebase_node.h"

class BehaviorManager {
    BT::BehaviorTreeFactory factory{};
    BT::Tree tree_sequence{};
    BT::Tree tree_reactive{};
    
    void RegisterActionNodes() {
        factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
        factory.registerNodeType<SaySomething>("SaySomething");
        factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
        factory.registerNodeType<MoveBaseAction>("MoveBase");
    }

    void LoadTree() {
        std::string tree_path = ros::package::getPath("bt_tests");
        tree_path.append(R"(/behavior_tree/test_tree_sequence.xml)");
        std::cout << "Sequence tree file path: "<< tree_path  << std::endl;
        tree_sequence = factory.createTreeFromFile(tree_path);

        tree_path = ros::package::getPath("bt_tests");
        tree_path.append(R"(/behavior_tree/test_tree_reactive.xml)");
        std::cout << "Reactive tree file path: "<< tree_path  << std::endl;
        tree_reactive = factory.createTreeFromFile(tree_path);
    }

    void Assert(bool condition)
    {
        if (!condition)
            throw RuntimeError("this is not what I expected");
    }

public:
    BehaviorManager() {
        std::cout << "Initializing BT... " << std::endl;
        try {
            RegisterActionNodes();
            LoadTree();
            std::cout << "BT is ready" << std::endl;
        } catch(std::exception& e) {
            std::cout << "Failed to setup BT: "<< e.what() << std::endl;
        }
    }
    ~BehaviorManager() {}
    
    
    void Run() {
        NodeStatus status;

        std::cout << "Running simple sequence tree: " << std::endl;

        std::cout << "\n--- 1st executeTick() ---" << std::endl;
        status = tree_sequence.tickRoot();
        Assert(status == NodeStatus::RUNNING);

        SleepMS(150);
        std::cout << "\n--- 2nd executeTick() ---" << std::endl;
        status = tree_sequence.tickRoot();
        Assert(status == NodeStatus::RUNNING);

        SleepMS(150);
        std::cout << "\n--- 3rd executeTick() ---" << std::endl;
        status = tree_sequence.tickRoot();
        Assert(status == NodeStatus::SUCCESS);
        std::cout << std::endl;

        std::cout << "Running reactive sequence tree: " << std::endl;

        std::cout << "\n--- 1st executeTick() ---" << std::endl;
        status = tree_reactive.tickRoot();
        Assert(status == NodeStatus::RUNNING);

        SleepMS(150);
        std::cout << "\n--- 2nd executeTick() ---" << std::endl;
        status = tree_reactive.tickRoot();
        Assert(status == NodeStatus::RUNNING);

        SleepMS(150);
        std::cout << "\n--- 3rd executeTick() ---" << std::endl;
        status = tree_reactive.tickRoot();
        Assert(status == NodeStatus::SUCCESS);
        std::cout << std::endl;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "behavior_tester");
    ros::NodeHandle nh;
    BehaviorManager bt_tester{};
    bt_tester.Run();
    ros::spin();
    return 0;
}