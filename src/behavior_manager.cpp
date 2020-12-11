#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include "dummy_nodes.h"
#include "movebase_node.h"

//#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

class BehaviorManager {
    const std::string pkg_name = "bt_tests";
    BT::BehaviorTreeFactory factory{};
    BT::Tree tree_sequence{};
    BT::Tree tree_reactive{};
    
    void RegisterActionNodes() {
        DummyNodes::RegisterNodes(factory);
        MoveBaseNodes::RegisterNodes(factory);
    }

    void LoadTree() {
        std::string tree_path = ros::package::getPath(pkg_name);
        tree_path.append(R"(/behavior_tree/test_tree_sequence.xml)");
        std::cout << "Sequence tree file path: "<< tree_path  << std::endl;
        tree_sequence = factory.createTreeFromFile(tree_path);

        tree_path = ros::package::getPath(pkg_name);
        tree_path.append(R"(/behavior_tree/test_tree_reactive.xml)");
        std::cout << "Reactive tree file path: "<< tree_path  << std::endl;
        tree_reactive = factory.createTreeFromFile(tree_path);
    }

    void Assert(bool condition)
    {
        if (!condition)
            throw RuntimeError("this is not what I expected");
    }

    std::string get_log_path(std::string file_name){
        std::string log_path = ros::package::getPath(pkg_name);
        log_path.append(R"(/log/)");
        log_path.append(file_name);
        log_path.append(".fbl");
        return log_path;
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
        FileLogger simple_tree_logger(tree_sequence, get_log_path("simple_sequence").c_str());

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
        // This logger saves state changes on file
        FileLogger reactive_tree_logger(tree_reactive, get_log_path("reactive_sequence").c_str());

        // This logger publish status changes using ZeroMQ. Used by Groot
        // PublisherZMQ publisher_zmq(tree_reactive);

        // This logger prints state changes on console
        //StdCoutLogger logger_cout(tree_reactive);

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