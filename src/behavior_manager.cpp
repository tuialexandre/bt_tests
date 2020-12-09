#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>

#include "dummy_nodes.h"

class BehaviorManager {
    BT::BehaviorTreeFactory factory{};
    BT::Tree tree{};
    
    void RegisterActionNodes() {
        factory.registerNodeType<SaySomething>("SaySomething");
        factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
    }

    void LoadTree() {
        std::string tree_path = ros::package::getPath("bt_tests");
        tree_path.append(R"(/behavior_tree/test_tree.xml)");
        std::cout << "Tree file path: "<< tree_path  << std::endl;
        tree = factory.createTreeFromFile(tree_path);
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
        tree.tickRoot();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "behavior_manager");
    ros::NodeHandle nh;
    BehaviorManager bt_manager{};
    bt_manager.Run();
    ros::spin();
    return 0;
}