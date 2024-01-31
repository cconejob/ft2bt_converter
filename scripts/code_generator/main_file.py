class MainFile:
    def __init__(self):
        """
        Main file class. It is used to generate the main file for the C++ code.
        """
        pass
    
    def common_includes(self):
        """
        Generate the common includes for the main file.
        
        Returns:
            str: Common includes for the main file.
        """
        return \
f"""#include <ros/ros.h>
#include <ros/package.h>
#include <cstdlib>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
"""

    def specific_includes(self, name, type_header='condition'):
        """
        Generate the specific includes for the main file.
        
        Args:
            name (str): Name of the node in the behavior tree.
            type_header (str, optional): Type of the header file. Defaults to 'condition'. Options: 'condition', 'action'.
        
        Returns:
            str: Specific includes for the main file.
        """
        if 'cond' in type_header:
            include_name = 'conditions'
        elif 'act' in type_header:
            include_name = 'actions'
        return \
f"""#include <{include_name}/{name.lower()}.hpp>\n"""

    def main_function(self, bt_name):
        """
        Generate the main function for the main file. 

        Args:
            bt_name (str): Name of the behavior tree.
        """
        return \
f"""
int main(int argc, char** argv)
{{
    ros::init(argc, argv, "safety_{bt_name}");
    
    ros::NodeHandle node;
    
    // Register ROS subscribers and publishers

    BT::BehaviorTreeFactory factory;
"""

    def register_node(self, name, description):
        """
        Generate the node registration for the main file. 

        Args:
            name (str): Name of the node in the behavior tree.
            description (str): Description of the node in the behavior tree.
            
        Returns:
            str: Node registration for the main file.
        """
        return \
f"""    
    // {description}
    factory.registerNodeType<{name}>("{name}"); \n"""
    
    def end_main_function(self, bt_name):
        """
        Generate the end of the main function for the main file. 
        
        Args:
            bt_name (str): Name of the behavior tree.

        Returns:
            str: End of the main function for the main file.
        """
        return \
f"""    
    std::string bt_path = ros::package::getPath("safety") + "/trees/behavior_trees/" + "BT_{bt_name}.xml";
    ROS_INFO("Loading behavior tree from file: %s", bt_path.c_str());

    BT::Tree tree = factory.createTreeFromFile(bt_path);
    BT::PublisherZMQ publisher_zmq(tree);
    ros::Rate loop_rate(50);

    while (ros::ok())
    {{
        tree.tickRoot();
        ros::spinOnce();
        loop_rate.sleep();
    }}

    return 0;
}}"""