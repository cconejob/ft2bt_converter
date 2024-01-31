class HeaderFile:
    def __init__(self, type_header='condition'):
        """
        Header file class. It is used to generate the header file for the C++ code.

        Args:
            type_header (str, optional): Type of the header file. Defaults to 'condition'. Options: 'condition', 'action'.
        """
        self.type_header = type_header

    def common_includes(self):
        """
        Generate the common includes for the header file. These includes are common to both condition and action nodes.
        
        Returns:
            str: Common includes for the header file.
        """
        if 'cond' in self.type_header:
            node_name = 'condition_node'
        elif 'act' in self.type_header:
            node_name = 'action_node'
        return \
f"""#include <behaviortree_cpp_v3/{node_name}.h>
#include <ros/ros.h>

"""

    def name_hpp(self, name):
        """
        Generate the name of the header file. The name is the name of the node in the behavior tree.
        
        Args:
            name (str): Name of the node in the behavior tree.
        
        Returns:
            str: Name of the header file.
        """
        return \
f"""#ifndef {name.upper()}_HPP
#define {name.upper()}_HPP

"""

    def class_hpp(self, name):
        """
        Generate the class for the header file. The class is the name of the node in the behavior tree.
        
        Args:
            name (str): Name of the node in the behavior tree.
        
        Returns:
            str: Class for the header file.
        """
        if 'cond' in self.type_header:
            node_name = 'ConditionNode'
        elif 'act' in self.type_header:
            node_name = 'AsyncActionNode'
        return \
f"""
class {name} : public BT::{node_name}
{{
public:

    {name}(const std::string& name, const BT::NodeConfiguration& config);
"""

    def end_hpp(self):
        """
        Generate the end of the header file.
        
        Returns:
            str: End of the header file.
        """
        end_hpp = \
"""
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();
"""
        if 'act' in self.type_header:
            end_hpp += \
"""
private:

    ros::NodeHandle private_nh_;
"""
        end_hpp += \
"""};
#endif
"""
        return end_hpp