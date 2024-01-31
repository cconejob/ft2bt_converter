class SourceFile:
    def __init__(self, type_source='condition'):
        """
        Source file class. It is used to generate the source file for the C++ code.

        Args:
            type_source (str, optional): Type of the source file. Defaults to 'condition'. Options: 'condition', 'action'.
        """
        self.type_source = type_source
        
    def common_includes(self):
        """
        Generate the common includes for the source file. These includes are common to both condition and action nodes.
        
        Returns:
            str: Common includes for the source file.
        """
        return \
"""#include <ros/ros.h>

"""

    def node_implementation(self, name, bt_name, description):
        """
        Generate the node implementation for the source file. The node implementation is the name of the node in the behavior tree.
        
        Args:
            name (str): Name of the node in the behavior tree.
            bt_name (str): Name of the behavior tree.
            description (str): Description of the node in the behavior tree.
        
        Returns:
            str: Node implementation for the source file.
        """
        if 'cond' in self.type_source:
            include_name = 'conditions'
            node_name = 'ConditionNode'
            extra = ''
        elif 'act' in self.type_source:
            include_name = 'actions'
            node_name = 'AsyncActionNode'
            extra = ', private_nh_("~")'
        return \
f"""#include <{include_name}/{name.lower()}.hpp>

{name}::{name}(const std::string& name, const BT::NodeConfiguration& config)
    : {node_name}(name, config){extra}{{
    ROS_DEBUG("{name} is initialized");
}};

BT::NodeStatus {name}::tick()
{{
    // TODO: Implement action for {bt_name} -> '{name}': {description}
    ROS_DEBUG("{name} is executing");
    return BT::NodeStatus::SUCCESS;
}};

BT::PortsList {name}::providedPorts()
{{
    return {{}};
}};

"""