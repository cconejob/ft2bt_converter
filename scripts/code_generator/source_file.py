class SourceFile:
    def __init__(self, filename, type_source='condition'):
        """
        Source file class. It is used to generate the source file for the C++ code.

        Args:
            type_source (str, optional): Type of the source file. Defaults to 'condition'. Options: 'condition', 'action'.
            filename (str): Name of the entire project.
        """
        self.type_source = type_source
        self.filename = filename
        
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
            return_type = 'BT::NodeStatus::FAILURE'
            
        elif 'act' in self.type_source:
            include_name = 'actions'
            node_name = 'AsyncActionNode'
            extra = ', private_nh_("~")'
            return_type = 'BT::NodeStatus::SUCCESS'
        
        return \
f"""#include <{self.filename}/{include_name}/{name.lower()}.hpp>

{name}::{name}(const std::string& name, const BT::NodeConfiguration& config)
    : {node_name}(name, config){extra}{{
    ROS_DEBUG("{name} is initialized");
}};

BT::NodeStatus {name}::tick()
{{
    {self.comment_tick(name, description, bt_name)}
    ROS_DEBUG("{name} is executing");
    return {return_type};
}};

BT::PortsList {name}::providedPorts()
{{
    return {{}};
}};

"""
    def comment_tick(self, name, description, bt_name):
        """
        Generate the comment for the tick function in the source file. The comment is the description of the node in the behavior tree.
        
        Args:
            name (str): Name of the node in the behavior tree.
            description (str): Description of the node in the behavior tree.
            bt_name (str): Name of the behavior tree.
        
        Returns:
            str: Comment for the source file.
        """
        if 'cond' in self.type_source:
            text = \
f"""// If Event: {description} occurs, then {bt_name} -> '{name}' is SUCCESS
    // return BT::NodeStatus::SUCCESS;
    
    // If Event: {description} does not occur, then {bt_name} -> '{name}' is FAILURE
    // return BT::NodeStatus::FAILURE;
"""
        elif 'act' in self.type_source:
            text = \
f"""// If Action: {description} is running, then {bt_name} -> '{name}' is RUNNING
    // return BT::NodeStatus::RUNNING;
    
    // If Action: {description} has already finished, then {bt_name} -> '{name}' is SUCCESS
    // return BT::NodeStatus::SUCCESS;
"""
        return \
f"""// TODO: Implement action for {bt_name} -> '{name}': {description}

    {text}"""

