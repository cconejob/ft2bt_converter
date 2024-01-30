import xml.etree.ElementTree as ET
from pathlib import Path


class CodeGenerator:
    def __init__(self):
        self.generate_generic_code()
    
    def generate_generic_code(self):
        """
        Generate the generic code for the actions and conditions header and source files.
        """
        # Generate the actions header file template
        self.actions_header_file_template = \
"""#ifndef ACTIONS_HPP
#define ACTIONS_HPP

class Actions{
public:
"""
        # Generate the conditions header file template
        self.conditions_header_file_template = \
"""#ifndef CONDITIONS_HPP
#define CONDITIONS_HPP

class Conditions{
public:
"""
        # Generate the actions source file template
        self.actions_source_file_template = \
"""#include <ros/ros.h>
#include <safety/actions.hpp>

"""
        # Generate the conditions source file template
        self.conditions_source_file_template = \
"""#include <ros/ros.h>
#include <safety/conditions.hpp>

"""
        
    def parse_behavior_tree_xml(self, xml_file_path):
        """
        Parse the behavior tree xml file and extract the behavior tree ID, actions, and conditions.
        
        Args:
            xml_file_path (str): Path to the behavior tree xml file
        
        Returns:
            actions (set): Set of actions
            conditions (set): Set of conditions
        """
        # Parse the XML file
        tree = ET.parse(xml_file_path)
        root = tree.getroot()

        # Extract Actions and Conditions
        actions = [node.get('ID') for node in root.findall(".//Action")]
        conditions = [node.get('ID') for node in root.findall(".//Condition")]
        descriptions_act = [node.get('name') for node in root.findall(".//Action")]
        descriptions_cond = [node.get('name') for node in root.findall(".//Condition")]
        
        # Remove None elements from the lists
        actions = [element for element in actions if element is not None]
        conditions = [element for element in conditions if element is not None]
        descriptions_act = [element for element in descriptions_act if element is not None]
        descriptions_cond = [element for element in descriptions_cond if element is not None]

        return actions, conditions, descriptions_act, descriptions_cond

    def generate_main_cpp_file(self, xml_file_path, bt_name):
        """
        Generate the main.cpp file from the behavior tree xml file. This file is used to run the behavior tree in ROS.
        
        Args:
            xml_file_path (str): Path to the behavior tree xml file
            bt_name (str): Name of the behavior tree
        """
        actions, conditions, _, _ = self.parse_behavior_tree_xml(xml_file_path)
        
        # Generate the main.cpp file template
        main_cpp_template = \
f"""#include <ros/package.h>
#include <cstdlib>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

int main(int argc, char** argv)
{{
    ros::init(argc, argv, "safety_{bt_name}");
    
    ros::NodeHandle node;
    
    // Register ROS subscribers and publishers

    BT::BehaviorTreeFactory factory;
"""
        # Add the action registrations
        for action in actions:
            main_cpp_template += \
f"""    factory.registerNodeType<Actions>("{action}");\n"""
        # Add the condition registrations
        for condition in conditions:
            main_cpp_template += \
f"""    factory.registerNodeType<Conditions>("{condition}");\n"""

        # Complete the template
        main_cpp_template += \
f"""    std::string bt_path = ros::package::getPath("safety") + "/trees/behavior_trees/" + "BT_{bt_name}.xml";
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
        xml_file_path = Path(xml_file_path)
        src_folder_name = xml_file_path.parent.parent / 'src'
        main_cpp_template_file_path = src_folder_name / f'main_{bt_name}.cpp'
        
        if not main_cpp_template_file_path.exists():
            src_folder_name.mkdir(parents=True, exist_ok=True)
        
        with open(main_cpp_template_file_path, 'w') as file:
            file.write(main_cpp_template)
            
        self.generate_libraries(xml_file_path)
        
    def generate_libraries(self, xml_file_path):
        """
        Generate the header files from the behavior tree xml file. These files are used to run the behavior tree in ROS.
        
        Args:
            xml_file_path (str): Path to the behavior tree xml file
        """
        actions, conditions, descriptions_act, descriptions_cond = self.parse_behavior_tree_xml(xml_file_path)
        
        if not actions and not conditions:
            return

        if actions:
            for idx in range(len(actions)):
                self.actions_header_file_template += \
f"""    void {actions[idx]}();\n"""
                
                self.actions_source_file_template += \
f"""void Actions::{actions[idx]}(){{
    // TODO: Implement action '{actions[idx]}': {descriptions_act[idx]}
    std::cout << "{actions[idx]} is executed." << std::endl;
}}\n
"""
        if conditions:
            for idx in range(len(conditions)):
                self.conditions_header_file_template += \
f"""    bool {conditions[idx]}();\n"""
                
                self.conditions_source_file_template += \
f"""bool Conditions::{conditions[idx]}(){{
    // TODO: Implement condition '{conditions[idx]}':  {descriptions_cond[idx]}
    std::cout << "{conditions[idx]} is checked." << std::endl;
    return true;
}}\n
"""

    def save_in_file(self):
        """
        Save the content in the file.
        """
        file_path = Path(__file__)
        root_folder_name = file_path.parent.parent
        header_folder_name = root_folder_name / 'include' / 'safety'
        src_folder_name = root_folder_name / 'src' / 'common'
        
        actions_header_file_path = header_folder_name / 'actions.hpp'
        actions_source_file_path = src_folder_name / 'actions.cpp'
        conditions_header_file_path = header_folder_name / 'conditions.hpp'
        conditions_source_file_path = src_folder_name / 'conditions.cpp'
        
        self.conditions_header_file_template += \
"""};
#endif"""
        self.actions_header_file_template += \
"""};
#endif"""
        # Save the actions header file template
        if not actions_header_file_path.exists():
            header_folder_name.mkdir(parents=True, exist_ok=True)
        with open(actions_header_file_path, 'w') as file:
            file.write(self.actions_header_file_template)
        
        # Save the actions source file template
        if not actions_source_file_path.exists():
            src_folder_name.mkdir(parents=True, exist_ok=True)
        with open(actions_source_file_path, 'w') as file:
            file.write(self.actions_source_file_template)
        
        # Save the conditions header file template
        if not conditions_header_file_path.exists():
            header_folder_name.mkdir(parents=True, exist_ok=True)
        with open(conditions_header_file_path, 'w') as file:
            file.write(self.conditions_header_file_template)
        
        # Save the conditions source file template
        if not conditions_source_file_path.exists():
            src_folder_name.mkdir(parents=True, exist_ok=True)
        with open(conditions_source_file_path, 'w') as file:
            file.write(self.conditions_source_file_template)