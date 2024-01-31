import xml.etree.ElementTree as ET
from pathlib import Path

from code_generator.header_file import HeaderFile
from code_generator.source_file import SourceFile
from code_generator.main_file import MainFile


class CodeGenerator:
    def __init__(self):
        self.actions_header = HeaderFile(type_header='actions')
        self.conditions_header = HeaderFile(type_header='conditions')
        self.actions_source = SourceFile(type_source='actions')
        self.conditions_source = SourceFile(type_source='conditions')
        self.main_cpp_file = MainFile()
        self.generate_generic_code()
    
    def generate_generic_code(self):
        """
        Generate the generic code for the actions and conditions header and source files.
        """
        self.actions_header_file_template = self.actions_header.common_includes()
        self.conditions_header_file_template = self.conditions_header.common_includes()
        self.actions_source_file_template = self.actions_source.common_includes()
        self.conditions_source_file_template = self.conditions_source.common_includes()
        
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
        actions, conditions, descriptions_act, descriptions_cond = self.parse_behavior_tree_xml(xml_file_path)
        
        # Generate the main.cpp file template
        main_cpp_template = self.main_cpp_file.common_includes()
        
        # Add the action includes
        for idx in range(len(actions)):
            main_cpp_template += self.main_cpp_file.specific_includes(actions[idx], type_header='action')
        
        # Add the condition includes
        for idx in range(len(conditions)):
            main_cpp_template += self.main_cpp_file.specific_includes(conditions[idx], type_header='condition')

        main_cpp_template += self.main_cpp_file.main_function(bt_name)
        
        # Add the action registrations
        for idx in range(len(actions)):
            main_cpp_template += self.main_cpp_file.register_node(actions[idx], descriptions_act[idx])
        
        # Add the condition registrations
        for idx in range(len(conditions)):
            main_cpp_template += self.main_cpp_file.register_node(conditions[idx], descriptions_cond[idx])

        # Complete the template
        main_cpp_template += self.main_cpp_file.end_main_function(bt_name)
        
        xml_file_path = Path(xml_file_path)
        src_folder_name = xml_file_path.parent.parent / 'src'
        main_cpp_template_file_path = src_folder_name / f'main_{bt_name}.cpp'
        
        if not main_cpp_template_file_path.exists():
            src_folder_name.mkdir(parents=True, exist_ok=True)
        
        with open(main_cpp_template_file_path, 'w') as file:
            file.write(main_cpp_template)
            
        self.generate_libraries(xml_file_path, bt_name)
        
    def generate_libraries(self, xml_file_path, bt_name):
        """
        Generate the header and source files from the behavior tree xml file. These files are used to run the behavior tree in ROS.
        
        Args:
            xml_file_path (str): Path to the behavior tree xml file
            bt_name (str): Name of the behavior tree
        """
        actions, conditions, descriptions_act, descriptions_cond = self.parse_behavior_tree_xml(xml_file_path)
        
        if not actions and not conditions:
            return

        if actions:
            for idx in range(len(actions)):
                actions_header_file_template = self.actions_header.name_hpp(actions[idx])
                actions_header_file_template += self.actions_header_file_template
                actions_header_file_template += self.actions_header.class_hpp(actions[idx])
                actions_header_file_template += self.actions_header.end_hpp()
                
                actions_source_file_template = self.actions_source_file_template
                actions_source_file_template += self.actions_source.node_implementation(actions[idx], bt_name, descriptions_act[idx])

                self.save_in_file(actions_header_file_template, actions_source_file_template, actions[idx], type='action')
                
        if conditions:
            for idx in range(len(conditions)):
                conditions_header_file_template = self.conditions_header.name_hpp(conditions[idx])
                conditions_header_file_template += self.conditions_header_file_template
                conditions_header_file_template += self.conditions_header.class_hpp(conditions[idx])
                conditions_header_file_template += self.conditions_header.end_hpp()
                
                conditions_source_file_template = self.conditions_source_file_template
                conditions_source_file_template += self.conditions_source.node_implementation(conditions[idx], bt_name, descriptions_cond[idx])
                
                self.save_in_file(conditions_header_file_template, conditions_source_file_template, conditions[idx], type='condition')

    def save_in_file(self, header_file_template, source_file_template, name, type='condition'):
        """
        Save the content in the file. The file is saved in the include and src folders.
        
        Args:
            header_file_template (str): Template for the header file
            source_file_template (str): Template for the source file
            name (str): Name of the behavior tree condition or action
            type (str): Type of the file to save. Can be 'action' or 'condition'. Default is 'condition'
        """
        file_path = Path(__file__)
        root_folder_name = file_path.parent.parent.parent
        
        header_folder_name = root_folder_name / 'include' / f'{type}s'
        src_folder_name = root_folder_name / 'src' / f'{type}s'
        header_file_path = header_folder_name / f'{name.lower()}.hpp'
        source_file_path = src_folder_name / f'{name.lower()}.cpp'

        # Save the conditions header file template
        if not header_file_path.exists():
            header_folder_name.mkdir(parents=True, exist_ok=True)
        with open(header_file_path, 'w') as file:
            file.write(header_file_template)
        
        # Save the actions source file template
        if not source_file_path.exists():
            src_folder_name.mkdir(parents=True, exist_ok=True)
        with open(source_file_path, 'w') as file:
            file.write(source_file_template)