import xml.etree.ElementTree as ET
from pathlib import Path

from .header_file import HeaderFile
from .source_file import SourceFile
from .main_file import MainFile


class CodeGenerator:
    def __init__(self, replace=False, filename='fault_tree'):
        """
        Generate the header and source files for the actions and conditions.

        Args:
            replace (bool, optional): Replace existing files? Defaults to False.
            filename (str, optional): Name of the fault tree xml file. Defaults to 'fault_tree.xml'.
        """
        self.replace = replace
        self.filename = filename
        self.actions_header = HeaderFile(type_header='actions')
        self.conditions_header = HeaderFile(type_header='conditions')
        self.actions_source = SourceFile(filename=self.filename, type_source='actions')
        self.conditions_source = SourceFile(filename=self.filename, type_source='conditions')
        self.main_cpp_file = MainFile()
        self.generate_generic_code()

        file_path = Path(__file__)
        root_folder_name = file_path.parent.parent.parent
        self.header_folder_name = root_folder_name / 'include' / self.filename
        self.src_folder_name = root_folder_name / 'src' / self.filename
    
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
        src_folder_name = xml_file_path.parent.parent / 'src' / self.filename
        main_cpp_template_file_path = src_folder_name / f'main_{bt_name}.cpp'
        
        if not self.file_exists(main_cpp_template_file_path):
            self.save_in_path(main_cpp_template, main_cpp_template_file_path)
            
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
                
    def file_exists(self, file_path):
        """
        Check if the a file exists.

        Args:
            file_path (str): Path to the file
        """
        exists = file_path.exists()
        name = file_path.name.split('/')[-1]

        if exists and not self.replace:
            print(f'File {name} already exists. Skipping...')
            return True

        else:
            print(f'Saving {name} file...')
            return False
        
    def save_in_path(self, file_template, file_path):
        """
        Save the content in the file. The file is saved in the include and src folders if replace is True or if the file does not exist.
        
        Args:
            file_template (str): Template for the file
            file_path (str): Path to the file
        """
        folder_name = file_path.parent          
        if not file_path.exists():
            folder_name.mkdir(parents=True, exist_ok=True)
        with open(file_path, 'w') as file:
            file.write(file_template)

    def save_in_file(self, header_file_template, source_file_template, name, type='condition'):
        """
        Save the content in the file. The file is saved in the include and src folders if replace is True or if the file does not exist.
        
        Args:
            header_file_template (str): Template for the header file
            source_file_template (str): Template for the source file
            name (str): Name of the behavior tree condition or action
            type (str): Type of the file to save. Can be 'action' or 'condition'. Default is 'condition'
        """
        header_file_path = self.header_folder_name / f'{type}s' / f'{name.lower()}.hpp'
        source_file_path = self.src_folder_name / f'{type}s' / f'{name.lower()}.cpp'
        
        if not self.file_exists(header_file_path):
            self.save_in_path(header_file_template, header_file_path)
            
        if not self.file_exists(source_file_path):
            self.save_in_path(source_file_template, source_file_path)