import argparse
from pathlib import Path

from xml_fta_parser import XMLFTAParser
from behavior_tree import BehaviorTree
from code_generator.code_generator import CodeGenerator


def main():  
    parser = argparse.ArgumentParser(description='Convert xml file from drawio to behavior tree xml file for Groot.')
    
    # Limit coordinates
    parser.add_argument('-f', '--fta_filename', type=str, help="*.xml fault tree name", required=True)
    parser.add_argument('-v', '--view', action='store_true', help="View the behavior tree renders?")
    parser.add_argument('-c', '--generate_cpp', action='store_true', help="Generate C++ code template?")
    parser.add_argument('-r', '--replace', action='store_true', help="Replace existing files?")
    args = parser.parse_args()
    
    # Get the path to the package
    module_path = Path(__file__).resolve()
    package_path = module_path.parent.parent
    
    # Add the .xml extension if it is not present
    if not args.fta_filename.endswith('.xml'):
        args.fta_filename += '.xml'
    
    # Generate the fault tree diagram from the XML file
    fault_tree_xml_file = package_path / 'fault_trees' / args.fta_filename
    fta_parser = XMLFTAParser(xml_file=fault_tree_xml_file)
    fta_list = fta_parser.generate_fault_trees(plot=args.view)

    # Generate the behavior tree diagram from every fault tree diagram
    behavior_tree_folder = package_path / 'behavior_trees'
    prev_bt = BehaviorTree(name='prev')
    code_generator = CodeGenerator(replace=args.replace, filename=args.fta_filename.lower())
    
    for fta in fta_list:
        bt = BehaviorTree(name=fta.name)
        bt.event_number = prev_bt.event_number
        bt.action_number = prev_bt.action_number
        bt.generate_from_fault_tree(fta)
        bt.generate_xml_file(folder_name=behavior_tree_folder, view=args.view)
        
        if args.generate_cpp:
            code_generator.generate_main_cpp_file(xml_file_path=bt.xml_file_path, bt_name=bt.name)
        prev_bt = bt
    

if __name__ == "__main__":    
    main()