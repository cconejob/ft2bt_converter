import argparse
from pathlib import Path

from ft2bt.scripts.fault_trees.xml_fta_parser import XMLFTAParser
from ft2bt.scripts.behavior_trees.behavior_tree import BehaviorTree
from ft2bt.scripts.code_generator.code_generator import CodeGenerator


def main():  
    parser = argparse.ArgumentParser(description='Convert xml file from drawio to behavior tree xml file for Groot.')
    
    # Limit coordinates
    parser.add_argument('-f', '--fta_filepath', type=str, help="*.xml fault tree global path", required=True)
    parser.add_argument('-v', '--view', action='store_true', help="View the behavior tree renders?")
    parser.add_argument('-c', '--generate_cpp', action='store_true', help="Generate C++ code template?")
    parser.add_argument('-r', '--replace', action='store_true', help="Replace existing files?")
    args = parser.parse_args()
    
    # Get the path to the package
    module_path = Path(__file__).resolve()
    package_path = module_path.parent.parent.parent
    
    # Add the .xml extension if it is not present
    if not args.fta_filepath.endswith('.xml'):
        args.fta_filepath += '.xml'
        print('.xml extension not found in the file path.')
        print(f'Added .xml extension to the file path: {args.fta_filepath}')
    
    # Generate the fault tree diagram from the XML file
    fta_filename = Path(args.fta_filepath).stem
    fta_parser = XMLFTAParser(xml_file=args.fta_filepath)
    fta_list = fta_parser.generate_fault_trees(plot=args.view)

    # Generate the behavior tree diagram from every fault tree diagram
    behavior_tree_folder = package_path / 'behavior_trees'
    prev_bt = BehaviorTree(name='prev')
    code_generator = CodeGenerator(replace=args.replace, filename=fta_filename.lower())
    
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