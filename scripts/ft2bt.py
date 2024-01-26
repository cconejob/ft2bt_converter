import argparse
from pathlib import Path

from xml_fta_parser import XMLFTAParser
from behavior_tree import BehaviorTree


def main():  
    parser = argparse.ArgumentParser(description='Convert xml file from drawio to behavior tree xml file for Groot.')
    
    # Limit coordinates
    parser.add_argument('-f', '--fta_filename', type=str, help="*.xml fault tree name", required=True)
    parser.add_argument('-r', '--render', action='store_true', help="Show the behavior tree render?")
    parser.add_argument('-v', '--view', action='store_true', help="View the behavior tree renders?")
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
    fta_list = fta_parser.generate_fault_trees()

    # Generate the behavior tree diagram from every fault tree diagram
    behavior_tree_folder = package_path / 'behavior_trees'
    for fta in fta_list:
        bt = BehaviorTree(name=fta.name)
        bt.generate_from_fault_tree(fta)
        bt.generate_xml_file(folder_name=behavior_tree_folder, render=args.render, view=args.view)
    

if __name__ == "__main__":    
    main()