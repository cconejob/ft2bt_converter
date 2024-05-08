import argparse
from pathlib import Path

from ft2bt.scripts.fault_trees.xml_fta_parser import XMLFTAParser
from ft2bt.scripts.behavior_trees.behavior_tree import BehaviorTree
from ft2bt.scripts.code_generator.code_generator import CodeGenerator
from ft2bt.scripts.hara.hara_parser import HARAParser


def main():  
    parser = argparse.ArgumentParser(description='Convert xml file from drawio to behavior tree xml file for Groot.')
    
    # Limit coordinates
    parser.add_argument('-f', '--fta_filepath', type=str, help="*.xml fault tree global path", required=True)
    parser.add_argument('-v', '--view', action='store_true', help="View the behavior tree renders?")
    parser.add_argument('-c', '--generate_cpp', action='store_true', help="Generate C++ code template?")
    parser.add_argument('-r', '--replace', action='store_true', help="Replace existing files?")
    parser.add_argument('-o', '--output_folder', type=str, help="Output folder for the behavior trees.")
    parser.add_argument('-p', '--probabilistic', action='store_true', help="Generate probabilistic behavior trees.")
    parser.add_argument('-H', '--HARA_filepath', type=str, help="*.csv HARA file global path.", default=None, required=False)
    args = parser.parse_args()
    
    # Get the path to the package
    module_path = Path(__file__).resolve()
    package_path = module_path.parent.parent.parent
    hara_available = args.HARA_filepath is not None
    
    # Add the .xml extension if it is not present
    if not args.fta_filepath.endswith('.xml'):
        args.fta_filepath += '.xml'
        print('.xml extension not found in the file path.')
        print(f'Added .xml extension to the file path: {args.fta_filepath}')
    
    # Generate the fault tree diagram from the XML file
    fta_filename = Path(args.fta_filepath).stem
    fta_parser = XMLFTAParser(xml_file=args.fta_filepath, probabilistic=args.probabilistic)
    fta_list = fta_parser.generate_fault_trees(plot=args.view)

    # Initialize the behavior tree and code generator objects
    prev_bt = None
    code_generator = CodeGenerator(replace=args.replace, filename=fta_filename.lower())
    
    # Create the folder for the behavior trees
    if args.output_folder:
        behavior_tree_folder = args.output_folder
    else:
        behavior_tree_folder = package_path / 'behavior_trees'
        
    # Read the HARA CSV file if it is provided
    if hara_available:
        hara_generator = HARAParser(hara_file=args.HARA_filepath)
        for item_id, hazard_dict in hara_generator.hara_dict.items():
            bt_hara = BehaviorTree(name=item_id, probabilistic=args.probabilistic)
            bt_hara.generate_from_hara(hazard_dict)
    
    for fta in fta_list:
        # Generate the behavior tree diagram from every fault tree diagram
        bt = BehaviorTree(name=fta.name, probabilistic=args.probabilistic)
        if prev_bt is None:
            prev_bt = bt
        bt.event_number = prev_bt.event_number
        if args.probabilistic:
            bt.node_probabilities = fta_parser.node_probabilities
        bt.generate_from_fault_tree(fta)
        bt.generate_xml_file(folder_name=behavior_tree_folder, view=args.view)
        
        # Attach the singular hazard detection nodes to the HARA behavior tree
        if hara_available:
            bt_hara.attach_hazard_detection(bt, hara_generator.hara_dict)

        # Generate the C++ code template for the behavior tree if the flag is set
        if args.generate_cpp and not hara_available:
            code_generator.generate_main_cpp_file(xml_file_path=bt.xml_file_path, bt_name=bt.name)
            
        prev_bt = bt
        
    if hara_available:
        bt_hara.generate_xml_file(folder_name=behavior_tree_folder, view=args.view)
        if args.generate_cpp:
            code_generator.generate_main_cpp_file(xml_file_path=bt_hara.xml_file_path, bt_name=bt_hara.name)
        

if __name__ == "__main__":    
    main()