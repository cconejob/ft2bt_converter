import argparse
from pathlib import Path

from ft2bt.scripts.fault_trees.xml_fta_parser import XMLFTAParser
from ft2bt.scripts.behavior_trees.behavior_tree import BehaviorTree
from ft2bt.scripts.hara.hara_parser import HARAParser
from ft2bt.scripts.formal_verification.ctl_specification_generator import CTLSpecificationGenerator
from ft2bt.scripts.formal_verification.supervisor_model_generator import SupervisorModelGenerator

def main():  
    parser = argparse.ArgumentParser(description='Convert xml file from drawio to behavior tree xml file for Groot.')
    
    parser.add_argument('-f', '--fta_filepath', type=str, help="*.xml fault tree global path", required=True)
    parser.add_argument('-H', '--HARA_filepath', type=str, help="*.csv HARA file global path.", required=False, default=None)
    parser.add_argument('-o', '--output_folder', type=str, help="Output folder for the behavior trees.")
    parser.add_argument('-ctl', '--ctl_specifications', action='store_true', help="Formally verify FuSa with CTL specifications.")
    parser.add_argument('-v', '--view', action='store_true', help="View the behavior tree renders?")
    args = parser.parse_args()
    
    print("\n-------------------------------------------------------------------------------------------------------------------------")
    print("\033[1mSummary\033[0m")
    
    print("\033[1mInput files\033[0m")
    
    # Get the global path of the package
    module_path = Path(__file__).resolve()
    package_path = module_path.parent.parent.parent
    hara_available = args.HARA_filepath is not None
    
    # Add the .xml extension if it is not present
    if not args.fta_filepath.endswith('.xml'):
        args.fta_filepath += '.xml'
        print('.xml extension not found in the file path.')
        print(f'Added .xml extension to the file path: {args.fta_filepath}')
        
    if not args.HARA_filepath.endswith('.csv'):
        args.HARA_filepath += '.csv'
        print('.csv extension not found in the file path.')
        print(f'Added .csv extension to the file path: {args.HARA_filepath}')
    
    # Generate the fault tree diagram from the XML file
    print(f"Fault tree: {args.fta_filepath}")
    fta_parser = XMLFTAParser(xml_file=args.fta_filepath)
    fta_list = fta_parser.generate_fault_trees(plot=args.view)

    # Initialize the behavior tree and code generator objects
    prev_bt = None
    
    # Create the folder for the behavior trees
    if args.output_folder:
        behavior_tree_folder = args.output_folder
    else:
        behavior_tree_folder = package_path / 'behavior_trees'
        
    print("\n\033[1mOutput files\033[0m")
    if not hara_available:
        # Generate the behavior tree diagram from every fault tree diagram
        for fta in fta_list:
            bt = BehaviorTree(name=fta.name)
            if prev_bt is None:
                prev_bt = bt
            bt.event_number = prev_bt.event_number
            bt.node_probabilities = fta_parser.node_probabilities
            bt.generate_from_fault_tree(fta)

            bt.generate_xml_file(folder_name=behavior_tree_folder, view=args.view)
            print(f"Behavior tree: {bt.xml_file_path}")
                
            prev_bt = bt
            
    else:
        print(f"HARA: {args.HARA_filepath}")
        hara_generator = HARAParser(hara_file=args.HARA_filepath)
        bt_hara_list = list()
        for item_id, hazard_dict in hara_generator.hara_dict.items():
            bt_hara = BehaviorTree(name=item_id)
            bt_hara.generate_from_hara(hazard_dict)
            bt_hara_list.append(bt_hara)
        
        # Generate the behavior tree diagram from every fault tree diagram
        for bt_hara in bt_hara_list:
            for fta in fta_list:
                bt = BehaviorTree(name=fta.name)
                if prev_bt is None:
                    prev_bt = bt
                bt.event_number = prev_bt.event_number
                bt.node_probabilities = fta_parser.node_probabilities
                bt.generate_from_fault_tree(fta)
                
                bt_hara.attach_hazard_detection(bt, hara_generator.hara_dict)
                
            bt_hara.generate_xml_file(folder_name=behavior_tree_folder, view=args.view)
                
            print(f"Behavior tree: {bt_hara.xml_file_path}")
    
    # Fowmally verify with CTL specifications if the flag is set
    if args.ctl_specifications:
        if not hara_available:
            print("HARA file not provided. Cannot generate CTL specifications.")
        else:
            ctl_spec_generator = CTLSpecificationGenerator(hara_file_path=args.HARA_filepath)
            nusmv_result_list = list()
            
            for bt_hara in bt_hara_list:
                supervisor_model_generator = SupervisorModelGenerator(bt_xml_file_path=bt_hara.xml_file_path)
                supervisor_model_generator.forward()

                specs = ctl_spec_generator.generate_ctl_specifications(supervisor_model_generator.root_id)
                ctl_spec_generator.write_ctl_specifications(supervisor_model_generator.bt_model_smv_path, specs)
            
                print(f"SMV Model: {supervisor_model_generator.bt_model_smv_path}\n")
                print(f"Running NuSMV formal verification on the supervisor model: {supervisor_model_generator.root_id}...\n")

                # Run NuSMV on the supervisor model
                result = supervisor_model_generator.run_nusmv()
                nusmv_result_list.append(result)
            
            print("-------------------------------------------------------------------------------------------------------------------------")
            # Inform the user about the results of the NuSMV verification 
            if all(result for result in nusmv_result_list):
                print("\n\033[1mAll CTL specifications were satisfied.\033[0m\n")
            else:
                print("\n\033[1mSome CTL specifications were not satisfied.\033[0m")
                print("\033[1mPlease check the SMV models for more information.\033[0m\n")
            
            print("-------------------------------------------------------------------------------------------------------------------------")
          
if __name__ == "__main__":    
    main()