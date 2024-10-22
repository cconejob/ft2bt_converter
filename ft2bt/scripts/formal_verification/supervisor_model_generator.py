import xml.etree.ElementTree as ET
from collections import defaultdict
import os
import subprocess

from ft2bt.scripts.formal_verification.ctl_specification_generator import CTLSpecificationGenerator

ft2bt_package_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


class SupervisorModelGenerator:
    def __init__(self, bt_xml_file_path):
        """
        Initializes the SupervisorModelGenerator with a given XML file path.
        - Parses the XML file.
        - Initializes data structures for tracking subtree dependencies and levels.
        
        Args:
            bt_xml_file_path (str): The file path to the behavior tree XML file.
        """
        self.bt_model_smv_path = bt_xml_file_path.replace(".xml", ".smv")
        self.tree = ET.parse(bt_xml_file_path)
        self.root = self.tree.getroot()
        self.smv_dict = {}
        self.prev_event_id = None
        self.subtree_dependencies = defaultdict(list)
        self.subtree_levels = {} 
        self.current_folder = os.path.dirname(os.path.abspath(__file__))

    def find_subtree_dependencies(self):
        """
        Parses the XML file to identify dependencies between subtrees.
        - Stores which subtrees reference other subtrees.
        """
        for bt_element in self.root.findall(".//BehaviorTree"):
            bt_id = bt_element.get('ID')
            for subtree in bt_element.findall(".//SubTree"):
                referenced_id = subtree.get('ID')
                self.subtree_dependencies[bt_id].append(referenced_id)

    def calculate_levels(self, subtree_id, visited=None):
        """
        Recursively calculates the level (depth) of each subtree based on its dependencies.
        - Subtrees that depend on other subtrees have a higher level.
        - Avoids cycles by tracking visited subtrees.
        - Returns the calculated level of the subtree.
        
        Args:
            subtree_id (str): The ID of the subtree to calculate the level for.
            visited (set): A set of visited subtree IDs to avoid redundant calculations.
            
        Returns:
            int: The calculated level of the subtree.
        """
        if visited is None:
            visited = set()
        
        if subtree_id in visited:
            return 0
        visited.add(subtree_id)

        if subtree_id in self.subtree_levels:
            return self.subtree_levels[subtree_id]

        level = int()
        if subtree_id in self.subtree_dependencies:
            level = 1 + max(self.calculate_levels(dep, visited) for dep in self.subtree_dependencies[subtree_id])

        self.subtree_levels[subtree_id] = level
        return level

    def get_sorted_subtrees(self):
        """
        Sorts all subtrees based on their calculated levels (from lowest to highest).
        - Calls find_subtree_dependencies to build the dependency map.
        - Returns the list of subtree IDs in sorted order by level.
        
        Returns:
            list: The sorted list of subtree IDs.
        """
        self.find_subtree_dependencies()
        
        all_subtrees = set(self.subtree_dependencies.keys()).union(
            dep for deps in self.subtree_dependencies.values() for dep in deps
        )
        for subtree in all_subtrees:
            self.calculate_levels(subtree)
        
        sorted_subtrees = sorted(self.subtree_levels.items(), key=lambda x: x[1])
        return [subtree for subtree, level in sorted_subtrees]
        
    def convert_subscripts(self, text):
        """
        Converts subscript characters in a string (e.g., "₁₂₃") to normal numbers.
        - Returns the string with converted numbers.
        
        Args:
            text (str): The text to convert.
            
        Returns:
            str: The text with converted numbers.
        """
        subscripts = str.maketrans("₀₁₂₃₄₅₆₇₈₉", "0123456789")
        return text.translate(subscripts)
    
    def invert_case(self, text):
        """
        Inverts the case of each character in the string (upper -> lower, lower -> upper).
        - Returns the string with inverted cases.
        
        Args:
            text (str): The text to invert the case for.
        
        Returns:
            str: The text with inverted cases.
        """
        return "".join([char.upper() if char.islower() else char.lower() for char in text])

    def build_condition(self, condition_id, event_name):
        """
        Builds an SMV condition node.
        - Returns the condition node as a string.
        
        Args:
            condition_id (str): The ID of the condition node.
            event_name (str): The name of the event associated with the condition.
            
        Returns:
            str: The condition node as a string.
        """
        return f"{condition_id} : bt_condition(TRUE, condition & {event_name});"

    def build_sequence(self, sequence_id, *components):
        """
        Builds an SMV sequence node.
        - Returns the sequence node as a string.
        
        Args:
            sequence_id (str): The ID of the sequence node.
            components (list): The components (nodes) of the sequence.
            
        Returns:
            str: The sequence node as a string.
        """
        return f"{sequence_id} : bt_sequence({', '.join(components)});"

    def build_fallback(self, fallback_id, *components):
        """
        Builds an SMV fallback node.
        - Returns the fallback node as a list of strings.
        """
        return [f"{fallback_id} : bt_fallback({', '.join(components)});"]

    def build_action(self, action_id, prev_event_name):
        """
        Builds an SMV action node.
        - Returns the action node as a string.
        """
        return f"{action_id} : bt_action({prev_event_name}.output = Success);"
    
    def parse_behavior_tree(self, bt_id):
        """
        Parses a specific behavior tree from the XML file, generating its SMV code.
        - Recursively parses nodes, conditions, sequences, fallbacks, actions, and subtrees.
        - Returns the generated SMV code, variable list, subtree ID, and dependent subtrees.
        """
        bt_element = self.root.find(f".//BehaviorTree[@ID='{bt_id}']")
        if bt_element is None:
            print(f"BehaviorTree with ID '{bt_id}' not found.")
            return None
        
        smv_code = list()
        vars_list = list()
        dependent_subtrees = list()
        fallback_counter = int()
        sequence_counter = int()

        def parse_node(node):
            """
            Parses a node in the behavior tree.
            - Recursively parses the node and its children.
            - Builds the SMV code for the node.
            
            Args:
                node (Element): The XML element representing the node.
            """
            nonlocal fallback_counter, sequence_counter
            if node.tag == 'Condition':
                condition_id = self.convert_subscripts(node.get('ID'))
                event_name = self.convert_subscripts(node.get('name'))
                condition_code = self.build_condition(condition_id, event_name)
                smv_code.append(condition_code)
                vars_list.append(self.convert_subscripts(node.get('name')))
                self.prev_event_id = condition_id
                return condition_id
            elif node.tag == 'Sequence':
                sequence_children = [parse_node(child) for child in node]
                if len(sequence_children) == 1:
                    placeholder_id = f"empty_{sequence_counter}"
                    smv_code.append(f"{placeholder_id} : bt_placeholder(TRUE);")
                    sequence_children.append(placeholder_id)
                while len(sequence_children) > 1:
                    sequence_id = self.convert_subscripts(f"seq_{sequence_counter}")
                    smv_code.append(self.build_sequence(sequence_id, sequence_children[0], sequence_children[1]))
                    sequence_children = [sequence_id] + sequence_children[2:]
                    sequence_counter += 1
                return sequence_children[0]
            elif node.tag == 'Fallback':
                fallback_children = [parse_node(child) for child in node]
                if len(fallback_children) == 1:
                    placeholder_id = f"empty_{fallback_counter}"
                    smv_code.append(f"{placeholder_id} : bt_placeholder(FALSE);")  # Add bt_placeholder
                    fallback_children.append(placeholder_id)  # Add the placeholder as the second child
                while len(fallback_children) > 1:
                    fallback_id = self.convert_subscripts(f"fallback_{fallback_counter}")
                    nested_fallback = self.build_fallback(fallback_id, fallback_children[0], fallback_children[1])
                    smv_code.extend(nested_fallback)
                    fallback_children = [fallback_id] + fallback_children[2:]
                    fallback_counter += 1
                return fallback_children[0]
            elif node.tag == 'Action':
                action_id = self.convert_subscripts(node.get('ID'))
                action_code = self.build_action(action_id, self.prev_event_id)
                smv_code.append(action_code)
                return action_id
            elif node.tag == 'SubTree':
                subtree_id = self.convert_subscripts(node.get('ID'))
                dependent_subtrees.append(subtree_id)
                smv_code.append(f"{subtree_id} : {subtree_id}(condition, {', '.join(self.smv_dict[subtree_id])});")
                self.prev_event_id = subtree_id
                return subtree_id

        root_node = bt_element[0]
        subtree_id = parse_node(root_node)
        return smv_code, vars_list, subtree_id, dependent_subtrees
    
    def generate_smv_header(self):
        """
        Generates the SMV header, which includes definitions for behavior tree node types (e.g., condition, sequence).
        
        Returns:
            str: The SMV header as a string.
        """
        header_file_path = os.path.join(self.current_folder, "smv_header.txt")
        
        with open(header_file_path, 'r') as file:
            header = file.read()
        return header+'\n\n'

    def generate_smv_module(self, bt_id):
        """
        Generates the SMV module for a specific behavior tree.
        - Constructs the full SMV module code for the tree and updates the dictionary of SMV variables.
        - Returns the SMV module as a string.
        
        Args:
            bt_id (str): The ID of the behavior tree to generate the module for.
            
        Returns:
            str: The SMV module for the behavior tree as a string.
        """
        bt_id_clean = self.convert_subscripts(bt_id)
        smv_code, vars_list, subtree_id, dependent_subtrees = self.parse_behavior_tree(bt_id)

        for dependent_subtree in dependent_subtrees:
            vars_list += self.smv_dict[dependent_subtree]
        vars_list = sorted(list(set(vars_list)))

        smv_module = f"MODULE {bt_id_clean}(condition, {', '.join(vars_list)})\n  VAR\n"
        smv_module += "    " + "\n    ".join(smv_code) + "\n"
        smv_module += f"  DEFINE\n    output := {subtree_id}.output;\n\n"
        
        self.smv_dict[bt_id_clean] = vars_list
        return smv_module
    
    def generate_smv_main_module(self, root_id):
        """
        Generates the main SMV module, including OS and event variables.
        - Constructs the full SMV module code for the main module.
        - Returns the main SMV module as a string.
        
        Args:
            root_id (str): The ID of the root behavior tree.
            
        Returns:
            str: The main SMV module as a string.
        """
        os_states = list()
        os_states_inverted = list()
        event_vars = list()
        for value in self.smv_dict.values():
            for var in value:
                if "condition_OS" in var:
                    os_states.append(var.replace("Event_condition_", ""))
                    os_states_inverted.append(self.invert_case(var.replace("Event_condition_", "")))
                else:
                    event_vars.append(var)

        os_states = sorted(set(os_states))
        os_states_inverted = sorted(set(os_states_inverted))
        event_vars = sorted(set(event_vars))

        os_enum = ", ".join(os_states_inverted)
        frozen_vars_events = "\n    ".join([f"{event}: boolean;" for event in event_vars])
        frozen_vars_events += "\n    " + "\n    ".join([f"Event_condition_{os}: boolean;" for os in os_states])

        root_id_clean = self.convert_subscripts(root_id)
        self.root_id = root_id_clean
        main_module = f"MODULE main\n"
        main_module += f"  FROZENVAR\n"
        main_module += f"    os: {{{os_enum}}};\n"
        main_module += f"    {frozen_vars_events}\n"

        assign_conditions = "\n    ".join([f"ASSIGN\n    init(Event_condition_{os}) := os = {self.invert_case(os)};" for os in os_states])
        main_module += f"  ASSIGN\n    {assign_conditions}\n"

        os_conditions = ", ".join([f"Event_condition_{os}" for os in os_states])
        main_module += f"  VAR\n    {root_id_clean} : {root_id_clean}(TRUE, {', '.join(event_vars)}, {os_conditions});\n"
        return main_module
    
    def save_in_file(self, smv_module, file_path):
        """
        Saves the generated SMV module to the specified file path.
        - Overwrites the existing file if it exists.
        
        Args:
            smv_module (str): The SMV module to save.
            file_path (str): The file path to save the SMV module to.
        """
        with open(file_path, 'w') as file:
            file.write(smv_module)
        print(f"SMV module for BehaviorTree saved to: {file_path}")
        
    def run_nusmv(self):
        """
        Runs NuSMV on the generated SMV file.
        - Calls the NuSMV binary using the system's command line.
        
        Returns:
            bool: True if the formal verification was successful, False otherwise.
        """
        try:
            result = subprocess.run(
                ["NuSMV", self.bt_model_smv_path], 
                capture_output=True, 
                text=True, 
                check=True
            )
            # Output the results
            print(result.stdout)  # Standard output (the result of the formalization)
            print(result.stderr)  # Standard error (if any warnings or errors occurred)
            
            # Check if the output indicates success or failure in the formalization
            if "is true" in result.stdout:
                print(f"\033[1mFormal verification for the BehaviorTree {self.root_id} was successful.\033[0m\n")
                return True
            else:
                print(f"\033[1mFormal verification for the BehaviorTree {self.root_id} failed or returned false.\033[0m\n")
                return False
                
        except subprocess.CalledProcessError as e:
            print(f"Error occurred: {e.stderr}")
            return False
        
    def forward(self):
        """
        Generates the full SMV model by processing all subtrees and the main module.
        - Saves the full model to the specified file path.
        """
        smv_code = self.generate_smv_header()
        subtree_list = self.get_sorted_subtrees()[:-1]
    
        for subtree in subtree_list:
            smv_code += self.generate_smv_module(subtree)    
    
        smv_code += self.generate_smv_main_module(subtree_list[-1])
        self.save_in_file(smv_code, self.bt_model_smv_path)


def main():
    bt_folder = os.path.join(ft2bt_package_path, "behavior_trees")
    hara_folder = os.path.join(ft2bt_package_path, "ft2bt/test/hara")
    supervisor_model_generator = SupervisorModelGenerator(f"{bt_folder}/BT_i_01.xml")
    supervisor_model_generator.forward()

    ctl_spec_generator = CTLSpecificationGenerator(hara_file_path=f"{hara_folder}/hara_example.csv")
    specs = ctl_spec_generator.generate_ctl_specifications()
    ctl_spec_generator.write_ctl_specifications(supervisor_model_generator.bt_model_smv_path, specs)
    
    supervisor_model_generator.run_nusmv()

if __name__ == "__main__":
    main()