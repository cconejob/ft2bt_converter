import xml.etree.ElementTree as ET
from collections import defaultdict
import os

from ft2bt.scripts.formal_verification.ctl_specification_generator import CTLSpecificationGenerator


class SupervisorModelGenerator:
    def __init__(self, bt_xml_file_path):
        # Replace xml by smv
        self.bt_model_smv_path = bt_xml_file_path.replace(".xml", ".smv")
        self.tree = ET.parse(bt_xml_file_path)
        self.root = self.tree.getroot()
        self.smv_dict = {}
        self.prev_event_id = None
        self.subtree_dependencies = defaultdict(list)  # To store dependencies of each subtree
        self.subtree_levels = {}  # To store the calculated level of each subtree

    def find_subtree_dependencies(self):
        # Traverse the XML and find all SubTree dependencies
        for bt_element in self.root.findall(".//BehaviorTree"):
            bt_id = bt_element.get('ID')
            for subtree in bt_element.findall(".//SubTree"):
                referenced_id = subtree.get('ID')
                self.subtree_dependencies[bt_id].append(referenced_id)

    def calculate_levels(self, subtree_id, visited=None):
        if visited is None:
            visited = set()
        
        if subtree_id in visited:
            # Avoid cycles and redundant calculations
            return 0
        visited.add(subtree_id)

        # If the level is already calculated, return it
        if subtree_id in self.subtree_levels:
            return self.subtree_levels[subtree_id]

        # Calculate the level by finding the max level of dependencies + 1
        level = 0
        if subtree_id in self.subtree_dependencies:
            level = 1 + max(self.calculate_levels(dep, visited) for dep in self.subtree_dependencies[subtree_id])

        self.subtree_levels[subtree_id] = level
        return level

    def get_sorted_subtrees(self):
        # Find all dependencies first
        self.find_subtree_dependencies()
        
        # Calculate levels for each subtree
        all_subtrees = set(self.subtree_dependencies.keys()).union(
            dep for deps in self.subtree_dependencies.values() for dep in deps
        )
        for subtree in all_subtrees:
            self.calculate_levels(subtree)
        
        # Sort subtrees by their levels (lowest to highest)
        sorted_subtrees = sorted(self.subtree_levels.items(), key=lambda x: x[1])
        return [subtree for subtree, level in sorted_subtrees]
        
    def convert_subscripts(self, text):
        # Define a mapping for subscript numbers to regular numbers
        subscripts = str.maketrans("₀₁₂₃₄₅₆₇₈₉", "0123456789")
        return text.translate(subscripts)
    
    def invert_case(self, text):
        # Inverts the case of each character in the text
        return "".join([char.upper() if char.islower() else char.lower() for char in text])

    # Helper functions to build SMV components
    def build_condition(self, condition_id, event_name):
        return f"{condition_id} : bt_condition(TRUE, condition & {event_name});"

    def build_sequence(self, sequence_id, *components):
        return f"{sequence_id} : bt_sequence({', '.join(components)});"

    def build_fallback(self, fallback_id, *components):
        return [f"{fallback_id} : bt_fallback({', '.join(components)});"]

    def build_action(self, action_id, prev_event_name):
        return f"{action_id} : bt_action({prev_event_name}.output = Success);"
    
    # Function to parse and translate a specific behavior tree
    def parse_behavior_tree(self, bt_id):
        bt_element = self.root.find(f".//BehaviorTree[@ID='{bt_id}']")
        if bt_element is None:
            print(f"BehaviorTree with ID '{bt_id}' not found.")
            return None
        
        smv_code = []
        vars_list = []
        dependent_subtrees = []
        fallback_counter = 0
        sequence_counter = 0

        # Recursive function to parse nodes
        def parse_node(node, parent_id=""):
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
                while len(sequence_children) > 1:
                    # Nest sequences if more than two children
                    sequence_id = self.convert_subscripts(f"seq_{sequence_counter}")
                    smv_code.append(self.build_sequence(sequence_id, sequence_children[0], sequence_children[1]))
                    sequence_children = [sequence_id] + sequence_children[2:]
                    sequence_counter += 1
                return sequence_children[0]
            elif node.tag == 'Fallback':
                fallback_children = [parse_node(child) for child in node]
                while len(fallback_children) > 1:
                    # Nest fallbacks if more than two children
                    fallback_id = self.convert_subscripts(f"fallback_{fallback_counter}")
                    nested_fallback = self.build_fallback(fallback_id, fallback_children[0], fallback_children[1])
                    smv_code.extend(nested_fallback)
                    fallback_children = [fallback_id] + fallback_children[2:]
                    fallback_counter += 1
                return fallback_children[0]
            elif node.tag == 'Action':
                action_id = self.convert_subscripts(node.get('ID'))
                action_name = self.convert_subscripts(node.get('name'))
                action_code = self.build_action(action_id, self.prev_event_id)
                smv_code.append(action_code)
                return action_id
            elif node.tag == 'SubTree':
                # Assume SubTree is referencing a pre-defined subtree module (like HZ_01 or HZ_02)
                subtree_id = self.convert_subscripts(node.get('ID'))
                dependent_subtrees.append(subtree_id)
                smv_code.append(f"{subtree_id} : {subtree_id}(condition, {', '.join(self.smv_dict[subtree_id])});")
                self.prev_event_id = subtree_id
                return subtree_id

        # Start parsing from the root node of the BehaviorTree
        root_node = bt_element[0]
        subtree_id = parse_node(root_node, bt_id)

        return smv_code, vars_list, subtree_id, dependent_subtrees
    
    def generate_smv_header(self):
        return create_header()

    # Function to generate the SMV module for a behavior tree
    def generate_smv_module(self, bt_id):
        bt_id_clean = self.convert_subscripts(bt_id)
        smv_code, vars_list, subtree_id, dependent_subtrees = self.parse_behavior_tree(bt_id)
        
        # If the behavior tree contains dependent sub-trees, the variables from the sub-trees are added to the list
        for dependent_subtree in dependent_subtrees:
            vars_list += self.smv_dict[dependent_subtree]
            
        # Sort the list of variables to ensure consistency
        vars_list = sorted(list(set(vars_list)))
        
        smv_module = f"MODULE {bt_id_clean}(condition, {', '.join(vars_list)})\n  VAR\n"
        smv_module += "    " + "\n    ".join(smv_code) + "\n"
        smv_module += f"  DEFINE\n    output := {subtree_id}.output;\n\n"
        
        self.smv_dict[bt_id_clean] = vars_list
        
        return smv_module
    
    def generate_smv_main_module(self, root_id):
        # Identify OS variables and event variables
        os_states = []
        os_states_inverted = []
        event_vars = []
        for value in self.smv_dict.values():
            for var in value:
                if "condition_OS" in var:
                    os_states.append(var.replace("Event_condition_", ""))
                    os_states_inverted.append(self.invert_case(var.replace("Event_condition_", "")))
                else:
                    event_vars.append(var)

        # Remove duplicates and sort for consistency
        os_states = sorted(set(os_states))
        os_states_inverted = sorted(set(os_states_inverted))
        event_vars = sorted(set(event_vars))

        # Construct os enumeration and frozen variables for events
        os_enum = ", ".join(os_states_inverted)
        frozen_vars_events = "\n    ".join([f"{event}: boolean;" for event in event_vars])
        frozen_vars_events += "\n    " + "\n    ".join([f"Event_condition_{os}: boolean;" for os in os_states])
        
        # Construct the main module with FROZENVAR, ASSIGN, and VAR blocks
        root_id_clean = self.convert_subscripts(root_id)
        main_module = f"MODULE main\n"
        main_module += f"  FROZENVAR\n"
        main_module += f"    os: {{{os_enum}}};\n"
        main_module += f"    {frozen_vars_events}\n"
        
        # Define Event_condition_OSX variables using ASSIGN block based on os variable
        assign_conditions = "\n    ".join([f"ASSIGN\n    init(Event_condition_{os}) := os = {self.invert_case(os)};" for os in os_states])
        main_module += f"  ASSIGN\n    {assign_conditions}\n"
        
        # Construct VAR block to initialize the root behavior tree
        os_conditions = ", ".join([f"Event_condition_{os}" for os in os_states])
        main_module += f"  VAR\n    {root_id_clean} : {root_id_clean}(TRUE, {', '.join(event_vars)}, {os_conditions});\n"
        
        return main_module
    
    def save_in_file(self, smv_module, file_path):
        with open(file_path, 'w') as file:
            file.write(smv_module)
        print(f"SMV module for BehaviorTree saved to: {file_path}")
        
    def run_nusmv(self):
        # Run NuSMV on the generated SMV file. Command line: NuSMV <file_path>
        os.system(f"NuSMV {self.bt_model_smv_path}")
        
    def forward(self):
        smv_code = self.generate_smv_header()
        
        subtree_list = self.get_sorted_subtrees()[:-1]
    
        for subtree in subtree_list:
            smv_code += self.generate_smv_module(subtree)    
    
        smv_code += self.generate_smv_main_module(subtree_list[-1])
        self.save_in_file(smv_code, self.bt_model_smv_path)


def create_header():
    
    header = """
    -------------------------------------------------------------------------------------------------------------------------
    -- CONDITION NODE
    -------------------------------------------------------------------------------------------------------------------------

    -- The output is `true` if the condition is true, `false` otherwise.

    MODULE bt_condition(enable_condition, condition)
    VAR
        enable : boolean;
        output : { None, Success, Failure };
    ASSIGN
        init(enable) := FALSE;
        init(output) := None;
        next(enable) := enable_condition;
        next(output) :=
        case
            condition & enable_condition: Success;
            TRUE : Failure;
        esac;
    -------------------------------------------------------------------------------------------------------------------------


    -------------------------------------------------------------------------------------------------------------------------
    -- ACTION NODE
    -------------------------------------------------------------------------------------------------------------------------

    -- The output is `running` while the action is being executed, `true` if the action is successful, `false` otherwise.

    MODULE bt_action(enable_condition)
    VAR
        enable : boolean;
        goal_reached : boolean;
        output : { None, Running, Failure, Success };
        i : { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    ASSIGN
        init(enable) := FALSE;
        init(output) := None;
        init(goal_reached) := FALSE;
        init(i) := 0;
        next(enable) := enable_condition | enable;
        next(output) :=
        case
            goal_reached : Success;
            enable_condition | enable: Running;
            TRUE : Failure;
        esac;
        next(i) :=
        case
            output = Running & i<10: i + 1;
            i = 10 : 10;
            TRUE : 0;
        esac;
        next(goal_reached) :=
        case
            goal_reached : goal_reached;
            i < 10: FALSE;
            i >= 10 : TRUE;
            TRUE : goal_reached;
        esac;
    -------------------------------------------------------------------------------------------------------------------------


    -------------------------------------------------------------------------------------------------------------------------
    -- FALLBACK NODE
    -------------------------------------------------------------------------------------------------------------------------

    -- The output is `running` if the left child is `running`, `true` if the left child is `true`, right child otherwise.

    MODULE bt_fallback(left_bt, right_bt)
    DEFINE 
    output := case
        left_bt.output in { Running, Success } : left_bt.output;
        TRUE : right_bt.output;
        esac;
    -------------------------------------------------------------------------------------------------------------------------


    -------------------------------------------------------------------------------------------------------------------------
    -- SEQUENCE NODE
    -------------------------------------------------------------------------------------------------------------------------

    -- The output is `running` if the left child is `running`, `false` if the left child is `false`, right child otherwise.

    MODULE bt_sequence(left_bt, right_bt)
    DEFINE
        output :=
        case
            left_bt.output in { Running, Failure } : left_bt.output;
            TRUE : right_bt.output;
        esac;
    -------------------------------------------------------------------------------------------------------------------------


    -------------------------------------------------------------------------------------------------------------------------
    -- NEGATION NODE
    -------------------------------------------------------------------------------------------------------------------------

    -- The output is `true` if the child output is `false`, `false` otherwise.

    MODULE bt_not(child_bt)
    DEFINE
        output :=
        case
            child_bt.output = Failure : Success;
            child_bt.output = Success : Failure;
            TRUE : child_bt.output;
        esac;
    -------------------------------------------------------------------------------------------------------------------------


    -------------------------------------------------------------------------------------------------------------------------
    -- PLACEHOLDER NODE
    -------------------------------------------------------------------------------------------------------------------------

    -- The output is `success` if the condition is true, `failure` otherwise.

    MODULE bt_placeholder(condition)
    DEFINE
        output := 
        case
            condition : Success;
            TRUE : Failure;
        esac;
    -------------------------------------------------------------------------------------------------------------------------
    """
    
    return header



def main():
    supervisor_model_generator = SupervisorModelGenerator("/home/tda/ft2bt_converter/behavior_trees/BT_i_01.xml")
    supervisor_model_generator.forward()

    ctl_spec_generator = CTLSpecificationGenerator(hara_file_path="/home/tda/ft2bt_converter/ft2bt/test/hara/hara_example.csv")
    specs = ctl_spec_generator.generate_ctl_specifications()
    ctl_spec_generator.write_ctl_specifications(supervisor_model_generator.bt_model_smv_path, specs)
    
    supervisor_model_generator.run_nusmv()

if __name__ == "__main__":
    main()