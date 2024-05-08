import graphviz
import os
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import networkx as nx

from .behavior_tree_node import BehaviorTreeNode


"""
======================================
BEHAVIOR TREE CLASS
======================================
"""

class BehaviorTree:
    """
    Behavior tree class.
    Each behavior tree has a dictionary of nodes and a list of edges.
    
    Args:
        name (str, optional): Name of the behavior tree. Defaults to str().
        probabilistic (bool, optional): Whether the behavior tree is probabilistic. Defaults to False.
    """
    def __init__(self, name=str(), probabilistic=False):
        self.nodes = dict()
        self.name = name
        self.event_number = int()
        self.action_number = int()
        self.probabilistic = probabilistic
        
        # Probabilistic behavior tree attributes
        if probabilistic:
            self.node_probabilities = dict()
            self.node_levels = dict()
            self.max_level = int()
            
    """
    ==============================================
    - ADD ELEMENTS TO BEHAVIOR TREE
    ==============================================
    """

    def add_node(self, node_id, node_type, label=None):
        """
        Add a node to the behavior tree. The node ID must be unique.

        Args:
            node_id (str): Node ID
            node_type (str): Node type
            label (str, optional): Node label. Defaults to None.
        """
        self.nodes[node_id] = BehaviorTreeNode(node_id, node_type, label)

    def add_edge(self, parent_id, child_id):
        """
        Add an edge to the behavior tree. The edge must be between two existing nodes.

        Args:
            parent_id (str): Parent node ID
            child_id (str): Child node ID
        """
        self.nodes[parent_id].children.append(self.nodes[child_id])
        
    def get_behavior_tree_name(self, node_id):
        """
        Get the behavior tree name from the nodes
        
        Returns:
            str: Behavior tree name
        """
        return self.nodes[node_id].label.split(' ')[0].strip('"').lower()
        
    """
    ==============================================
    - CONVERSION FROM FAULT TREES
    ==============================================
    """
    
    def generate_from_fault_tree(self, fault_tree):
        """
        Generate a behavior tree from the fault tree NetworkX graph
        
        Args:
            fault_tree (nx.DiGraph): Fault tree
        """
        # Reverse the fault tree to start from the root nodes
        fault_tree = fault_tree.reverse()
        
        # Classify nodes and add them to the behavior tree
        for node_id in fault_tree.nodes:
            self.classify_node(node_id, fault_tree)
        
        # Add edges based on the digraph structure of the reversed graph
        for source, target in fault_tree.edges():
            self.classify_edge(source, target, fault_tree)
        
        # Add probabilities to the nodes if the behavior tree is probabilistic
        if self.probabilistic:
            self.add_node_probabilities(fault_tree)
            self.extract_node_levels(fault_tree)
            self.sort_nodes_probability_level(fault_tree)
        
    def classify_node(self, node_id, fault_tree):
        """
        Classify a fault tree node and add it to the behavior tree
        
        Args:
            node_id (str): Node ID
            fault_tree (nx.DiGraph): Fault tree
        """
        node_label = fault_tree.nodes[node_id].get('label', '')
        
        # If the node is another type of node, add it to the behavior tree
        if fault_tree.in_degree(node_id) == 0:
            node_type = 'Root'
        elif fault_tree.out_degree(node_id) == 0:
            node_type = 'Condition'
        else:
            if "AND" in node_label:
                node_type = "Sequence" 
            elif "OR" in node_label:
                node_type = "Fallback"    
            else:
                node_type = "Subtree"
                
        self.add_node(node_id, node_type, label=node_label)
            
    def classify_edge(self, source, target, fault_tree):
        """
        Classify an edge and add it to the behavior tree

        Args:
            source (str): Source node ID
            target (str): Target node ID
            fault_tree (nx.DiGraph): Fault tree
        """
        source_label = fault_tree.nodes[source].get('label', '')
            
        # If the node is an action node, link it to the sequence node
        if 'action' in source_label.lower():
            sequence_node_id = f'sequence_{source}'
            self.add_edge(sequence_node_id, target)
        
        # If the node is another type of node, link it to its parent
        else:
            self.add_edge(source, target)
            
    def extract_node_levels(self, fault_tree):
        """
        Extract the levels of the nodes in the fault tree
        
        Args:
            fault_tree (nx.DiGraph): Fault tree
        """
        # Get the levels of the nodes in the fault tree
        levels = dict(nx.shortest_path_length(fault_tree.reverse()))
        
        # Extract the maximum level of the nodes
        for node_id, level in levels.items():
            for key, value in level.items():
                max_value = max(value, max_value) if 'max_value' in locals() else value
            self.nodes[node_id].level = max_value
            self.max_level = max(max_value, self.max_level) if 'self.max_level' in locals() else max_value
            
    def sort_nodes_probability_level(self, fault_tree):
        """
        Sort the nodes based on their probability and level
        
        Args:
            fault_tree (nx.DiGraph): Fault tree
        """
        # For each level in the fault tree
        for i in range(self.max_level + 1)[::-1]:
            # For each node in the fault tree
            for node_id in fault_tree.nodes:
                # If the node is at the current level
                if self.nodes[node_id].level == i:
                    # Sort the children of the node based on their probability (from highest to lowest)
                    if len(self.nodes[node_id].children) > 1:
                        self.nodes[node_id].children = sorted(self.nodes[node_id].children, key=lambda x: x.probability, reverse=True)
                        
    """
    ================================================
    * Consider Fault Tree Probabilities
    ================================================
    """
        
    def add_node_probabilities(self, fault_tree):
        """
        Add probabilities to the nodes of the behavior tree
        
        Args:
            fault_tree (nx.DiGraph): Fault tree
        """
        # Calculate the probability of each node
        for node_id in fault_tree.nodes:
            self.calculate_node_probability(node_id, fault_tree)
                
    def calculate_node_probability_sequence(self, node_id, fault_tree):
        """
        Calculate the probability of a sequence node based on its children
        
        Args:
            node_id (str): Node ID
            fault_tree (nx.DiGraph): Fault tree
            
        Returns:
            float: Probability of the node
        """
        p = 1.0
        for child in fault_tree.successors(node_id):
            if child not in self.node_probabilities:
                self.calculate_node_probability(child, fault_tree)
            p *= self.node_probabilities[child]
            
        return p
    
    def calculate_node_probability_fallback(self, node_id, fault_tree):
        """
        Calculate the probability of a fallback node based on its children
        
        Args:
            node_id (str): Node ID
            fault_tree (nx.DiGraph): Fault tree
            
        Returns:
            float: Probability of the node
        """
        p = 1.0
        for child in fault_tree.successors(node_id):
            if child not in self.node_probabilities:
                self.calculate_node_probability(child, fault_tree)
            p *= 1 - self.node_probabilities[child]
            
        return 1 - p
            
    def calculate_node_probability(self, node_id, fault_tree):
        """
        Calculate the probability of a node based on its children
        
        Args:
            node_id (str): Node ID
            fault_tree (nx.DiGraph): Fault tree
        """     
        if self.nodes[node_id].probability is not None:
            return
          
        if self.probabilistic and self.nodes[node_id].node_type == 'Condition':
            p = self.node_probabilities[node_id] if node_id in self.node_probabilities else 0.0
        
        elif self.nodes[node_id].node_type == 'Sequence':
            p = self.calculate_node_probability_sequence(node_id, fault_tree)
        
        elif self.nodes[node_id].node_type == 'Fallback':
            p = self.calculate_node_probability_fallback(node_id, fault_tree)
            
        else:
            return
            
        self.nodes[node_id].probability = p 
        self.node_probabilities[node_id] = p
        
    """
    ==============================================
    - CONVERSION FROM HARA
    ==============================================
    """
    
    def generate_from_hara(self, hazard_dict):
        """
        Generate a behavior tree from the HARA data
        
        Args:
            hazard_dict (dict): HARA data
        """
        # Initialize dictionary for maximum ASIL for each hazard
        hz_asil = dict()
        
        # Add item as a root node
        self.add_node(self.name, 'Root', label=self.name)
        
        # Add a fallback node for the root node
        fallback_root_id = f'fallback_{self.name}'
        self.add_node(fallback_root_id, 'Fallback', label='Fallback')
        self.nodes[self.name].children.append(self.nodes[fallback_root_id])
        
        for operating_scenario_id, hazard_id_dict in hazard_dict.items():
            # Add sequence node for each operating scenario
            seq_operating_scenario_id = f'sequence_{operating_scenario_id}'
            self.add_node(seq_operating_scenario_id, 'Sequence', label=seq_operating_scenario_id)
            self.nodes[fallback_root_id].children.append(self.nodes[seq_operating_scenario_id])
            
            # Add operating scenario as a condition node
            condition_operating_scenario_id = f'condition_{operating_scenario_id}'
            self.add_node(condition_operating_scenario_id, 'Condition', label=condition_operating_scenario_id)
            self.nodes[seq_operating_scenario_id].children.append(self.nodes[condition_operating_scenario_id])
            
            # Add operating scenario as a subtree node
            self.add_node(operating_scenario_id, 'Subtree', label=operating_scenario_id)
            self.nodes[seq_operating_scenario_id].children.append(self.nodes[operating_scenario_id])
            hz_asil[seq_operating_scenario_id] = list()
            
            # Add FallBack node for each operating scenario
            fallback_id = f'fallback_{operating_scenario_id}'
            self.add_node(fallback_id, 'Fallback', label='Fallback')
            self.nodes[operating_scenario_id].children.append(self.nodes[fallback_id])
            
            for hazard_id, data in hazard_id_dict.items():
                # Get the ASIL and Safety State ID from the HARA data
                asil = data['ASIL']
                sequence_id = f'sequence_{hazard_id}_{operating_scenario_id}'
                safety_state_id = data['Safety_State_ID']
                hz_asil[seq_operating_scenario_id].append(asil)
                
                # Create a Sequence node for each operating scenario
                self.add_node(sequence_id, 'Sequence', label='Sequence')
                self.nodes[sequence_id].asil = asil
                self.nodes[fallback_id].children.append(self.nodes[sequence_id])
                
                # Add operating scenario as a condition node
                hazard_id = f'{hazard_id}_{operating_scenario_id}'
                self.add_node(hazard_id, 'Condition', label=hazard_id)
                self.nodes[sequence_id].children.append(self.nodes[hazard_id])
                
                # Add safety state as an action node
                safety_state_id = f'action_{safety_state_id}'
                self.add_node(safety_state_id, 'Action', label=safety_state_id)
                self.nodes[sequence_id].children.append(self.nodes[safety_state_id])           
            
            # Sort Operating Scenario verification by ASIL
            self.sort_nodes_asil(fallback_id)
            
            # Get ASIL level of the Hazard
            self.nodes[seq_operating_scenario_id].asil = sorted(hz_asil[seq_operating_scenario_id])[-1]
            
        # Find the HZ ASIL level from the maximum Operating Scenario ASIL
        self.sort_nodes_asil(fallback_root_id)
        
    def attach_hazard_detection(self, bt, hara_dict):
        """
        Attach hazard detection to the behavior tree nodes
        
        Args:
            bt (BehaviorTree): Behavior tree of the hazard detection
            hara_dict (dict): HARA data
        """
        # For node in the Hazard Detection BT
        for _, node in bt.nodes.items():
            node_label = node.label.strip('"')
            # Get the HARA data for the root node
            if node.node_type == 'Root':
                node.node_type = 'Subtree'
                node.node_label = node_label + '_HARA'
                for item in hara_dict.keys():
                    for operating_scenario in hara_dict[item].keys():
                        if node_label in hara_dict[item][operating_scenario].keys():
                            id = f'sequence_{node_label}_{operating_scenario}'
                            if id in self.nodes.keys():
                                self.nodes[id].children.remove(self.nodes[id].children[0])
                                self.nodes[id].children.insert(0, node)
                                
                                
    def sort_nodes_asil(self, node_id):
        """
        Sort the nodes based on their ASIL
        
        Args:
            node_id (str): Node ID
        """
        self.nodes[node_id].children = sorted(self.nodes[node_id].children, key=lambda x: x.asil, reverse=True)
        
    def check_safety_states(self, operating_scenario_dict):
        """
        Check if safety states are the same for a hazard in the diverse operating scenario
        
        Args:
            operating_scenario_dict: OS data
        """
        ss_list = list()
        asil_list = list()
        
        for os in operating_scenario_dict.keys():
            ss_list.append(operating_scenario_dict[os]['Safety_State_ID'])
            asil_list.append(operating_scenario_dict[os]['ASIL'])

        return len(set(ss_list)) == 1, f'action_{ss_list[0]}', max(asil_list)
                                
    """
    ==============================================
    - RENDERIZE BEHAVIOR TREE
    ==============================================
    """
                
    def create_graphviz_dot(self):
        """
        Create a Graphviz dot string from the nodes and edges. This can be used to render the tree graphically.
        """
        dot = graphviz.Digraph(comment='Behavior Tree')

        for node_id, node in self.nodes.items():
            label = node.label if node.label else node.node_id
            dot.node(node_id, f'{node.node_type}({label})')
            for child in node.children:
                dot.edge(node_id, child.node_id)            

        return dot

    def render_graphviz_tree(self, filename='behavior_tree', view=False):
        """
        Render the behavior tree graphically using Graphviz. The tree is rendered as a PDF file.

        Args:
            filename (str, optional): Filename of the rendered tree. Defaults to 'behavior_tree'.
            view (bool, optional): View the tree after rendering. Defaults to False.
        """
        dot = self.create_graphviz_dot()
        dot.render(filename, view=view, cleanup=True, format='pdf')
    
    """
    ==============================================
    - BEHAVIOR TREE .XML GROOT FILE
    ==============================================
    """
            
    def add_nodes_xml(self, parent_element, node):
        """
        Add nodes to the behavior tree XML recursively. 
        
        Args:
            parent_element (ET.Element): Parent element
            node (BehaviorTreeNode): Node to add
        """
        node_type = node.node_type.lower()
        if node_type == 'sequence':
            bt_node = ET.SubElement(parent_element, 'Sequence')
        elif node_type == 'fallback':
            bt_node = ET.SubElement(parent_element, 'Fallback')
        elif node_type == 'condition':
            self.event_number += 1
            id = node.label.strip('"')
            bt_node = ET.SubElement(parent_element, 'Condition', attrib={'ID': id, 'name': f'Event_{id}'})
        elif node_type =='action':
            self.action_number += 1
            id = '_'.join(node.label.strip('"').split('_')[1:])
            bt_node = ET.SubElement(parent_element, 'Action', attrib={'ID': id, 'name': f'Action_{id}'})
        else:
            node_type = 'root' if node.node_type == 'Root' else 'subtree'
            bt_node = ET.SubElement(parent_element, 'SubTree', attrib={'ID': node.label.strip('"')})
        
        # Recursively add child nodes
        for child in node.children:
            self.add_nodes_xml(bt_node, child)
            
    def convert_xml_structure(self, original_xml):
        """
        Convert the XML structure to be compatible with BehaviorTree.CPP library.

        Args:
            original_xml (str): Original XML string

        Returns:
            str: Converted XML string compatible with BehaviorTree.CPP library
        """
        root = ET.fromstring(original_xml)

        # Create a dictionary to store new BehaviorTrees for each SubTree
        subtrees = {}

        # Find all SubTrees and create corresponding new BehaviorTrees
        for subtree in root.findall('.//SubTree'):
            subtree_id = subtree.get('ID')
            new_tree = ET.Element('BehaviorTree', ID=subtree_id)
            new_tree.extend(subtree)
            subtrees[subtree_id] = new_tree

            # Replace original SubTree with a reference
            ref_subtree = ET.Element('SubTree', ID=subtree_id)
            subtree.clear()
            subtree.attrib = ref_subtree.attrib
        comment = ET.Comment(' ////////// ')
        
        # Append new BehaviorTrees to the root
        for new_tree in subtrees.values():
            root.append(comment)
            root.append(new_tree)

        # Construct the TreeNodesModel section (example, adjust as needed)
        tree_nodes_model = ET.Element('TreeNodesModel')
        for node_id in {'Condition', 'SubTree'}:  # Add other node types as needed
            tree_nodes_model.append(ET.Element(node_id))
        root.append(tree_nodes_model)

        return ET.tostring(root, encoding='unicode')
            
    def generate_xml_file(self, folder_name, view=False):
        """
        Generate a behavior tree XML compatible with BehaviorTree.CPP library and save it to a file.
        
        Args:
            folder_name (str): Folder name to save the XML file
            view (bool, optional): Display the tree. Defaults to False.
        """
        # Add root nodes to the behavior tree
        actual_root_nodes = [node_id for node_id, node in self.nodes.items() if node.node_type == 'Root']
        
        # Create folder inside xml_file folder to store the behavior trees
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        
        root = ET.Element('root', attrib={'main_tree_to_execute': 'BehaviorTree'})
        behavior_tree = ET.SubElement(root, 'BehaviorTree', attrib={'ID': 'BehaviorTree'})
        
        for root_node_id in actual_root_nodes:
            self.add_nodes_xml(behavior_tree, self.nodes[root_node_id])
            self.name = self.get_behavior_tree_name(root_node_id)

        # Generate XML string
        xml_str = ET.tostring(root, encoding='unicode')
        converted_xml = self.convert_xml_structure(xml_str)
        xml_parsed = minidom.parseString(converted_xml)
        pretty_xml_str = xml_parsed.toprettyxml(indent="  ")

        # Write to file
        self.xml_file_path = os.path.join(folder_name, f'BT_{self.name}.xml')
        with open(self.xml_file_path, 'w') as file:
            file.write(pretty_xml_str)
            
        print(f'Behavior tree XML file saved to {self.xml_file_path}')
            
        # Render and view the tree graphically using Graphviz if requested
        pdf_file_path = os.path.join(folder_name, 'render', f'BT_{self.name}')
        if view:
            self.render_graphviz_tree(filename=pdf_file_path, view=view)