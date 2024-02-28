import graphviz
import os
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

from behavior_trees.behavior_tree_node import BehaviorTreeNode


class BehaviorTree:
    """
    Behavior tree class.
    Each behavior tree has a dictionary of nodes and a list of edges.
    
    Args:
        name (str, optional): Name of the behavior tree. Defaults to str().
    """
    def __init__(self, name=str()):
        self.nodes = dict()
        self.action = bool()
        self.name = name
        self.event_number = int()
        self.action_number = int()

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
        
    def classify_node(self, node_id, fault_tree):
        """
        Classify a node and add it to the behavior tree
        
        Args:
            node_id (str): Node ID
            fault_tree (nx.DiGraph): Fault tree
        """
        node_label = fault_tree.nodes[node_id].get('label', '')
            
        # If the node is an action node, create a sequence node and a new node representing the action
        if 'action' in node_label.lower():
            self.action = True
            
            # For action nodes, create a sequence node and a new node representing the action
            sequence_node_id = f'sequence_{node_id}'
            action_node_id = f'action_{node_id}'
            self.add_node(sequence_node_id, 'Sequence', label='Sequence')
            self.add_node(action_node_id, 'Action', label=node_label)

            # Link the action node to the sequence node
            self.add_edge(sequence_node_id, action_node_id)
        
        # If the node is another type of node, add it to the behavior tree
        else:
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
            self.action = True
        
        # If the node is another type of node, link it to its parent
        else:
            self.add_edge(source, target)
        
    def postprocess_tree(self):
        """
        Postprocess the tree to rearrange nodes for altering execution order.
        Specifically, ensure that 'Subtree' nodes come before 'Action' nodes on the same level.
        """
        for node_id, node in self.nodes.items():
            has_subtree = any(child.node_type == 'Subtree' for child in node.children)
            has_action = any(child.node_type == 'Action' for child in node.children)

            if has_subtree and has_action:
                subtree_children = [child for child in node.children if child.node_type == 'Subtree']
                action_children = [child for child in node.children if child.node_type == 'Action']
                other_children = [child for child in node.children if child.node_type not in ['Subtree', 'Action']]
                self.nodes[node_id].children = subtree_children + other_children + action_children 
                
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
            
        if self.action:
            self.postprocess_tree()
        
    def get_behavior_tree_name(self, node_id):
        """
        Get the behavior tree name from the nodes
        
        Returns:
            str: Behavior tree name
        """
        return self.nodes[node_id].label.split(' ')[0].strip('"').lower()
            
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
            bt_node = ET.SubElement(parent_element, 'Condition', attrib={'ID': f'Event_{self.event_number}', 'name': node.label.strip('"')})
        elif node_type =='action':
            self.action_number += 1
            name = node.label.strip('"').split(' ')[1:]
            bt_node = ET.SubElement(parent_element, 'Action', attrib={'ID': f'Action_{self.action_number}', 'name': ' '.join(name)})
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
        root = ET.Element('root', attrib={'main_tree_to_execute': 'BehaviorTree'})
        behavior_tree = ET.SubElement(root, 'BehaviorTree', attrib={'ID': 'BehaviorTree'})

        # Add root nodes to the behavior tree
        if self.action:
            actual_root_nodes = [node_id for node_id, node in self.nodes.items() if node.node_type == 'Sequence' and node_id.startswith('sequence_')]
        else:
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
            
        # Render and view the tree graphically using Graphviz if requested
        pdf_file_path = os.path.join(folder_name, 'render', f'BT_{self.name}')
        if view:
            self.render_graphviz_tree(filename=pdf_file_path, view=view)