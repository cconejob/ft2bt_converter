import graphviz
import os
import html
import xml.etree.ElementTree as ET

from behavior_tree_node import BehaviorTreeNode


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
        return self.nodes[node_id].children[0].label.split(' ')[0].strip('"').lower()
            
    def add_nodes_xml(self, parent_element, node):
        """
        Add nodes to the behavior tree XML recursively. 
        
        Args:
            parent_element (ET.Element): Parent element
            node (BehaviorTreeNode): Node to add
        """
        if node.node_type == 'Sequence':
            bt_node = ET.SubElement(parent_element, 'Sequence', attrib={'name': node.label.strip('"')})
        elif node.node_type == 'Fallback':
            bt_node = ET.SubElement(parent_element, 'Fallback', attrib={'name': node.label.strip('"')})
        elif node.node_type == 'Condition':
            self.event_number += 1
            bt_node = ET.SubElement(parent_element, 'Condition', attrib={'ID': f'Event_{self.event_number}', 'name': node.label.strip('"')})
        elif 'action' in node.node_type.lower() :
            self.action_number += 1
            bt_node = ET.SubElement(parent_element, 'Action', attrib={'ID': f'Action_{self.action_number}', 'name': node.label.strip('"')})
        else:
            bt_node = ET.SubElement(parent_element, 'SubTree', attrib={'name': node.label.strip('"')})
        
        # Recursively add child nodes
        for child in node.children:
            self.add_nodes_xml(bt_node, child)
            
    def generate_xml_file(self, folder_name, render=False, view=False):
        """
        Generate a behavior tree XML compatible with BehaviorTree.CPP library and save it to a file.
        
        Args:
            folder_name (str): Folder name to save the XML file
            render (bool, optional): Render the tree graphically using Graphviz. Defaults to False.
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
        
        # Unescape HTML entities in the XML string
        xml_str = html.unescape(xml_str)

        # Write to file
        xml_file_path = os.path.join(folder_name, f'BT_{self.name}.xml')
        with open(xml_file_path, 'w') as file:
            file.write(xml_str)
            
        # Render and view the tree graphically using Graphviz if requested
        pdf_file_path = os.path.join(folder_name, 'render', f'BT_{self.name}')
        if render:
            self.render_graphviz_tree(filename=pdf_file_path, view=view)