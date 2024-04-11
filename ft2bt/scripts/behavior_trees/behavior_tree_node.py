class BehaviorTreeNode:
    """
    Behavior tree node class. 
    Each node has a node ID, node type, and a list of children
    """
    def __init__(self, node_id, node_type, label=None, probability=None):
        self.node_id = node_id
        self.node_type = node_type
        self.children = list()
        self.label = label
        self.probability = probability
        self.level = 0
        self.asil = None