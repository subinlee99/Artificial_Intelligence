# import dependencies
from queue import PriorityQueue
import json

# constants
FILE_G = "./data/G.json"
FILE_COST = "./data/Cost.json"
FILE_DIST = "./data/Dist.json"
FILE_COORD = "./data/Coord.json"
BUDGET = 287932
ROOT = "1"
GOAL = "50"

class Node:
    """
    Node class to represent the distance, cost of the node from the root node.
    It also contains the list of its neighboring nodes and the previous node that 
    led to this node

    Constructor arguments:
    dist -- the total distance from the root node to this node
    cost -- the total energy cost from the root node to this node
    current -- the number representing this node e.g "1", "50"
    neighbors -- the list of nodes that are neighbors of this node e.g. ["2", "4", "5"]
    prevNode -- the node object that precedes the current node  
    """
    def __init__(self, dist, cost, current, neighbors, prevNode):
        self.dist = dist
        self.cost = cost
        self.current = current
        self.neighbors = neighbors
        self.prev = prevNode
        return
        
    def get_distance(self):
        return self.dist
    
    def get_cost(self):
        return self.cost
    
    def get_previousNode(self):
        return self.prev
    
    def get_neighbors(self):
        return self.neighbors
    
    def get_current(self):
        return self.current
    
    # overridden methods for object comparison
    def __eq__(self, other):
        return self.dist == other.dist

    def __lt__(self, other):
        return self.dist < other.dist

    def __gt__(self, other):
        return self.dist > other.dist
    
    def __cmp__(self,other):
        return cmp(self, other)

def get_path(node):
    """
    Function takes in a node and then it goes from the current node to its previous node 
    all the way to the root node so we can trace the path from the root node.
    
    Keyword arguments:
    node -- the node whom path we want to trace 

    Return:
    The path from the root node to the given node
    """
    path = []
    while (node.get_previousNode() is not None):
        path.insert(0,node.get_current())  # add the previous node to the front of the list
        path.insert(0,"->")
        node = node.get_previousNode()
    
    path.insert(0,node.get_current())
    return ''.join(map(str,path))

def uniform_cost_search(graph, dist, cost, budget, root, goal):
    """
    Function implements Uniform Cost Search using a priority queue
    to find shortest path within the energy budget
    
    Keyword arguments:
    graph -- the entire graph in the form of an adjacency list. Python dictionary
             where key is the node and value is the list of its neighboring nodes
    dist -- all the distances between one node to another. Python dictionary where
            key is the nodes involved and value is the distance.
            For e.g. 
            dist = { "1,2": 4, "1,3": 2, "1,4": 4 } means that the distance from
            node 1 to node 2 is 4, node 1 to node 3 is 2 and node 1 to node 4 is
            4.
    cost -- all the energy costs between one node to another. Python dictionary 
            where key is the nodes involved and value is the energy cost. Same 
            format as dist.
    budget -- the energy cost budget to adhere to
    root -- the starting node
    goal -- the goal node
    """
    q = PriorityQueue()
    visited = {}  # dictionary to store the nodes that have already been visited
    
    rootNode = Node(0, 0, root, graph[root], None)  # set up the root node
    q.put(rootNode)
    
    while not q.empty():
        cur = q.get()
        if cur.get_current() == goal:
            if cur.get_cost() <= budget:  # if the node is the goal node, check that it is within budget
                path = get_path(cur)
                print(f'Shortest path: {path}')
                print("Shortest distance: %.0f" % cur.get_distance())
                print(f'Total energy cost: {cur.get_cost()}')
                break
        else:
            neighbors = cur.get_neighbors()
            visited[cur.current] = True # add current node to the visited list since it is not a goal node
            # add each neighboring node of the current node to the priority queue it has not already been visited
            for neighbor in neighbors:
                if (neighbor not in visited):
                    neighborDist = dist[cur.get_current() + "," + neighbor] + cur.get_distance()
                    neighborCost = cost[cur.get_current() + "," + neighbor] + cur.get_cost()
                    if neighborCost <= budget: # only add neighboring node if the current energy cost does not exceed the budget
                        q.put(Node(neighborDist, neighborCost, neighbor, graph[neighbor], cur))

# the main program
def main():
	G = open(FILE_G)
	Dist = open(FILE_DIST)
	Cost = open(FILE_COST)

	G_JSON = json.load(G)
	Dist_JSON = json.load(Dist)
	Cost_JSON = json.load(Cost)

	uniform_cost_search(G_JSON, Dist_JSON, Cost_JSON, BUDGET, ROOT, GOAL)

if __name__ == "__main__":
    main()