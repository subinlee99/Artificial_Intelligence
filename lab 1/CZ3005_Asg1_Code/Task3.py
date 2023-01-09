# Import dependencies
from queue import PriorityQueue
import math
import json

# Find the shortest path between 2 locations within EnergyBudget
class AStarSearch:
    ExploredSet = None
    TotalDistance = None
    TotalEnergy = None
    ShortestPath = ""

    def __init__(self):
        self.ExploredSet = None
        self.TotalDistance = None
        self.TotalEnergy = None
        self.ShortestPath = ""

    def runSearch(self, StartNode, EndNode, EnergyHighCap, G, Cost, Dist, Coord):
        # Set up a PriorityQueue to be used as the frontier of nodes to be explored
        Frontier = PriorityQueue()

        # Initialize the frontier by inserting the start node with 0 energy cost, dist, priority
        # each elem is in order of (priority, dist, (node, cost))
        Frontier.put((0, 0, (StartNode, 0)))

        # Initiallize other required parameters
        Explored = {}  # Dict of explored nodes {node : parentNode}
        DistCost = {}  # Dict of dist cost from start to end node {node : cost}
        minCost = {}  # Dict to store the lowest cost for the specific node {node : cost}
        minDist = {}  # Dict to store the lowest dist for the specific node {node : distCost}

        # Init values for the starting node
        Explored[(StartNode, 0)] = None  # Start node no parent

        DistCost[(StartNode, 0)] = 0  # Start to Start cost = 0

        # Repeat the process until either the frontier is empty or the goal has been reached
        while not Frontier.empty():
            # Get the first vertex in the frontier to be explored ([1] is used to get the node id)
            currentP, currentDist, (current, currentCost) = Frontier.get() 
            
            # Check if there exist a short path to current node/vertex, check the next element in the priority queue
            if current in minDist and current in minCost and minDist[current] <= currentDist and minCost[current] <= currentCost:
                continue
            
            # Update the min dictionary for dist and cost when there exist a better dist/energy cost to the current node that we are visiting
            if current not in minDist or minDist[current] > currentDist:
                minDist[current] = currentDist
            if current not in minCost or minCost[current] > currentCost:
                minCost[current] = currentCost
            
            # Check if the current node is the goal
            if current == EndNode:
                break
            
            # If yet to reach the goal, continue to explore all neighbours adjacent to current node
            neighbours = G[current]
            for next in neighbours:  # Explore all adjacent nodes
                # Get cost for this new node
                dist = Dist[current + "," + next]
                energy = Cost[current + "," + next]
                
                # Calculate totalCost based on current node
                newDist = DistCost[(current, currentCost)] + dist
                newEnergy = currentCost + energy

                # Consider whether to explore this node,
                # Either unexplored or the new costs to this node is better than the explored ones
                # Check energy cost, dont explore if too high
                if (next, newEnergy) not in Explored and newEnergy <= EnergyHighCap:
                    # calculate the A*'s FCost to be used as Priority
                    priority = newDist + (self.heuristic(current, next, Coord))
                    
                    # Insert the next node into the frontier, with its fCost as the exploration priority
                    Frontier.put((priority, newDist, (next, newEnergy)))
                    
                    # Store its costs
                    DistCost[(next, newEnergy)] = newDist

                    # Update the exploration status and set the current node to be the exploration parent node
                    Explored[(next, newEnergy)] = (current, currentCost)
                            
        # Store calculated data into class
        self.ExploredSet = Explored
        self.TotalDistance = DistCost[EndNode,minCost[EndNode]]
        self.TotalEnergy = minCost[EndNode]
        self.exploredResultStr(EndNode,minCost[EndNode],Dist,Cost)
        return self

    def exploredResultStr(self, EndNode, MinCost, Dist, Cost, Checker = False):
        result = ""
        tempDist = 0
        tempCost = 0
        current = self.ExploredSet[EndNode, MinCost]
        while current != None:
            result = current[0] + "->" + result
            if self.ExploredSet[current] != None and Checker == True:
                tempDist += Dist[self.ExploredSet[current][0]+ ',' + current[0]]
                tempCost += Cost[self.ExploredSet[current][0] + ',' + current[0]]
            current = self.ExploredSet[current]
        self.ShortestPath = result + EndNode
        if Checker == True:
            tempDist += Dist[self.ExploredSet[EndNode, MinCost][0]+','+EndNode]
            tempCost += Cost[self.ExploredSet[EndNode, MinCost][0] + ',' + EndNode]
            print(tempDist)
            print(tempCost)
        return self

    def heuristic(self, currVertex, nextVertex, coord):
        currCoord = coord[currVertex]
        nextCoord = coord[nextVertex]

        # Calculate Eucleadian Dist (Pythagorean)
        eucDist = pow(nextCoord[0] - currCoord[0], 2) + pow(nextCoord[1] - currCoord[1], 2)
        return math.sqrt(eucDist) 
    
    def printResults(self):
        print("Shortest Path: " + self.ShortestPath)
        print("Shortest Distance: ", self.TotalDistance)
        print("Total Energy Cost: ", self.TotalEnergy)

# End of class
################################################################################################################################################################################################

# Constants
FILE_G = "./data/G.json"
FILE_COST = "./data/Cost.json"
FILE_DIST = "./data/Dist.json"
FILE_COORD = "./data/Coord.json"

# Global json data, Set to none first
G = None     # Graph dictionary
Cost = None  # Edge cost dictionary
Dist = None  # Edge distance dictionary
Coord = None # Node coordination dictionary

SET_DEFAULT = False    # Set initial state to default
ITERATIONS = 10     # Num of iterations

# Run the program 
def main():  
    # Load json into dict from file
    # print("\n< Loading JSON files")
    G = jsonLoadFromFile(FILE_G)
    Cost = jsonLoadFromFile(FILE_COST)
    Dist = jsonLoadFromFile(FILE_DIST)
    Coord = jsonLoadFromFile(FILE_COORD)
    #print()

    # Set up starting and ending nodes with energy Budget
    Start = "1"
    End = "50"
    EnergyBudget = 287932
    
    #print("\n> Run AStar Search: ")
    AStarSearch().runSearch(Start, End, EnergyBudget, G, Cost, Dist, Coord).printResults()
    return

def jsonLoadFromFile(PATH):
    # Open JSON file
    f = open(PATH)
    # Load JSON object as a dictionary
    data = json.load(f)

    if SET_DEFAULT:
        print("\n> Output: " + PATH)
        i = 0
        for attribute, value in data.items(): # Iterate through the json data
            print(attribute, value)
            i += 1  # increment
            if i  >= ITERATIONS:
                break
    
    # Close file after running
    f.close()
    #print("File: " + PATH + "> Loaded Successfully")
    # Return parsed data
    return data

if __name__ == "__main__":
    main()