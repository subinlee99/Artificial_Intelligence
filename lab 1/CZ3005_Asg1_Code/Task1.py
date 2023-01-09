#Task 1
#the algorithm of Dijkstra, used for this task, follow the below
# 1. Set the starting point
# 2. Initialize distance table of each node as inifinite, except for the starting point, which is set as 0
# 3. Chose the shortest distance among the adjacent vertices
# 4. Calculate the distance through that node to another node if that is shorter, modify the distance table
#5. repeat
#

def getfile():
    import json
    FILE_DIST = "./data/Dist.json"
    #get json file and load to Dist
    Dist = json.load(open(FILE_DIST))

    #define a new dict to modify the file 
    #graph format -> {vertex: {adjacent vertex1: distance1,adjacent vertex2: distance2}... } 
    graph={}
    for a, b, in Dist.items(): 
        x = a.split(",")
        graph.setdefault(x[0], {})[x[1]] = Dist[a]
    return graph

def dijkstras_path(graph, start_v):
    import heapq
    import sys
    inf = sys.maxsize
    #every distance except the starting point should be set to "Infinite"
    distances = {vertex: inf for vertex in graph}
    #distance to starting point is 0
    distances[start_v] = 0
    #get the number of the nodes
    n = len(graph)
    #set array path to save the path from starting point to the destination
    path = [0] *(n+1)

    #"priority queue" list to search for the graph.
    priority_q = [(0, start_v)]
    
    #while queue
    while priority_q:

        curr_d, curr_v = heapq.heappop(priority_q)
        #checks whether current node is already processed
        # if visited, do not need to change the distance cost (continue)
        if distances[curr_v]< curr_d :
            continue
        
        #for every neighbooring vertecies compare distance
        for adjacent_v, cost in graph[curr_v].items():
            #if previous distance to node A is bigger than new distance (path passing adjacent_v)
            #chage the distances[adjacent_v] to new distnace, else continue
            if curr_d + cost < distances[adjacent_v]:
                #change the distnace to a new one
                distances[adjacent_v] = curr_d + cost
                #save the path (curr_v -> adjacent_v)
                path[int(adjacent_v)] = curr_v
                #put the new distance and the adjacent_v to a queue
                heapq.heappush(priority_q, (distances[adjacent_v], adjacent_v))
    return distances, path

def print_output(path,distances, start, destination):
    #define infinite
    import sys
    inf = sys.maxsize
    
    #back track the path from the destination to the starting point 
    track = path[50]

    #save the path to a new array
    path_reverse=[]
    path_reverse.append(destination)
    
    len_path=0 #save the lenght of the path
    while (True):
        #back track the path and put into path_reverse array
        path_reverse.append(track) 
        track = path[int(track)] #define the previous node on the path as track
        len_path+=1 
        if track == start: #if reached the starting node stop
            path_reverse.append(start)
            break

    count =0
    print("Shortest path: " , end='')
    while True:
        if (len_path-count+1)==0:
            print(path_reverse[0])
            break
        #print path from the end of the array 
        print(path_reverse[len_path-count+1],end='->')  
        count+=1

    dis = 0
    #if no short path -> print infitnity
    if distances[destination] == inf:
        dis =  "infinity"
    else: #if there is, print the shortest distance
        dis =  distances[destination] 
    print("Shortest distance:", dis)

def main():
    #Define starting point and the destination nodes
    start = '1' 
    destination = '50'

    #get Distance file and modify it in a new arry
    graph = getfile()

    #get the shortest distance and the path
    distances, path = dijkstras_path(graph, start)

    #print output
    print_output(path, distances, start, destination)

#run 
if __name__ == "__main__":
    main()=