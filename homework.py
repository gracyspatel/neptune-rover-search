"""
Subject: CSCI-561 Foundations of Artificial Intelligence
Name: Gracy Patel
USC ID: 7810602876
Homework-1 Exploring Neptune (Help rover reach Neptune using BFS, UCS and A*)
"""

# dependencies
import math
import heapq
from queue import Queue

# Healper Methods
# readFile, bfspathfunction, ucspathfunction, astarpathfunction, get3ddistance, is_valid_move

def readFile():
    with open("input.txt", "r") as f:
        algo = f.readline().strip()
        energy_limit = int(f.readline().strip())
        momentum_limit = int(f.readline().strip())
        
        noflocations = int(f.readline().strip())
        locations = {}
        for i in range(noflocations):
            name, x, y, z = f.readline().split()
            locations[name] = (int(x), int(y), int(z))
        
        nofedges = int(f.readline().strip())
        adj = {name: [] for name in locations}
        for i in range(nofedges):
            u, v = f.readline().split()
            adj[u].append(v)
            adj[v].append(u)
        
        f.close()
    return algo, energy_limit, momentum_limit, locations, adj

def outputfile(str):
    with open("output.txt", "w") as f:
        f.write(str)

def get3ddistance(location1, location2):
    return math.sqrt((location1[0] - location2[0])**2 + (location1[1] - location2[1])**2 + (location1[2] - location2[2])**2)

def is_valid_move(location_from, location_to, curr_momentum, mom_limit, ene_limit):
    required_energy = location_to[2] - location_from[2]
    # print("Required Energy:", required_energy)

    if required_energy <= (curr_momentum + ene_limit):
        # print("CAN MOVE")
        if required_energy > 0:
            next_momentum = max(0, curr_momentum - required_energy)
        elif required_energy < 0:
            next_momentum = min(mom_limit, curr_momentum + abs(required_energy))
        elif required_energy == 0:
            next_momentum = curr_momentum # or 0 (always 0)
    else:
        # print("CANNOT MOVE")
        return False, 0
    return True, next_momentum

# all path functions
def bfspathfunction(path):
    return path

def ucspathfunction(visited, goal_node, goal_momentum):
    path = []
    curr = (goal_node, goal_momentum)
    while curr[0] is not None:
        path.append(curr[0])
        # Move to the parent state stored in visited
        _, parent_node, parent_mom = visited[curr]
        curr = (parent_node, parent_mom)
    return path[::-1]

def astarpathfunction(par, curr_node, curr_mom):
    path = []
    curr = (curr_node, curr_mom)
    while curr[0] is not None:
        path.append(curr[0])
        curr = par[curr]
    return path[::-1]

# Algorithm Methods
# BFS, UCS, A*

# BFS
def bfs(source_node, goal_node, adj, locations, momentum_limit, energy_limit):
    que = Queue()
    que.put((source_node,0,[source_node]))

    visited = set()
    visited.add((source_node, 0))

    while not que.empty():
        current_node, current_momentum, path = que.get()

        if current_node == goal_node:
            return (len(path)-1),bfspathfunction(path)
        
        for neighbor in sorted(adj[current_node]):
            is_valid, next_momentum = is_valid_move( locations[current_node], locations[neighbor],current_momentum, momentum_limit, energy_limit )

            if is_valid:
                new_state = (neighbor, next_momentum)
                if new_state not in visited:
                    visited.add(new_state)
                    que.put((neighbor, next_momentum, path+[neighbor]))
    return None, None


# UCS
def ucs(source_node, goal_node, adj, locations, momentum_limit, energy_limit):
    priority_queue = [(0.0, source_node, 0.0)]
    
    visited = {(source_node, 0.0): (0.0, None, None)}

    while priority_queue:
        current_cost, current_node, current_momentum = heapq.heappop(priority_queue)

        # reached gaol condition
        if current_node == goal_node:
            return current_cost, ucspathfunction(visited, goal_node, current_momentum)
        
        if current_cost> visited.get((current_node, current_momentum), (float('inf'),))[0]:
            continue

        for neighbor in adj[current_node]:
            step_cost =  math.sqrt((locations[neighbor][0] - locations[current_node][0])**2 + (locations[neighbor][1] - locations[current_node][1])**2)
            
            is_valid, next_momentum = is_valid_move(
                locations[current_node], 
                locations[neighbor], 
                current_momentum, 
                momentum_limit, 
                energy_limit
            )

            if is_valid:
                total_cost = current_cost + step_cost
                state = (neighbor, next_momentum)

                if state not in visited or total_cost < visited[state][0]:
                    visited[state] = (total_cost, current_node, current_momentum)
                    heapq.heappush(priority_queue, (total_cost, neighbor, next_momentum))
    return None, None

# A*
# here we have heuristic
def astar(source_node, goal_node, adj, locations, momentum_limit, energy_limit):
    heuristic = get3ddistance(locations[source_node],locations[goal_node])
    priority_queue = [(heuristic,0.0, source_node, 0.0)]
    
    visited = {(source_node, 0.0): 0.0}
    parent = {(source_node,0.0): (None, None)}

    while priority_queue:
        funf, fung , current_node, current_momentum = heapq.heappop(priority_queue)

        # reached gaol condition
        if current_node == goal_node:
            return fung, astarpathfunction(parent, current_node, current_momentum)
        # current_cost put this aswell

        if fung> visited.get((current_node, current_momentum), (float('inf'),)):
            continue

        current_location = locations[current_node]

        for neighbor in adj[current_node]:
            neighbor_location = locations[neighbor]
            
            is_valid, next_momentum = is_valid_move(
                locations[current_node], 
                locations[neighbor], 
                current_momentum, 
                momentum_limit, 
                energy_limit
            )

            if is_valid:
                move_cost = get3ddistance(current_location,neighbor_location)
                new_g = fung +move_cost
                h = get3ddistance(neighbor_location,locations[goal_node])
                newf = new_g + h

                state = (neighbor, next_momentum)

                if state not in visited or new_g < visited[state]:
                    visited[state] = new_g
                    parent[state] = (current_node,current_momentum)
                    heapq.heappush(priority_queue, (newf,new_g, neighbor, next_momentum))

    return None, None

def main():
    algo, energy_limit, momentum_limit, locations, adj = readFile()
    
    print("Algorithm:", algo)
    print("Energy Limit:", energy_limit)
    print("Momentum Limit:", momentum_limit)
    # print("Locations:", locations)
    # print("Adjacency List:", adj)
     
    source_node = "start"
    goal_node = "goal"

    if algo == "BFS":
        print("BFS Algo")
        pathlength, path = bfs(source_node, goal_node , adj, locations, momentum_limit, energy_limit)
    elif algo == "UCS":
        print("UCS Algo")
        pathlength, path = ucs(source_node, goal_node , adj, locations, momentum_limit, energy_limit)
    elif algo == "A*":
        print("A* Algo")
        pathlength, path = astar(source_node, goal_node, adj, locations, momentum_limit,energy_limit)
    else:
        print("Invalid Algorithm")

    if pathlength == None and path == None:
        print("FAIL")
        outputfile("FAIL")
    else:
        print("Path Length:", pathlength)
        print("Path:", " ".join(path))
        outputfile(" ".join(path))

if __name__ == "__main__":
    main()