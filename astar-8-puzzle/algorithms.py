from copy import deepcopy
import numpy as np
from itertools import combinations

class State:
    def __init__(self, array):
        self.array = np.array(array)
    
    def __repr__(self):
        return str(self.array[0]) + '\n' + str(self.array[1]) + '\n' + str(self.array[2]) + '\n'
    
    def __eq__(self, other):
        return (self.array == other.array).all()


class PriorityQueue:
    def __init__(self):
        self.list = []

    def put(self, item, priority):
        self.list.append((priority, item))
        self.list.sort(key=lambda x: x[0])

    def get(self):
        return self.list.pop(0)[1]
    
    def empty(self):
        return len(self.list) == 0

class Node:
    def __init__(self, start_state):
        self.state = start_state
        self.pos = self.get_pos()
        self.path = [start_state]
    
    def reach_next_state(self, next_state):
        new_node = deepcopy(self)
        new_node.state = next_state
        new_node.pos = new_node.get_pos()
        new_node.path.append(next_state)
        
        return new_node
    
    def get_pos(self):
        for i in range(3):
            for j in range(3):
                if self.state.array[i][j] == 0:
                    return i, j

def goal_test(node, stop_state):
    return node.state == stop_state

def bfs(start_state, stop_state):
    node = Node(start_state)
    fringe = [node]
    cnt = 0
    while True:
        cnt += 1
        
        if fringe == []:
            return False
        node = fringe.pop(0)
        if goal_test(node, stop_state):
            print("Found path in {} steps".format(cnt))
            return node.path
        # successor generator
        x, y = node.pos
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            if i+x < 0 or i+x >= 3 or j+y < 0 or j+y >= 3:
                continue
            new_state = deepcopy(node.state)
            new_state.array[x+i][y+j], new_state.array[x][y] = new_state.array[x][y], new_state.array[x+i][y+j] # swap
            new_node = node.reach_next_state(new_state)
            if new_node.state in node.path:         # avoid cycles
                continue
            fringe.append(new_node)

def dfs(start_state, stop_state):
    node = Node(start_state)
    fringe = [node]
    cnt = 0
    while True:
        cnt += 1
        
        if fringe == []:
            return False
        node = fringe.pop(0)
        if goal_test(node, stop_state):
            print("Found path in {} steps".format(cnt))
            return node.path
        # successor generator
        x, y = node.pos
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            if i+x < 0 or i+x >= 3 or j+y < 0 or j+y >= 3:
                continue
            new_state = deepcopy(node.state)
            new_state.array[x+i][y+j], new_state.array[x][y] = new_state.array[x][y], new_state.array[x+i][y+j] # swap
            new_node = node.reach_next_state(new_state)
            if new_node.state in node.path:         # avoid cycles
                continue
            fringe.insert(0, new_node)

# best-first search 
def ucs(start_state, stop_state):
    node = Node(start_state)
    fringe = PriorityQueue()
    fringe.put(node, 0)
    cnt = 0
    while True:
        cnt += 1
        
        if fringe.empty():
            return False
        node = fringe.get()
        if goal_test(node, stop_state):
            print("Found path in {} steps".format(cnt))
            return node.path
        # successor generator
        x, y = node.pos
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            if i+x < 0 or i+x >= 3 or j+y < 0 or j+y >= 3:
                continue
            new_state = deepcopy(node.state)
            new_state.array[x+i][y+j], new_state.array[x][y] = new_state.array[x][y], new_state.array[x+i][y+j] # swap
            new_node = node.reach_next_state(new_state)
            if new_node.state in node.path:         # avoid cycles
                continue
            fringe.put(new_node, len(new_node.path)-1)

def astar(start_state, stop_state, heuristic):
    heuristic_available = ["Hamming", "Manhatten", "Permutation Inversion", "non-admissible"]
    if heuristic not in heuristic_available:
        print("Heuristic not available")
    elif heuristic == "Hamming":
        hn = lambda node: sum([sum([1 for i in range(3) for j in range(3) if node.state.array[i][j] != stop_state.array[i][j]])])
    elif heuristic == "Manhatten":
        hn = lambda node: sum([sum([abs(i-j) for i, j in zip(*node.state.array.nonzero())])])
    elif heuristic == "Permutation Inversion":
        def cal_inversions(array):
            array = list(array)
            inversions = 0
            for i, j in combinations(stop_state.array.reshape(9), 2):
                if array.index(i) > array.index(j):
                    inversions += 1
            return inversions
            
        hn = lambda node: cal_inversions(node.state.array.reshape(9))
    elif heuristic == "non-admissible":
        # non-admissible heuristic, the longer the path, the smaller the hn
        hn = lambda node: int(100 / len(node.path)) 



    node = Node(start_state)
    fringe = PriorityQueue()
    fringe.put(node, 0)
    cnt = 0
    while True:
        cnt += 1
        
        if fringe.empty():
            return False
        node = fringe.get()
        if goal_test(node, stop_state):
            print("Found path in {} steps".format(cnt))     # steps = num of expanded nodes
            return node.path
        # successor generator
        x, y = node.pos
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            if i+x < 0 or i+x >= 3 or j+y < 0 or j+y >= 3:
                continue
            new_state = deepcopy(node.state)
            new_state.array[x+i][y+j], new_state.array[x][y] = new_state.array[x][y], new_state.array[x+i][y+j] # swap
            new_node = node.reach_next_state(new_state)
            if new_node.state in node.path:         # avoid cycles
                continue
            fringe.put(new_node, len(new_node.path)-1 + hn(new_node))  # only here is different from ucs