from copy import deepcopy

class Node:
    def __init__(self, start_pos):
        self.pos = start_pos
        self.path = [start_pos]
    
    def reach_next_pos(self, next_pos):
        new_node = deepcopy(self)
        new_node.pos = next_pos
        new_node.path.append(next_pos)
        
        return new_node

class ucs_Node:
    def __init__(self, start_pos):
        self.pos = start_pos
        self.path = [start_pos]
        self.totalCost = 0
    
    def reach_next_pos(self, next_pos, cost):
        new_node = deepcopy(self)
        new_node.pos = next_pos
        new_node.path.append(next_pos)
        new_node.totalCost = self.totalCost + cost
        
        return new_node

class astar_Node:
    def __init__(self, start_pos, goal_pos, heuristic):
        self.pos = start_pos
        self.goal_pos = goal_pos
        self.heuristic = heuristic
        self.path = [start_pos]
        self.cost_till_now = 0
        self.estimated_total = self.cost_till_now + self.hn()
        
    def hn(self):
        if self.heuristic == 'manhattan':
            return abs(self.pos[0] - self.goal_pos[0]) + abs(self.pos[1] - self.goal_pos[1])
        elif self.heuristic == 'euclidean':
            return ((self.pos[0] - self.goal_pos[0])**2 + (self.pos[1] - self.goal_pos[1])**2)**0.5
        else:
            print("Invalid heuristic")

    def reach_next_pos(self, next_pos, cost):
        new_node = deepcopy(self)
        new_node.pos = next_pos
        new_node.path.append(next_pos)
        new_node.cost_till_now = self.cost_till_now + cost
        new_node.estimated_total = new_node.cost_till_now + new_node.hn()
        
        return new_node

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


def goal_test(node, stop_pos):
    return node.pos == stop_pos

def bfs(map, start_pos, stop_pos):
    rows, cols = len(map), len(map[0])
    start_pos = (start_pos[0]-1, start_pos[1]-1)
    stop_pos = (stop_pos[0]-1, stop_pos[1]-1)
    
    node = Node(start_pos)
    fringe = [node]
    cnt = 0
    while True:
        cnt += 1
        
        if fringe == []:
            return False
        node = fringe.pop(0)
        if goal_test(node, stop_pos):
            print("Found path in {} steps".format(cnt))
            return node.path
        # expand fringe
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_pos = (node.pos[0] + i, node.pos[1] + j)
            if new_pos in node.path:
                continue
            if new_pos[0] < 0 or new_pos[0] >= rows or new_pos[1] < 0 or new_pos[1] >= cols:
                continue
            if map[new_pos[0]][new_pos[1]] == 0:
                continue
            fringe.append(node.reach_next_pos(new_pos))

def ucs(map, start_pos, stop_pos):
    rows, cols = len(map), len(map[0])
    start_pos = (start_pos[0]-1, start_pos[1]-1)
    stop_pos = (stop_pos[0]-1, stop_pos[1]-1)
    
    node = ucs_Node(start_pos)
    fringe = PriorityQueue()
    fringe.put(node, 0)
    cnt = 0
    while True:
        cnt += 1
        
        if fringe.empty():
            return False
        node = fringe.get()
        if goal_test(node, stop_pos):
            print("Found path in {} steps".format(cnt))
            return node.path
        # expand fringe
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_pos = (node.pos[0] + i, node.pos[1] + j)
            
            if new_pos in node.path:
                continue
            if new_pos[0] < 0 or new_pos[0] >= rows or new_pos[1] < 0 or new_pos[1] >= cols:
                continue
            if map[new_pos[0]][new_pos[1]] == 0:
                continue
            delta = map[new_pos[0]][new_pos[1]] - map[i][j]
            cost = delta + 1 if delta > 0 else 1
            new_node = node.reach_next_pos(new_pos, cost)
            # print(new_node)
            fringe.put(new_node, new_node.totalCost)

def astar(map, start_pos, stop_pos, heuristic):
    rows, cols = len(map), len(map[0])
    start_pos = (start_pos[0]-1, start_pos[1]-1)
    stop_pos = (stop_pos[0]-1, stop_pos[1]-1)
    
    node = astar_Node(start_pos, stop_pos, heuristic)
    fringe = PriorityQueue()
    fringe.put(node, 0)
    cnt = 0
    while True:
        cnt += 1
        
        if fringe.empty():
            return False
        node = fringe.get()
        if goal_test(node, stop_pos):
            print("Found path in {} steps".format(cnt))
            return node.path
        # expand fringe
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_pos = (node.pos[0] + i, node.pos[1] + j)
            
            if new_pos in node.path:
                continue
            if new_pos[0] < 0 or new_pos[0] >= rows or new_pos[1] < 0 or new_pos[1] >= cols:
                continue
            if map[new_pos[0]][new_pos[1]] == 0:
                continue
            delta = map[new_pos[0]][new_pos[1]] - map[i][j]
            cost = delta + 1 if delta > 0 else 1
            new_node = node.reach_next_pos(new_pos, cost)
            # print(new_node)
            fringe.put(new_node, new_node.estimated_total)