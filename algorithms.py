from __future__ import print_function
from heapq import * #Hint: Use heappop and heappush

ACTIONS = [(0,-1),(-1,0),(0,1),(1,0)]

class Agent:
    def __init__(self, grid, start, goal, type):
        self.grid = grid
        self.start = start 
        self.grid.nodes[start].start = True
        self.goal = goal
        self.grid.nodes[goal].goal = True
        self.final_cost = 0 #Make sure to update this value at the end of UCS and Astar
        self.search(type)
    def heuristic(self, start, end):
        return abs(start[0]-end[0])+abs(start[1]-end[1])
    def search(self, type):
        self.finished = False
        self.failed = False
        self.type = type
        self.previous = {}
        if self.type == "dfs":
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "ucs":
            self.frontier = [(0, self.start)]
            self.explored = []
        elif self.type == "astar":
            self.frontier = [(self.heuristic(self.start, self.goal), 0,self.start)]
            self.explored = []
    def show_result(self):
        current = self.goal
        while not current == self.start:
            current = self.previous[current]
            self.grid.nodes[current].in_path = True #This turns the color of the node to red
    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()
    #DFS
    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop()
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        for node in children:
            if node in self.explored or node in self.frontier:
                continue
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                if not self.grid.nodes[node].puddle:
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                    else:
                        self.frontier.append(node)
                        self.grid.nodes[node].frontier = True

    #Implement BFS here
    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop(0)
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        for node in children:
            if node in self.explored or node in self.frontier:
                continue
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                if not self.grid.nodes[node].puddle:
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                    else:
                        self.frontier.append(node)
                        self.grid.nodes[node].frontier = True
    #Implement UCS here
    def ucs_step(self):
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        temp = heappop(self.frontier)
        currentCost = temp[0]
        current = temp[1]
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        for node in children:
            if node in self.explored or any(node in i for i in self.frontier):
                continue
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                if not self.grid.nodes[node].puddle:
                    cost = self.grid.nodes[node].cost()+currentCost
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        self.final_cost = cost
                    else:
                        heappush(self.frontier, (cost,node))
                        self.grid.nodes[node].frontier = True
    #Implement Astar here
    def astar_step(self):
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        temp = heappop(self.frontier)
        currentCost = temp[1]
        current = temp[2]
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        for node in children:
            if node in self.explored or any(node in i for i in self.frontier):
                continue
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                if not self.grid.nodes[node].puddle:
                    cost = self.grid.nodes[node].cost()+currentCost
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        self.final_cost = cost
                    else:
                        heappush(self.frontier, (cost+self.heuristic(node,self.goal),cost,node))
                        self.grid.nodes[node].frontier = True

