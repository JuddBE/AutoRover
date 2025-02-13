"""
NOT A NODE, JUST A HELPER CLASS.
IMPLEMENTS D* LITE ALGORITHM FOR PATH PLANNING.
"""

from rover.PriorityQueue import PriorityQueue, Priority
from math import sqrt

class Node:
    def __init__(self, id, cost):
        # x and y coord
        self.id = id

        # edge cost
        self.edge_cost = cost

        # g approximation
        self.g = float('inf')

        # rhs value
        self.rhs = float('inf')

        # dict of neighbors (key, edge cost)
        self.neighbors = {}

        # list of connected neighbor edges
        self.conNbrs = []

    def updateNeighbors(self, grid, neighbors):
        for i in neighbors:
            self.neighbors[str(i)] = grid[i[1]][i[0]].edge_cost

    def __str__(self):
        return 'Node: ' + str(self.id) + ' g: ' + str(self.g) + ' rhs: ' + str(self.rhs)

    def __repr__(self):
        return self.__str__() 
    
    
class DStarLite:
    def __init__(self, s_start, s_goal, grid_width, grid_height, trav_map):
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.grid_width = grid_width
        self.grid_height = grid_height

        self.k_m = 0 # accumulation
        self.queue = PriorityQueue()
        self.grid = [[0] * grid_width for _ in range(grid_height)]
        self.map = trav_map

    def heuristic_from_s(self, id):
        x_distance = abs(id[0] - self.s_start[0])
        y_distance = abs(id[1] - self.s_start[1])
        # return max(x_distance, y_distance)
        return sqrt(x_distance**2 + y_distance**2)

    def calculateKey(self, id):
        x_cur = id[0]
        y_cur = id[1]
        return Priority(min(self.grid[y_cur][x_cur].g, self.grid[y_cur][x_cur].rhs) + self.heuristic_from_s(id) + self.k_m, min(self.grid[y_cur][x_cur].g, self.grid[y_cur][x_cur].rhs))

    def initDStar(self):
        # Fill grid with nodes
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                self.grid[i][j] = Node((j, i), self.map[i][j])

        # Assign neighbors to each node
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                neighbors = []

                # Add all possible neighbors and their edge costs.
                if j > 0:
                    neighbors.append((j - 1, i))
                if j < self.grid_width - 1:
                    neighbors.append((j + 1, i))
                if i > 0:
                    neighbors.append((j, i - 1))
                if i < self.grid_height - 1:
                    neighbors.append((j, i + 1))
                if j > 0 and i > 0:
                    neighbors.append((j - 1, i - 1))
                if j < self.grid_width - 1 and i < self.grid_height - 1:
                    neighbors.append((j + 1, i + 1))
                if i > 0 and j < self.grid_width - 1:
                    neighbors.append((j + 1, i - 1))
                if j > 0 and i < self.grid_height - 1:
                    neighbors.append((j - 1, i + 1))
                
                self.grid[i][j].updateNeighbors(self.grid, neighbors)

        # Goal value init
        self.k_m = 0
        self.grid[self.s_goal[1]][self.s_goal[0]].rhs = 0

        # Priority queue init
        self.queue.insert(self.s_goal, Priority(self.heuristic_from_s(self.s_goal), 0))

    def contain(self, u):
        return u in self.queue.vertices_in_heap

    def updateVertex(self, u):
        # convert string back to tuple of coords
        if type(u) != tuple:
            u = tuple(map(int, u[1:-1].split(', ')))

        if self.grid[u[1]][u[0]].g != self.grid[u[1]][u[0]].rhs and self.contain(u):
            # update key of u in priority queue
            self.queue.update(u, self.calculateKey(u))
        elif self.grid[u[1]][u[0]].g != self.grid[u[1]][u[0]].rhs and not self.contain(u):
            # insert state into priority queue
            self.queue.insert(u, self.calculateKey(u))
        elif self.grid[u[1]][u[0]].g == self.grid[u[1]][u[0]].rhs and self.contain(u):
            # Remove u from priority queue
            self.queue.remove(u)

    def computeShortestPath(self):
        x_start = self.s_start[0]
        y_start = self.s_start[1]

        while (self.queue.top_key() < self.calculateKey(self.s_start)) or (self.grid[y_start][x_start].rhs > self.grid[y_start][x_start].g):
            u = self.queue.top()
            k_old = self.queue.top_key()
            k_new = self.calculateKey(u)
            if k_old < k_new:
                self.queue.update(u, k_new)
            elif self.grid[u[1]][u[0]].g > self.grid[u[1]][u[0]].rhs:
                self.grid[u[1]][u[0]].g = self.grid[u[1]][u[0]].rhs
                self.queue.remove(u)
                
                for s in self.grid[u[1]][u[0]].neighbors:
                    s = tuple(map(int, s[1:-1].split(', '))) # convert string back to tuple of coords
                    if s != self.s_goal:
                        self.grid[s[1]][s[0]].rhs = min(self.grid[s[1]][s[0]].rhs, self.grid[s[1]][s[0]].neighbors[str(u)] + self.grid[u[1]][u[0]].rhs)
                        self.updateVertex(s)
            else:
                self.g_old = self.grid[u[1]][u[0]].g
                self.grid[u[1]][u[0]].g = float('inf')
                
                combined = dict(self.grid[u[1]][u[0]].neighbors)
                combined[str(u)] = self.grid[u[1]][u[0]].edge_cost
                for s in combined:
                    s = tuple(map(int, s[1:-1].split(', '))) # convert string back to tuple of coords
                    if self.grid[s[1]][s[0]].rhs == combined[str(s)] + self.g_old:
                        if s != self.s_goal:
                            min_s = float('inf')
                            for s_ in self.grid[s[1]][s[0]].neighbors:
                                s_ = tuple(map(int, s_[1:-1].split(', '))) # convert string back to tuple of coords
                                temp = self.grid[s[1]][s[0]].neighbors[str(s_)] + self.grid[s_[1]][s_[0]].g
                                if min_s > temp:
                                    min_s = temp
                            self.grid[s[1]][s[0]].rhs = min_s
                    self.updateVertex(s)