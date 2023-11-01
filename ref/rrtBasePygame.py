import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.MapH, self.MapW = self.MapDimensions

        #window setting
        self.map = pygame.display.set_mode((self.MapW, self.MapH))
        pygame.display.set_caption('RRT')
        self.map.fill((255,255,255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        self.grey = (70,70,70)
        self.blue = (0,0,255)
        self.red = (255,0,0)
        self.green = (0,255,0)
        self.black = (0,0,0)
        self.white = (255,255,255)
        
    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad+5, 0)
        pygame.draw.circle(self.map, self.green, self.goal, self.nodeRad+20, 1)
        self.drawObs(obstacles)
    def  drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node, self.nodeRad + 3, 0)
    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while(len(obstaclesList) > 0):
            obs = obstaclesList.pop()
            pygame.draw.rect(self.map, self.grey, obs)
        

class RRTGraph:
    def __init__(self,start,goal, MapDimensions,obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.mapH, self.mapW = MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        
        #initialize tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        #obs
        self.obstacle = []
        self.obsdim = obsdim
        self.obsnum = obsnum
        #path
        self.goalState = None
        self.path = []
        
    def makeRandomRect(self):
        upperCornerX = int(random.uniform(0, self.mapW - self.obsdim))
        upperCornerY = int(random.uniform(0, self.mapH - self.obsdim))
        return(upperCornerX, upperCornerY)
    
    def makeObs(self):
        obs = []
        for i in range(0, self.obsnum):
            rect = None
            startGoalCol = True
            while startGoalCol:
                upper = self.makeRandomRect()
                rect = pygame.Rect(upper, (self.obsdim, self.obsdim))
                if rect.collidepoint(self.start) or rect.collidepoint(self.goal):
                    startGoalCol = True
                else:
                    startGoalCol = False
            obs.append(rect)
        self.obstacle = obs
        return obs

    def addNode(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def removeNode(self, n):
        self.x.pop(n)
        self.y.pop(n)
        
    def addEdge(self, parent, child):
        self.parent.insert(child, parent)
        
    def removeEdge(self, n):
        self.parent.pop(n)
    
    def numberOfNodes(self):
        return len(self.x)
        
    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - (float(y2))) ** 2
        return math.sqrt(px + py)
    
    def sampleEnvir(self):
        x = int(random.uniform(0, self.mapW))
        y = int(random.uniform(0, self.mapH))
        return(x, y)

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear
        
    def isFree(self):
        n = self.numberOfNodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacle.copy()
        while(len(obs) > 0):
            rect = obs.pop()
            if rect.collidepoint(x, y):
                self.removeNode(n)
                return False
        return True
    def crossObstacle(self, x1, y1, x2, y2):
        obs = self.obstacle.copy()
        while(len(obs) > 0):
            rect = obs.pop()
            if rect.clipline(x1, y1, x2, y2):
                return True
            # for i in range(0, 101):
            #     u = i/100
            #     x = x1 * u + x2 * (1-u)
            #     y = y1 * u + y2 * (1-u)
            #     if rect.collidepoint(x, y):
            #         return True
        return False
            
        
    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, y1, x2, y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True
        
    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax :
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)), int(ynear + dmax * math.sin(theta)))
            self.removeNode(nrand)
            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.addNode(nrand, self.goal[0], self.goal[1])
                self.goalState = nrand
                self.goalFlag = True
            else:
                self.addNode(nrand, x, y)
    def pathToGoal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalState)
            newPos = self.parent[self.goalState]
            while(newPos != 0):
                self.path.append(newPos)
                newPos = self.parent[newPos]
            self.path.append(0)
        return self.goalFlag
    
    def getPathCoords(self):
        pathCoord = []
        for node in self.path:
            x,y = (self.x[node], self.y[node])
            pathCoord.append((x,y))
        return pathCoord
    
    def bias(self, ngoal):
        n = self.numberOfNodes()
        self.addNode(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent
    
    def expand(self):
        n = self.numberOfNodes()
        x, y = self.sampleEnvir()
        self.addNode(n, x, y)
        if self.isFree():
            xnear = self.nearest(n)
            self.step(xnear, n)
            self.connect(xnear, n)
        return self.x, self.y, self.parent
    def const(self):
        pass