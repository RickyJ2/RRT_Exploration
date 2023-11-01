import random, math
import pygame

class Node:
    def __init__(self,x,y, parent = None):
        self.x = x
        self.y = y
        self.parent = parent
    def setParent(self, parent):
        self.parent = parent
    def distance(self, node):
        return math.sqrt((float(self.x) - float(node.x))**2 + (float(self.y) - float(node.y))**2)

class Obstacle:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)
    def collidepoint(self, point):
        return self.rect.collidepoint(point)
    def clipline(self, start, end):
        return self.rect.clipline(start, end)

def makeRandomRect(mapW, mapH, obsdim):
    upperCornerX = int(random.uniform(0, mapW - obsdim))
    upperCornerY = int(random.uniform(0, mapH - obsdim))
    return(upperCornerX, upperCornerY)

def makeRandomObs(mapW, mapH, obdDim, obsNum, start, goal):
    obs = []
    for i in range(0, obsNum):
        rect = None
        startGoalCol = True
        while startGoalCol:
            upper = makeRandomRect(mapW, mapH, obdDim)
            rect = Obstacle(upper[0], upper[1], obdDim, obdDim)
            if rect.collidepoint(start) or rect.collidepoint(goal):
                startGoalCol = True
            else:
                startGoalCol = False
        obs.append(rect)
    return obs

class RRT:
    def __init__(self, start, goal, MapDimensions, GoalRadius, Epsilon):
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.mapW, self.mapH = MapDimensions
        self.nodes = []
        self.goalRadius = GoalRadius
        self.epsilon = Epsilon

        self.goalState = None
        self.path = []
        self.nodes.append(Node(start[0], start[1], 0))
        self.obstacle = makeRandomObs(self.mapW, self.mapH, 30, 50, self.start, self.goal)

    def addNode(self, n):
        self.nodes.append(n)
    
    def numberOfNodes(self):
        return len(self.nodes)
    
    def sampleEnvir(self):
        findNode = False
        x, y = 0,0
        while not findNode:
            x = random.random()*self.mapW
            y = random.random()*self.mapH
            if self.isFree(Node(x,y)):
                findNode = True
        return Node(x,y)
    
    def isFree(self, node):
        for obs in self.obstacle:
            if obs.collidepoint((node.x, node.y)):
                return False
        return True
    
    def crossObstacle(self, n1, n2):
        for obs in self.obstacle:
            if obs.clipline((n1.x, n1.y), (n2.x, n2.y)):
                return True
        return False

    def nearest(self, n):
        nn = self.nodes[0]
        for p in self.nodes:
            if p.distance(n) < nn.distance(n):
                nn = p
        return nn
    
    def step_from_to(self, p1, p2):
        if p1.distance(p2) < self.epsilon:
            return p2
        else:
            theta = math.atan2(p2.y-p1.y,p2.x-p1.x)
            return Node(p1.x + self.epsilon*math.cos(theta), p1.y + self.epsilon*math.sin(theta))

    def isGoal(self, n):
        if n.distance(Node(self.goal[0], self.goal[1])) < self.goalRadius:
            return True
        else:
            return False

    def expand(self):
        rand = self.sampleEnvir()
        nn = self.nearest(rand)
        newnode = self.step_from_to(nn,rand)
        if(self.crossObstacle(nn, newnode)):
            return None
        newnode.setParent(nn)
        self.addNode(newnode)
        if self.isGoal(newnode):
            self.goalState = newnode
            self.goalFlag = True
        return newnode
    
    def pathToGoal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalState)
            newPos = self.goalState.parent
            while(newPos != 0):
                self.path.append(newPos)
                newPos = newPos.parent
        return self.goalFlag