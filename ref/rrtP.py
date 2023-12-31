import pygame
from rrtBasePygame import RRTGraph, RRTMap
import time

def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal=(510, 510)
    obsdim = 30
    obsnum = 50
    iteration = 0
    t1=0

    t1 = time.time()
    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacle = graph.makeObs()
    map.drawMap(obstacle)

    while(not graph.pathToGoal()):
        elapsed = time.time() - t1
        t1 = time.time()
        if elapsed > 10:
            raise

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad + 2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
        else :
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad + 2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)

        if iteration % 5 == 0:
            pygame.display.update()
            
        iteration += 1
    
    map.drawPath(graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

running = True
result = False
while running:
    while not result:
        try:
            main()
            result = True
        except:
            result = False  
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False