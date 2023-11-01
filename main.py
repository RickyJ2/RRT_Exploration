import pygame
from display import display, WHITE, BLACK, GREY, BLUE, RED, GREEN
from rrt import RRT, Node

#constants
XDIM = 1000
YDIM = 600
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
# STARTNODE = (XDIM/2.0,YDIM/2.0)
STARTNODE = (50.0, 50.0)
GOALNODE = (510.0, 510.0)
GOALRADIUS = 25
EPSILON = 35

def checkPause():
    for event in pygame.event.get():
        if event.type == pygame.KEYUP:
            raise

#setup
displayMap = display(WINSIZE, 'RRT')
rrt = RRT(STARTNODE, GOALNODE, WINSIZE, GOALRADIUS, EPSILON)
displayMap.drawCircle(GREEN, STARTNODE, 7)
displayMap.drawCircle(GREEN, GOALNODE, GOALRADIUS, 1)
displayMap.drawMap(rrt.obstacle)
displayMap.update()

#run loop
def loop():
    while(not rrt.pathToGoal()):
        node = rrt.expand()
        if node == None:
            continue
        displayMap.drawCircle(WHITE, (node.x, node.y))
        displayMap.drawLine(BLUE, (node.x, node.y), (node.parent.x, node.parent.y))
        displayMap.update()
        checkPause()
    path = rrt.path
    for node in path:
        displayMap.drawCircle(RED, (node.x, node.y), 3)
    displayMap.update()
    pygame.event.clear()
    pygame.event.wait(0)


running = True
pause = False
while running:
    if(not pause):
        try:
            loop()
        except:
            pause = True

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYUP:
            pause = False