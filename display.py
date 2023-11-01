import pygame

#const
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (70, 70, 70)
BLUE = (0,0,255)
RED = (255,0,0)
GREEN = (0,255,0)
NODERAD = 2
NODETHICKNESS = 0
EDGETHICKNESS = 1

class display:
    def __init__(self, dimensions, caption):
        self.map = pygame.display.set_mode(dimensions)
        pygame.display.set_caption(caption)
        self.map.fill(BLACK)
    
    def drawCircle(self, color, center, radius = NODERAD, thickness = NODETHICKNESS):
        pygame.draw.circle(self.map, color, center, radius, thickness)
    
    def drawLine(self, color, start, end, thickness = EDGETHICKNESS):
        pygame.draw.line(self.map, color, start, end, thickness)
    
    def drawMap(self, obstacles):
        for obs in obstacles:
            self.drawRect(GREY, obs)

    def drawRect(self, color, rect):
        pygame.draw.rect(self.map, color, rect)
    
    def update(self):
        pygame.display.update()