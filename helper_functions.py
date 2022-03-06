# from multiprocessing import parent_process
import numpy as np
import matplotlib.pyplot as plt
import pygame 
import heapq as pq
import copy as copy
import matplotlib.pyplot as plt

#define global map paramaters

height ,width = 250 , 400

#colors to descibe map
black = (220 ,220,220)
gray = (0,0,0)
black =(254,254,254)
blue = (0,20,108)
red = (255 ,0,0)
cyan = (255,0,0)
# cyan = (136 , 250 , 196)

# Define the obstacles
def defineObstacles():
    ob1 = Hexagon(200 , 100 ,70)
    ob2 = Circle(300 , 185 , 40)
    ob3 = Poly((36, 185) , (105,100) , (80,180) , (115,210))
    return [ob2 , ob1 , ob3]

#Creating a hexagon class which creates a hexagon using center co-ordinates and distance between two parallel lines 
class Hexagon:
    def __init__(self,x_center,y_center,dx):
        self.type = 'polygon'
        self.cx ,self.cy = x_center,y_center
        a = dx/np.sqrt(2)
        dy = np.sqrt(a**2 - (dx/2)**2)

        self.point1 , self.point4 = (x_center , y_center + dy) , (x_center , y_center - dy)
        self.point2 , self.point3 = (x_center - dx/2 , y_center + dy/2) , (x_center - dx/2 , y_center - dy/2)
        self.point5 , self.point6 = (x_center + dx/2 , y_center -dy/2) , (x_center + dx/2 , y_center + dy/2)

        self.points = np.array([self.point1 , self.point2 , self.point3 , self.point4 , self.point5 , self.point6])

    def inObstacle(self,x,y):
        #check if a point is inside the hexagon by checking where it lies w.r.t all the line segments made by the hexagon 
        #here mij  = slope of line containing points i and j  and cij = intercept of line containing i and j
        m12 , m34 , m45 , m61 = 0.5 , -0.5 , 0.5,-0.5  #slope
        c12 , c34 , c45 , c61 =36 , 165 , -35 , 235,
        c56 , c23 =  235 , 165  #intercept

        #condition on where a point should lie w.r.t to a line defined by twon points of hexagon
        line1 = (y - m12*x - c12) < 0 
        line2 = (x - c23) > 0
        line3 = (y - m34*x - c34) > 0
        line4 = (y - m45*x - c45) > 0
        line5 = (x - c56) < 0
        line6 = (y-m61*x - c61) < 0

        inside = line1 and line2 and line3 and line4 and line5 and line6
        return inside


class Circle:
    def __init__(self , x_center , y_center , radius):
        self.type = 'circle'
        self.x = x_center
        self.y = y_center
        self.radius = radius

    def inObstacle(self , x,y):
        l = (x - self.x)**2 + (y - self.y)**2 - self.radius**2
        if l < 0:
            return True
        else :
            return False

class Poly():
    def __init__ (self , *args):
        self.type = 'polygon'
        self.points = np.array(args)

    def inObstacle(self , x, y):
        # calculated slope and intercept using two points on the line
        m12 , m23 , m34 , m41 = -1.24 , -3.2 , 0.85 , 0.32
        c12 ,c23 , c34 , c41 = 230 , 439 , 112 , 173


        line1 = (y - m12*x - c12) >0
        line2 = (y - m23*x - c23) < 0 
        line_mid = (y - (-0.1)*x - 189) < 0 
        line_right = ( y - (-0.1)*x - 189) >= 0
        line3 = (y - m34*x - c34) > 0 
        line4 = (y - m41*x - c41) < 0
        inside = line1 and line2 and line_mid or line_right and line4 and line3
        # inside = l1 and l2 and l3 and l4 and lmid or lright
        return inside

class Node:
    def __init__(self , x_center, y_center):
        self.x = x_center
        self.y = y_center
        self.parent = None
        self.neighbours = {}
        self.dor = float('inf')

    def __lt__(self , other):
        return self.dor < other.dor

def flipme(map):
    pts = copy.deepcopy(map)
    for i in range(len(pts)):
        pts[i][1] = height - pts[i][1]
    
    return pts

class Map:
    def __init__(self, obstacles , gridDisplay):
        self.obstacles = obstacles
        self.gridDisplay = gridDisplay
        self.map_generation()


    def map_generation(self):
        
        self.gridDisplay.fill(gray)
        for obstacle in self.obstacles:
            if obstacle.type == 'circle':
                #change height  ->  witdh
                pygame.draw.circle(self.gridDisplay , blue , [obstacle.x , height - obstacle.y] , obstacle.radius)
            if obstacle.type == 'polygon':
                # change flip -> !flip
                pygame.draw.polygon(self.gridDisplay, blue, flipme(obstacle.points))

    def withinObstacle(self,x , y ):
        states = []
        for obstacle in self.obstacles:
            states.append(obstacle.inObstacle(x,y))
        return sum(states) > 0

    def drawMap(self):
        grid = np.zeros((height , width))
        # grid = np.array([[0 for j in range(height)] for i in range(width)])
        # grid = np.array(grid)
        for i in range(width):
            for j in range(height):
                grid[i][j] = int(self.withinObstacle(i,j))*255
        plt.imshow(np.flipud(grid.T))
        plt.show()





    















 



