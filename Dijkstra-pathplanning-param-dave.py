import numpy as np
from helper_functions import *
import matplotlib.pyplot as plt
from tqdm import tqdm
import heapq as pq
import time
import os
import glob
import cv2 as cv
from datetime import datetime


#create grid

grid = np.zeros((400 , 250) , dtype= np.uint)


#creata a function to get the input
def getsource():
    src = []
    print("Enter starting point")
    for i in range(2):
        source = int(input())
        src.append(source)
    
    if src[0] > 450 or src[1] > 250:
        print("Please enter number withing the grid")
        return getsource()
    return src


#create a function to input the output
def getdestination():
    print("Enter the Goal Point")
    goal = []
    for i in range(2):
        gl = int(input())
        goal.append(gl)
   
    if goal[0] > 450 or goal[1] >250:
        print("Please enter number withing the grid")
        return getdestination()
    return goal 

#moves with their costs
def moveleft(node , cost):
    node[0] = node[0] - 1
    node[1] = node[1]
    cost = cost + 1
    return node , cost

def moveright(node , cost):
    node[0] = node[0] + 1
    node[1] = node[1]
    cost = cost + 1
    return node , cost

def moveup(node , cost):
    node[0] = node[0]
    node[1] = node[1] + 1
    cost = cost + 1
    return node , cost

def movedown(node , cost):
    node[0] = node[0]
    node[1] = node[1] - 1
    cost = cost+1
    return node , cost 

def moveupright(node , cost):
    node[0] = node[0] + 1
    node[1] = node[1] + 1
    cost = cost + 1.4

def moveupleft(node , cost):
    node[0] = node[0] - 1
    node[1] = node[1] + 1
    cost = cost + 1

def movedownright(node , cost):
    node[0] = node[0] + 1
    node[1] = node[1] - 1
    cost = cost + 1.4

def movedownleft(node , cost):
    node[0] = node[0] - 1
    node[1] = node[1] - 1
    cost = cost + 1.4

#class for main algorithm 
class dstar:

    def __init__(self , map , start ,end):
        self.map = map
        self.start = start
        self.end = end
        self.visited = {}
        self.directions = [ [0,1] , [1,1] , [1,0] , [1,-1] , [0,-1]  , [-1,-1] , [-1,0] , [ -1,1]  ] 

        if self.map.withinObstacle(self.start.x , self.start.y) and self.map.withinObstacle(self.end.x , self.end.y) :

            print("Enter a start and point within the grid and not inside the obstacle")
            return
        if (0<= start.x< width) and (0<= start.y< height) and (0 <= end.x < width) and (0 <=end.y < height):
           print("Valid intial and goal positions")

        self.frame_counter = 0
    def getNieghbours(self , currNode):
        x , y = currNode.x , currNode.y 

        nieghbour = {}
        for direction in self.directions:
            x_new = x + direction[0]
            y_new = y + direction[1]

            if(0 <= x_new < width) and (0 <= y_new < height) and (not self.map.withinObstacle (x_new , y_new)):
                nieghbour[Node(x_new , y_new)] = 1
        return nieghbour
    
    def Path(self):
        visited = {}
        priorityQueue = []

        pq.heappush(priorityQueue , (self.start.dor ,self.start))
        while len(priorityQueue):
            #change
            node = pq.heappop(priorityQueue)[1]
            # if node we are looking for is the end goal
            if node.x == self.end.x and node.y == self.end.y:
                return True

            
            if tuple([node.x , node.y]) in visited:
                continue
            # put the current node in visited node
            visited[tuple([node.x , node.y])] = True

            node.neighbours = self.getNieghbours(node)

            for neb , newD in node.neighbours.items():
                neb.dor = node.dor + newD
                neb.parent = node
                pq.heappush(priorityQueue,(neb.dor ,neb ))
        
        return False

    def save(self, i , freq = 500):
        if i%freq == 0:
            filename = "./outputs/image" + str(i).zfill(6) + ".png"
            pygame.image.save(self.map.gridDisplay , filename)



    def visualize(self):
        grid = np.zeros((400 , 250) , dtype= np.uint)

        priorityQueue = []

        pq.heappush(priorityQueue , (self.start.dor , self.start))
        end = False
        while len(priorityQueue):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    end = True
            
            if end:
                break

            node = pq.heappop(priorityQueue)[1]

            if node.x == self.end.x and node.y == self.end.y:
                grid = self.backTrack(node , grid)
                return True , grid

            if tuple([node.x , node.y]) in self.visited:
                continue

            self.visited[tuple([node.x , node.y])] = True
            for neb , newd in node.neighbours.items():
                nx , ny = neb.x , neb.y

                if((nx-3 < self.start.x < nx +3 ) and ( ny-3 < self.start.y < ny +3)) or ((nx -3 < self.end.x < nx + 3  ) and (ny - 3 <self.end.y < ny + 3) ):
                    pygame.draw.rect(self.map.gridDisplay , black , [nx , height - ny , 1, 1])
                    pygame.display.update()
                else:
                    pygame.draw.rect(self.map.gridDisplay , cyan , [nx , height-ny , 1,1 ])
                    pygame.display.update()

                self.save(self.frame_counter , 2500) 
                self.frame_counter += 1

                neb.dor = node.dor + newd
                neb.parent = node

                pq.heappush(priorityQueue , (neb.dor , neb))


        return False , grid
    #function for backtrack
    def backTrack(self,child , grid):
        clock = pygame.time.Clock()
        while child != None:
            grid[child.x][child.y] = 1
            pygame.draw.rect(self.map.gridDisplay , black , [child.x , height - child.y, 2,2 ])
            pygame.display.update()
            self.save(self.frame_counter , freq = 1)
            self.frame_counter += 1
            clock.tick(500)

            child = child.parent
        return grid
#saving and visualizing the path
def foldceek(save_path):
    if not os.path.exists(save_path):
        os.makedirs(save_path)

#saving and visualizing the path
def remove_file(file):
    try:
        os.remove(file)
    except OSError:
        print("Error: %s : %s" % (file , OSError) )


if __name__ == "__main__":

    start_time = datetime.now()
    
    src = getsource()
    dest = getdestination()
#taking the input
    x1 = src[0]
    y1 = src[1]

    x2 = dest[0]
    y2 = dest[1]

    foldceek("outputs")

    start = Node(x1 , y1)
    start.dor = 0
    end = Node(x2 , y2)

    pygame.init()
    gridDisplay = pygame.display.set_mode((width , height))
    pygame.display.set_caption("Project 2")
    obstacles = defineObstacles()
    #map initialization
    map = Map(obstacles , gridDisplay)
    #mai algorithm initialization
    solution = dstar(map , start , end)

    #conditions where there is no solution possible
    if not solution.Path():
        print("can't find a solution")
        exit()
    flag , path_grid  = solution.visualize()
    #displayinhg the video
    if not flag:
        print("Visualization not possible")
        images = sorted(glob("outputs/*.png"))
        for i in tqdm(range(len(images))):
            image = cv.imread(images[i])
    
            if i == 0:
                h,w,_ = image.shape
                videowriter = cv.VideoWriter('output_video.mp4' , cv.VideoWriter_fourcc(*'DIVX'), 30, (w,h))
                
            remove_file(image[i])
        videowriter.release()

    remove_file('outputs')
    time.sleep(5)
    pygame.quit()
    
    end_time = datetime.now()
    print('Duration: {}'.format(end_time - start_time))
    exit()



        



                






        
                    




