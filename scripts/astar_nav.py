#!/usr/bin/env python

import pygame
import math
import rospy
import roslib
import time
import numpy as np
from Queue import PriorityQueue
from nav_msgs.msg import *
from std_msgs.msg import *


#Pygame window dimension setup
BASE_WIDTH = 1000 #Base Window witdh (pixels)

#Window colors
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
CYAN = (0, 255, 255)

class Spot: #Object for each of the cells of the grid
    
    def __init__(self, row, col, width, total_rows, total_cols):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.width = width
        self.color = WHITE
        self.neighbors = []
        self.total_rows = total_rows
        self.total_cols = total_cols
        
    #Retrieve info functions
    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED 
    
    def is_open(self):
        return self.color == GREEN
    
    def is_barrier(self):
        return self.color == BLACK 
    
    def is_start(self):
        return self.color == ORANGE
    
    def is_end(self):
        return self.color == CYAN

    #Define state functions       
    def reset(self):
        self.color = WHITE

    def make_closed(self):
        self.color = RED 
    
    def make_open(self):
        self.color = GREEN
    
    def make_barrier(self):
        self.color = BLACK 
    
    def make_start(self):
        self.color = ORANGE
    
    def make_end(self):
        self.color = CYAN
    
    def make_path(self):
        self.color = PURPLE

    #Plot on window
    def draw(self,win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self,grid):
        #Check all neighbors of current the spot
        self.neighbors = []
        if self.row < self.total_cols - 1 and not grid[self.row + 1][self.col].is_barrier(): #Check Down spot
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0  and not grid[self.row - 1][self.col].is_barrier(): #Check Up spot
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): #Check Right spot
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0  and not grid[self.row][self.col - 1].is_barrier(): #Check Left spot
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other): 
        return False
    
#Heuristic calculation function (Manhatan distance)
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs (y1 - y2)

def reconstruct_path(came_from, current, draw):
    path = np.array([current.get_pos()])
    while current in came_from:
        current = came_from[current]
        current.make_path()
        path = np.append([current.get_pos()], path, 0)
    draw()
    return path

#A* Algorithm
def algorithm(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            path = reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            draw()
            return path

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        
        

        if current != start:
            current.make_closed()
    
    return False

#Create empty grid 
def make_grid(rows, cols, width):
    grid = []
    gap = width // cols
    for i in range(cols):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows, cols)
            grid[i].append(spot)
    return grid

#Draw grid on a window
def draw_grid(win, rows, cols, width):
    gap = width // cols
    heigth = gap * rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(cols): 
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, heigth))

#General draw function (Grid + Spots)
def draw(win, grid, rows, cols, width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)
    
    draw_grid(win, rows, cols, width)
    pygame.display.update()

#Transform clicked position (x,y) to row and column on the grid
def get_clicked_pos(pos, rows, heigth):
    gap = heigth // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col

def astar(win, width, heigth, my_rows, my_cols, matrix, start_point, end_point):
    #Grid dimensions
    ROWS = my_rows
    COLS = my_cols
    grid = make_grid(ROWS, COLS, width) #Generate grid
    
    for row in range(ROWS):
        for col in range(COLS):
            #print("row: " + str(row))
            #print("col: " + str(col))
            val = matrix[row][col]
            #print("val: " + str(val))
            if val > 50:
                grid[col][row].make_barrier()


    #Limit points initialization
    start = grid[start_point[0]][start_point[1]] 
    start.make_start()
    end = grid[end_point[0]][end_point[1]] 
    end.make_end()


    #Flag States
    run = True

    #Game loop
    while run:
        draw(win, grid, ROWS, COLS, width)
        for event in pygame.event.get():    

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    path = algorithm(lambda: draw(win, grid, ROWS, COLS, width), grid, start, end)
                    run = False

    print("Initial point")
    print(start.get_pos())
    print("Final Point")
    print(end.get_pos())
    return path

#Recieve map data from map server topic
def callback(data):
    rospy.loginfo("Map recieved")
    cost_map.data = data.data
    cost_map.info = data.info
    cost_map.header = data.header

def angle_between(p0,p1,p2):
    v0 = np.array(p1) - np.array(p0)
    v1 = np.array(p2) - np.array(p0)

    angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    return angle

def xy2traj(dots):
    """
    From xy coordinates generates a complete trajectory
    Takes x,y coordinates of a trajectory and calculates x,y,theta coordinates
    with intermidiate points that assure twists of 90
    
    Parameters
    ----------
    dots : list of [x,y]
        Dots [[x_0,y_0],[x_1,y_1],...]
    Motion : control.Motion
        Class where the control and motion is settled
    Returns
    -------
    list of [x,y,theta]
        Complete trajectory
    """
    traj = []
    last_dot = 0 
    last_x = 0
    last_y = 0
    for count, dot in enumerate(dots):
        x = dot[0]
        y = dot[1]
        if (count == 0) :
            theta = 90
            traj.append([x,y,theta])           #Radians to deg
        else:
            theta = angle_between(last_dot,[last_x+1,last_y],dot)
            traj.append([last_x,last_y,np.rad2deg(theta)])
            traj.append([x,y,np.rad2deg(theta)])
        last_dot = dot
        last_theta = theta
        last_x = x
        last_y = y
    return traj


if __name__ == "__main__":

    #Node initialization
    rospy.init_node("A*_path", anonymous = False)
    rate = rospy.Rate(20) # 50 Hz ROS

    #Map message
    cost_map = OccupancyGrid()

    #Map suscriber
    rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid , callback)

    #Make a pause before recieving map
    print("Retrieving Map")
    time.sleep(1)
    print("Map Data: ")
    print(cost_map.data)
    print("Map Info: ")
    print(cost_map.info)

    #Map dimmensions
    map_width = cost_map.info.width
    map_height = cost_map.info.height
    map_origin = np.array([cost_map.info.origin.position.x,cost_map.info.origin.position.y])


    #Convert map from vector to matrix
    map_vect = cost_map.data
    map_grid = np.reshape(map_vect, (map_height, map_width))
    map_grid = map_grid[::-1,:]
    print("Converted Map:")
    print(map_grid)
    print("Converted map shape: " + str(map_grid.shape))

    #GRAPHICAL PARAMS
    #Grid Dims
    ROWS = map_height  
    COLS = map_width
    #Adjust for non square grids
    FIXED_SIZE = (BASE_WIDTH // COLS)
    HEIGHT =  FIXED_SIZE  * ROWS
    WIDTH = FIXED_SIZE * COLS

    #Create game window
    WIN = pygame.display.set_mode((WIDTH,HEIGHT))
    pygame.display.set_caption("A* Path Finding Algorithm")

    #PHYSICAL PARAMS
    map_resolution = cost_map.info.resolution
    real_width = map_resolution * map_width #Widht in meters
    real_height = map_resolution * map_height #Widht in meters

    origin = np.array([0,0])
    origin_discrete = (origin -[map_origin[0],-(map_height*map_resolution)-map_origin[1]])//map_resolution #Respecto origen mapa
    print("Origen en matriz :" + str(origin_discrete))

    goal = np.array([-1,3.5])
    goal_discrete = (goal -[map_origin[0],-(map_height*map_resolution)-map_origin[1]])//map_resolution #Respecto origen mapa
    print("Origen en matriz :" + str(origin_discrete))

    path = astar(WIN, WIDTH, HEIGHT, ROWS, COLS, map_grid, np.uint8(origin_discrete), np.uint8(goal_discrete))
    print("Path")
    print(path)

    path[:,1]=-path[:,1]
    path_mundo = np.array(path)*map_resolution+np.array([map_origin[0],map_height*map_resolution+map_origin[1]])
    print("Path Mundo")
    print(path_mundo)

    temp = xy2traj(path_mundo)
    trajectory = np.array(temp)
    print("Path Mundo + Orientacion")
    print(trajectory)

    while(not rospy.is_shutdown()):
        rate.sleep() #Wait for ROS node cycle

    pygame.quit()

    