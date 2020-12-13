import pygame
import math
import numpy as np
from Queue import PriorityQueue

#Pygame window dimension setup
BASE_WIDTH = 600 #Base Window witdh (pixels)
#Grid Dims
ROWS = 100   
COLS = 40
#Adjust for non square grids
FIXED_SIZE = (BASE_WIDTH // COLS)
HEIGHT =  FIXED_SIZE  * ROWS
WIDTH = FIXED_SIZE * COLS

#Create game window
WIN = pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption("A* Path Finding Algorithm")

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
        
        draw()

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

def main(win, width, heigth, my_rows, my_cols):
    #Grid dimensions
    ROWS = my_rows
    COLS = my_cols
    grid = make_grid(ROWS, COLS, width) #Generate grid
    
    #Limit points initialization
    start = None
    end = None 
    
    #Flag States
    run = True

    #Game loop
    while run:
        draw(win, grid, ROWS, COLS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT: #Check if Quit "X" has benn pressed
                run = False
                
            if pygame.mouse.get_pressed()[0]: #Check for mouse left click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, heigth)
                spot = grid[row][col]

                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()
                
                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]: #Check for mouse right click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, heigth)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None 
                if spot == end:
                    end = None 

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    path = algorithm(lambda: draw(win, grid, ROWS, COLS, width), grid, start, end)
                    print("Initial point")
                    print(start.get_pos())
                    print("Final Point")
                    print(end.get_pos())
                    print("Path")
                    print(path)
                    
                
                if event.key == pygame.K_c:
                    start = None
                    end = None 
                    grid = make_grid(ROWS, COLS, width)

    pygame.quit()

    

main(WIN, WIDTH, HEIGHT, ROWS, COLS)