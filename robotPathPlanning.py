# -*- coding: utf-8 -*-
"""
@author: Sainandan Tummalapalli
"""

import sys
import pygame
import cv2
import os

os.environ["SDL_VIDEO_CENTERED"] = "1" #Making sure the window is in the center of the screen
pygame.init()

screen = pygame.display.set_mode((440,280)) #assign size of window
screen.fill([255,255,255]) 

myFont  = pygame.font.SysFont("arial", 15, bold=True, italic=False) #assigning font format and size
pygame.display.set_caption("Robot Path Planning Project - Intelligent Systems") 

srcpath = os.getcwd()
srcpath+= "//testgrid.png"
grid   = pygame.image.load(srcpath).convert_alpha() #load the image, convert pixel by pixel from image to window

adjacencyPath = [(1,0),(-1,0),(0,1),(0,-1)] # for generating next possible steps

class PathPlanning:
    def __init__(obj):
        obj.clock = pygame.time.Clock()
        obj.initialize()
        
    def initialize(obj):
        obj.state = "s"  #current state as start state
        obj.screen=screen
        obj.cellSize = (10,10)
        
        obj.start = None
        obj.goal = None
        obj.current = None
        
        obj.obstacles = set() #store all obstacles
        obj.addInitialObstacles()
        
        # Initial Values
        obj.addObstacles = False
        obj.delObstacles = False
        
        obj.hValue = {} #heuristic function value
        obj.gValue = {} #cost from first step to current step
        obj.fValue = {} #fvalue = gValue + hValue
        
        obj.isSolved = False
        obj.solution = []
        obj.pathind = 0
        obj.next = None
        obj.closedset = set()
        obj.openset = set()
        obj.pathTrace = {} # store path taken
        
        obj.startTime = 0.0
        obj.endTime   = 0.0
    
    def addInitialObstacles(obj): #creating boundaries
        for i in range(-2,29):
            obj.obstacles.update(set(((-1,i),)))
            obj.obstacles.update(set((( 0,i),)))
            obj.obstacles.update(set(((44,i),)))
            obj.obstacles.update(set(((45,i),)))
        for j in range(-1,45):
            obj.obstacles.update(set(((j,-1),)))
            obj.obstacles.update(set(((j, 0),)))
            obj.obstacles.update(set(((j,28),)))
            obj.obstacles.update(set(((j,29),)))
        
        file = os.getcwd()
        file = file + "\\testimg2.png"
        img = cv2.imread(file,0)#load obstacles
        for i in range(22, 260):
            for j in range(22, 420):
                if (img[i][j] < 128):
                    obj.obstacles.update(set(((j//obj.cellSize[0],i//obj.cellSize[1]),)))
        
    def manageStates(obj):
        obj.mousePosition = pygame.mouse.get_pos() #get position of mouse pointer
        for events in pygame.event.get():
            if events.type == pygame.MOUSEBUTTONDOWN:
                if pygame.mouse.get_pressed()[0]: #get which button was clicked
                    obj.mousePosition = pygame.mouse.get_pos()
                    if (20 < obj.mousePosition[0] < 425) and (20 < obj.mousePosition[1] < 265):
                        if obj.state == "o":#state-obstacle
                            obj.addObstacles = True
                        elif obj.state == "s":#state - start
                            obj.start = (obj.mousePosition[0]//obj.cellSize[0],obj.mousePosition[1]//obj.cellSize[1])
                            obj.gValue[obj.start] = 0
                            obj.closedset.update(set((obj.start,)))
                            obj.state = "g"
                        elif obj.state == "g":#state-goal
                            obj.goal = (obj.mousePosition[0]//obj.cellSize[0],obj.mousePosition[1]//obj.cellSize[1])
                            if obj.goal == obj.start:
                                print("Start state is the goal state")
                                obj.state = "d"#state-completed/done
                            else:
                                obj.state = "o"
                                obj.hValue[obj.start] = obj.calcDistance(obj.start,obj.goal)
                    if obj.state == "o":
                        if (10 < obj.mousePosition[0] < 255) and (0 < obj.mousePosition[1] < 17):
                            obj.state = "r"#state-run,start algorithm
                            obj.openset = obj.options()
                            obj.startTime = pygame.time.get_ticks()
                            obj.current = obj.start
                    if obj.state == "d":
                        if (10 < obj.mousePosition[0] < 135) and (0 < obj.mousePosition[1] < 15):
                            obj.initialize()
                        elif (140 < obj.mousePosition[0] < 255) and (0 < obj.mousePosition[1] < 15):
                            obj.reset()
                elif pygame.mouse.get_pressed()[2] and obj.state == "o":
                    obj.delObstacles = True
            
            if events.type == pygame.MOUSEBUTTONUP:
                if not pygame.mouse.get_pressed()[0]:
                    obj.addObstacles  = False
                if not pygame.mouse.get_pressed()[2]:
                    obj.delObstacles  = False
            
            if events.type == pygame.KEYDOWN:
                if events.key == pygame.K_RETURN:
                    obj.initialize()
                if events.key == pygame.K_ESCAPE:
                    obj.state = "e"
                if obj.state == "o" and events.key == pygame.K_SPACE:
                    obj.state = "r"
                    obj.current = obj.start
                    obj.startTime = pygame.time.get_ticks()
                    obj.openset = obj.options()
                if obj.state != "r":
                    if obj.state == "d" and events.key == pygame.K_r:
                        obj.reset()
                        
            if events.type == pygame.QUIT:
                obj.state = "e"
                
        if obj.state == "o":
            obj.mousePosition = pygame.mouse.get_pos()
            if (20 < obj.mousePosition[0] < 425) and (20 < obj.mousePosition[1] < 265):
                if obj.addObstacles:
                    if (obj.mousePosition[0]//obj.cellSize[0],obj.mousePosition[1]//obj.cellSize[1]) != obj.goal:
                        obj.obstacles.update(set(((obj.mousePosition[0]//obj.cellSize[0],obj.mousePosition[1]//obj.cellSize[1]),)))
                elif obj.delObstacles:
                    obj.obstacles -= set(((obj.mousePosition[0]//obj.cellSize[0],obj.target[1]//obj.cellSize[1]),))
        
    def updateUI(obj):#drawing start,goal,obstacle and path on the screen
        for element in obj.obstacles:
            obj.screen.fill((240,0,0),((element[0]*obj.cellSize[0],element[1]*obj.cellSize[1]),obj.cellSize))
        for element in obj.closedset:
            obj.screen.fill((240,240,0),((element[0]*obj.cellSize[0],element[1]*obj.cellSize[1]),obj.cellSize))
        if obj.isSolved:
            for element in obj.solution:
                obj.screen.fill((0,240,0),((element[0]*obj.cellSize[0],element[1]*obj.cellSize[1]),obj.cellSize))
                obj.pathind += 1
            obj.pathind = 0
        if obj.start:
            obj.screen.fill((0,240,0),((obj.start[0]*obj.cellSize[0],obj.start[1]*obj.cellSize[1]),obj.cellSize))
        if obj.goal:
            obj.screen.fill((0,240,0),((obj.goal[0]*obj.cellSize[0],obj.goal[1]*obj.cellSize[1]),obj.cellSize))
        
    def calcDistance(obj,start,goal):#calculate distance between 2 points
        if goal and start:
            dist = abs(goal[0]-start[0])
            dist = dist + abs(goal[1]-start[1])
        else:
            dist = "End is not selected"
        return dist
        
    def options(obj): #generate steps for that node
        oSet = set()
        for (i,j) in adjacencyPath:
            check = (obj.current[0]+i,obj.current[1]+j)
            if check not in obj.obstacles and check not in obj.closedset:
                oSet.update(set((check,)))
        return oSet
    
    def reset(obj):#reset everything apart from start, goal state and obstacles
        temporaryStart = obj.start
        temporaryGoal = obj.goal
        temporaryObstacles = obj.obstacles
        
        obj.initialize()
        obj.start = temporaryStart
        obj.goal = temporaryGoal
        obj.obstacles = temporaryObstacles
        
        obj.gValue[obj.start] = 0
        obj.closedset.update(set((obj.start,)))
        obj.hValue[obj.start] = obj.calcDistance(obj.start,obj.goal)
        obj.state = "o"
        
    def path(obj,cell):#get the path to the goal
        if cell in obj.pathTrace:
            obj.solution.append(cell)
            obj.path(obj.pathTrace[cell])
        
    def evaluate(obj):#implement the algorithm to each the goal state
        if obj.openset and not obj.isSolved:
            if obj.next:
                obj.current = obj.next

            for element in obj.openset:
                if element not in obj.pathTrace:
                    obj.gValue[element] = 1
                    obj.hValue[element] = obj.calcDistance(element,obj.goal)
                    obj.fValue[element] = obj.gValue[element]+obj.hValue[element]
                    obj.pathTrace[element] = obj.start
                if obj.current not in obj.openset:
                    obj.current = element
                elif obj.fValue[element] < obj.fValue[obj.current]:
                    obj.current = element
                    
            if obj.current == obj.goal:
                obj.path(obj.current)
                obj.endTime = pygame.time.get_ticks()
                obj.isSolved = True
            
            obj.openset.discard(obj.current)
            obj.closedset.update(set((obj.current,)))
            neighbors = obj.options()
            obj.next = None
            
            for element in neighbors:#compare neighbours and choose the best option
                tempG = obj.gValue[obj.current]+1
                if element not in obj.openset:
                    obj.openset.update(set((element,)))
                    better = True
                elif element in obj.gValue and tempG < obj.gValue[element]:
                    better = True
                else:
                    better = False

                if better:
                    obj.pathTrace[element] = obj.current
                    obj.gValue[element] = tempG
                    obj.hValue[element] = obj.calcDistance(element,obj.goal)
                    obj.fValue[element] = obj.gValue[element]+obj.hValue[element]
                    if not obj.next:
                        obj.next = element
                    elif obj.fValue[element]<obj.fValue[obj.next]:
                        obj.next = element
                   
        elif obj.isSolved:
            obj.state = "d"
        else:
            obj.endTime = pygame.time.get_ticks()
            obj.state = "d"
        
    def update(obj):#update the screen with appropriate messages
        if obj.state == "r":
            obj.evaluate()
            obj.updateUI()
            obj.screen.blit(grid,(0,0))
            pygame.display.update()
            obj.clock.tick(30)
        else:
            obj.screen.fill((255,255,255))
            obj.updateUI()
            obj.screen.blit(grid,(0,0))
            if obj.state == "s":
                obj.screen.blit(myFont.render("Select Start:",1,(240,240,240)),(5,0))
            elif obj.state == "g":
                obj.screen.blit(myFont.render("Select Goal:",1,(240,240,240)),(5,0))
            elif obj.state == "o":
                obj.screen.blit(myFont.render("Select to add obstacles and then press spacebar for finding path:",1,(240,240,240)),(5,0))
            elif obj.state == "d":
                obj.screen.blit(myFont.render("Press 'Enter' for restarting.  Press 'r' to reset the space.",1,(240,240,240)),(5,0))
                if obj.isSolved:
                    obj.screen.blit(myFont.render("Steps taken: "+str(len(obj.solution)),1,(240,240,240)),(20,260))
                    obj.screen.blit(myFont.render("Time taken in (ms): "+str(obj.endTime-obj.startTime),1,(240,240,240)),(130,260))
                else:
                    obj.screen.blit(myFont.render("solution not found.",1,(240,240,240)),(15,260))
                    obj.screen.blit(myFont.render("Time taken in (ms): "+str(obj.endTime - obj.startTime),1,(240,240,240)),(130,260))
            obj.manageStates()
            pygame.display.update()
        if obj.state == "e":
            pygame.quit();
            sys.exit()
        
        
def main():
    ob.update()

if __name__ == "__main__":
    ob = PathPlanning()
    while True:
        main()
