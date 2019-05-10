# Author: Shengen Wei
# Path-planning algorithms
# including: A*, BFS, Dijkstra
# 

import cv2 as cv
import numpy
from heapq import *
from collections import deque

# A* algorithm
# input
#   array: roadmap  numpy.array,dtype=uint8. In array, 0 is passable, 1 or other is impassable
#   start: startpoint tuple,(start_x,start_y)
#   goal : goalpoint  tuple,(goal_x,goal_y)
# output: if path is exist, output the list of point(tuple), otherwise, output False
def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0] + i, current[1] + j)          
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] !=0:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False
# BFS algorithm
# input
#   array: roadmap  numpy.array,dtype=uint8. In array, 0 is passable, 1 or other is impassable
#   start: startpoint tuple,(start_x,start_y)
#   goal : goalpoint  tuple,(goal_x,goal_y)
# output: if path is exist, output the list of point(tuple), otherwise, output False
def bfs(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    came_from = {}    
    search_queue = deque()
    close_set = set()

    search_queue.append(start)    
    close_set.add(start)
    while search_queue:
        current = search_queue.popleft()
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data
        else:
            for i,j in neighbors:
                neighbor = (current[0]+i,current[1]+j)
                if 0<=neighbor[0]<array.shape[0] and 0<=neighbor[1]<array.shape[1]:
                    if neighbor not in close_set and array[neighbor[0]][neighbor[1]] ==0:
                        search_queue.append(neighbor)
                        close_set.add(neighbor)
                        came_from[neighbor] = current
    return False

def dijkstra(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    oheap = []
    heappush(oheap, start)
    close_set = set()
    gscore = {start:0}
    came_from = {}
    while oheap:
        current = heappop(oheap)
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data

        close_set.add(current)
        for i,j in neighbors:
            neighbor = current[0]+i,current[1]+j
            cost = gscore[current] + 1

            if neighbor[0]<0 or neighbor[0]>=array.shape[0] or neighbor[1]<0 or neighbor[1]>=array.shape[1]:
                continue
            if array[neighbor[0]][neighbor[1]] !=0:
                continue
            if neighbor in close_set and cost >= gscore.get(neighbor, 0):
                continue
            if cost < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = cost
                heappush(oheap, (neighbor))
                
    return False

'''Here is an example of using my algo with a numpy array,
   astar(array, start, goal)

   astar function returns a list of points (shortest path)'''
if __name__ == "__main__":
    nmap = numpy.array([
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1,1,1,1,1,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,0,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1,1,1,1,1,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,0,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1,1,1,1,1,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0]],dtype=numpy.uint8)
    print('astar:',astar(nmap,(0,0),(10,13)))
    print('bfs: ', bfs(nmap, (0,0), (10,13)))
    #nmap = nmap*255
    #print(nmap.dtype)
    #cv.imshow("nmap",nmap)
    #cv.waitKey(0)
    #cv.destroyAllWindows() #使用Window name销毁指定窗口