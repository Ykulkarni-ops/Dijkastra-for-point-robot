from cmath import sqrt
from locale import DAY_2
import numpy as np
import cv2
import math
from collections import defaultdict
from heapq import *


#define class Node

class Node(object):

    #initialize the varibales for start, goal, clearance, radius
    def __init__(self, start, goal, position, clearance, radius):
        self.start = start
        self.goal = goal
        self.position = position
        self.clearance = clearance
        self.radius = radius
        self.cost = 0
        self.x = 0
        self.y = 0
        self.numRows = 300
        self.numColumns = 300

    
    # return position
    def __eq__(self, other):
        return self.position == other.position

    #check if the clearnce is valid 
    def validClearance(self, currRow, currColumn):
        sum = self.clearance + self.radius
        return (currRow >= (1+ sum) and currRow <=(200 - sum ) and currColumn >= (1+ sum) and currColumn <= (300 - sum))

    #define the obstacles
    def obstacle(self, row, col):
        sum = self.clearance + self.radius
        sqrt_rc = 1.414 * sum

        #check  circle
        d1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225) - ((25 - sum) * 25 - sum))

        #check polygon

        (x1, y1) = (120 - (2.62 * sum ), 20 - (1.205 *sum))
        (x2, y2) = (150 - sqrt_rc, 50 )
        (x3, y3) = (185 + sum, 25 - (sum * 0.924))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row -x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col -y3) * (x1 -x3)) - ((y1 - y3) * (row - x3))
        d2 = 1
        if (first <= 0 and second <=  0 and third  <= 0):
            d2 = 0 
        (x1, y1) = (150 - sqrt_rc , 50)
        (x2, y2) = (185 + sum, 25 - (sum * 0.924))
        (x3, y3) = (185 + sum, 75 - (sum * 0.514))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row -x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col -y3) * (x1 -x3)) - ((y1 - y3) * (row - x3))
        d3 = 1
        if ( first >= 0 and second >= 0 and third >= 0):
            d3 = 0
        (x1, y1) = (10 - sqrt_rc, 50)
        (x2, y2) = (25, 200 - sqrt_rc)
        (x3, y3) = (40 + sqrt_rc, 225)
        (x4, y4) = (25, 250 + sqrt_rc)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row -x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col -y3) * (x4 -x3)) - ((y3 - y4) * (row - x3))
        fourth = ((col -y4) * (x1 -x4)) - ((y1 - y4) * (row - x4))
        d4 = 1
        d5 = 1
        if (first <= 0 and second <= 0 and third <= 0 and fourth <=0):
            d4 = 0
            d5 = 0

        #check square 

        (x1, y1) = ( 150 - sqrt_rc , 30)
        (x2, y2) = ( 120 - sqrt_rc , 50)
        (x3, y3) = ( 150, 80 + sqrt_rc)
        (x4, y4) = ( 185 + sum , 75 + (sum * 0.514))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row -x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col -y3) * (x4 -x3)) - ((y3 - y4) * (row - x3))
        fourth = ((col -y4) * (x1 -x4)) - ((y1 - y4) * (row - x4))
        d6 = 1 
        d7 = 1
        if (first <= 0 and second <= 0 and third <= 0 and fourth <=0):
            d6 = 0
            d7 = 0

        #defining the actions for Dijkastra

        #action to move up 
        def moveUp(self, currRow, currCol):
            if(self.validClearance(currRow - 1, currCol) and self.obstacle(currRow -1 , currCol) == False):
                self.x = 0
                self.y = 1
                self.cost = 1
                return True 
            else: 
                return False

        #action to move down
        def moveDown(self, currRow, currCol):
            if(self.validClearance(currRow, currCol - 1) and self.obstacle( currRow, currCol -1) == False):
                self.x = 0
                self.y = -1
                self.cost = 1
                return True
            else:
                return False

        #action to move right 
        def moveRight(self, currRow, currCol):
            if(self.validClearance(currRow + 1, currCol) and self.obstacle( currRow + 1, currCol) == False):
                self.x = 1
                self.y = 0
                self.cost = 1
                return True
            else:
                return False
        
        #action to move left
        def moveLeft(self, currRow, currCol):
            if(self.validClearance(currRow - 1, currCol) and self.obstacle( currRow - 1, currCol) == False):
                self.x = -1
                self.y = 0
                self.cost = 1
                return True
            else:
                return False

        #action to move up right 
        def moveUpRight(self, currRow, currCol):
            if(self.validClearance(currRow + 1, currCol + 1) and self.obstacle( currRow + 1, currCol + 1) == False):
                self.x = 1
                self.y = 1
                self.cost = np.sqrt(2)
                return True
            else:
                return False

        #action to move up left 
        def moveUpLeft(self, currRow, currCol):
            if(self.validClearance(currRow - 1, currCol + 1) and self.obstacle( currRow - 1, currCol + 1) == False):
                self.x = -1
                self.y = +1
                self.cost = np.sqrt(2)
                return True
            else:
                return False

        #action to move down right 
        def moveDownRight(self, currRow, currCol):
            if(self.validClearance(currRow + 1, currCol - 1) and self.obstacle( currRow + 1, currCol - 1) == False):
                self.x = 1
                self.y = -1
                self.cost = np.sqrt(2)
                return True
            else:
                return False

        #action to move down left 
        def moveDownLeft(self, currRow, currCol):
            if(self.validClearance(currRow - 1, currCol - 1) and self.obstacle( currRow - 1, currCol - 1) == False):
                self.x = -1
                self.y = -1
                self.cost = np.sqrt(2)
                return True
            else:
                return False

        #check if goal is reached
        def goalReached(self, currRow, currCol):
            if(currRow == self.goal[0] and currCol == self.goal[1]):
                print("GOAL REACHED !!!!!")
                return True
            else:
                return False

        #Dijkastra algorithm
        def dijAlgo(self):
            costMap = {}
            visited_nodes = {}
            path = {}

            for row in np.arange(1, self.numRows + 1 ,1):
                for col in np.arange(1, self.numCols + 1, 1):
                    costMap[(row,col)] = float('inf')
                    visited_nodes[(row,col)] = False
                    path[(row,col)] = -1

            #define two lists one for explored nodes and other for queue

            explored_nodes = []
            queue = []

            heappush(queue, (0, self.start))
            costMap[self.start] = 0

            while(len(queue) > 0):
                heapify(queue)
                _, currNode = heappop(queue)
                visited_nodes[currNode] = True
                explored_nodes.append(currNode)

                # if goal is reached break the loop
                if(self.goalReached(currNode[0],currNode[1]) == True):
                    break

                # move up and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.    
                if(self.moveUp(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))

                
                # move down and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.
                if(self.moveDown(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))

                
                # move right and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.    
                if(self.moveRight(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))


                # move left and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.    
                if(self.moveLeft(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))


                # move up right  and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.    
                if(self.moveUpRight(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))


                # move up Left and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.    
                if(self.moveUpLeft(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))

                
                # move down right and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.    
                if(self.moveDownRight(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))

                
                # move down Left and check the cost of next node and compare it update the cost for current node
                # with the minimum cost obtained after comparison.    
                if(self.moveDownLeft(currNode[0],currNode[1] and visited_nodes[(currNode[0] + self.x , currNode[1] + self.y)] == False 
                    and costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[currNode] + self.cost )):

                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode) + self.cost]
                    path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                    heappush(queue, (costMap[(currNode[0] + self.x , currNode[1] + self.y)], (currNode[0] + self.x , currNode[1] + self.y)))

            
            #define a list to check if path exits 
            check = []

            #define variables for goal 
            goalx = self.goal[0]
            goaly = self.goal[1]

            #update the check list if the path exits 
            for a in np.arange(goalx - 1, goaly + 1, 1):
                for b in np.arange(goalx - 1, goaly + 1 , 1 ):
                    check.append(costMap[a,b])

            #if the path exists print path exits if not then print path does not exist

            dummy_cost = float('inf')
            for c in range(len(check)):
                if (check[c] != dummy_cost):
                    print ("PATH EXISTS !!!")
                
                noPath = 1
                break 
            
            if (noPath == 0):
                print("PATH DOES NOT EXITST !!!")
                return(explored_nodes , [], costMap[goalx, goaly])

            print(costMap[goalx, goaly], "Shortest path")
            result = (goalx, goaly)


            #back tracking

            #initialize list for backtrack
            backtrack = []
            node = result

            while(path[node] != -1):
                backtrack.append(node)
                node = path[node]

            backtrack.append(self.start)
            backtrack = list(reversed(backtrack))
            print(backtrack)
            return (explored_nodes, backtrack, costMap[goalx, goaly])


        def animation(self, explored_nodes, backtrack, path):
