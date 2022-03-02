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

        #define circle
        d1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225) - ((25 - sum) * 25 - sum))

        #define polygon

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

        #define square 

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

        