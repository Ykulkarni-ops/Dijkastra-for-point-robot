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
    def validClearnace(self, currRow, currColumn):
        sum = self.clearance + self.radius
        return (currRow >= (1+ sum) and currRow <=(200 - sum ) and currColumn >= (1+ sum) and currColumn <= (300 - sum))

    
