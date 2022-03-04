from dijkastra import *
import sys

print("The start and end coordinates should lie between 200 x 300 area.")
startRow = int(input("Enter the x-coordinate for start node: "))
startCol = int(input("Enter the y-coordinate for start node: "))
goalRow = int(input("Enter the x-coordinate for goal node: "))
goalCol = int(input("Enter the y-coordinate for goal node: "))
radius = int(input("Enter the radius for the robot : "))
clearance = int(input("Enter the clearance for the robot : "))

'''
startRow = 10
startCol = 10
goalRow = 185
goalCol = 185
radius = 1
clearance = 1
'''
# take start and goal node as input
start = (startRow, startCol)
goal = (goalRow, goalCol)
dij = Node(start, goal, None, clearance, radius)

if(dij.validClearance(start[0], start[1])):
    if(dij.validClearance(goal[0], goal[1])):
        if(dij.obstacle(start[0],start[1]) == False):
            if(dij.obstacle(goal[0], goal[1]) == False):
                (explored, backstates, distance_from_start_to_goal) = dij.Dij()
                dij.animation(explored, backstates, "./dijkstraalgo.avi")
                if(distance_from_start_to_goal == float('inf')):
                    print("\nNo optimal path found.")
                else:
                    print("\nOptimal path found. Distance is " + str(distance_from_start_to_goal))
            else:
                print("The entered goal node is an obstacle ")
               
        else:
            print("The entered initial node is an obstacle ")
            
    else:
        print("The entered goal node outside the map ")
        
else:
    print("The entered initial node is outside the map ")
    