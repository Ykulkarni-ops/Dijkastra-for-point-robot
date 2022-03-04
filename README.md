# DIJKASTRA ALGORITHM USING PYTHON
The project is a part of the course ENPM661 Path planning for robots. The project is the implementation of Dijkastra algorithm for a point robot.
Various libraris inclusing the OpenCv and numpy have been used to obtain the desired result

## DEPENDENCIES
This code was tested with the following dependencies:
- Python 
- Opencv 3.4.5
- Numpy
- heapq
- math
- collections

## INSTRUCTIONS

The first thing to do is cloning the github repository into your PC

- open the windows terminal/ linux shell into the desired location 
- type the command 'git clone https://github.com/Ykulkarni-ops/Dijkastra-for-point-robot.git'
- press '<enter>'
- this will clone the repository in your pc


After the repository is cloned it consists of two folders: 
- code
- results

To run the code :
- check the dijkastra.py file [contains the main algorithm for the dijkastra and the visualization]
- check the dij,py file [here the dijkastra.py is imported with all the function definations]
- after the files are checked do the following to obtain results:
	1. Make sure that both the files dij.py and dijkastra.py are in same folder
	2. Open terminal in the code folder
	3. Type python dij.py or python3 dij.py
- The blue part indicates the explored nodes 
- the green part indicates the unexplored nodes

To view results:
- After the program is run the file dijkastraalgo.avi will be generated
- Also two windows will pop up showing the explored nodes and the final result of path traveled

## RESULTS

- Explored nodes 

![Explored nodes](https://github.com/Ykulkarni-ops/Dijkastra-for-point-robot/blob/main/results/explored%20nodes.JPG)

-Result

![Result](https://github.com/Ykulkarni-ops/Dijkastra-for-point-robot/blob/main/results/result.JPG)

-Path tracking

![Path tracking](https://github.com/Ykulkarni-ops/Dijkastra-for-point-robot/blob/main/results/pathtracking.gif)

## AUTHOR
[Yash Kulkarni](https://github.com/Ykulkarni-ops)
