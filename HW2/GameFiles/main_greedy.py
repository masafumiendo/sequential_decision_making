"""
Authors: Masafumi Endo
Date: 02/03/2021
Version: 1.0
Objective: main function to run greedy navigator
"""

import gzip
import numpy as np
from PIL import Image
from RobotClass import Robot
from GreedyGameClass import GreedyGame
from GreedyNavigator import GreedyNavigator
from networkFolder.functionList import Map, WorldEstimatingNetwork, DigitClassificationNetwork

# Create a Map Class Object
map = Map()
map.getNewMap()
map.getNewMap()
# Get the current map from the Map Class
data = map.map

# Print the number of the current map
print(map.number)

# Create a Robot that starts at (0,0)
# The Robot Class stores the current position of the robot
# and provides ways to move the robot
robot = Robot(0, 0)

# The GreedyNavigator class makes the robot move in greedy directions
navigator = GreedyNavigator()

# Create a Game object, providing it with the map data, the goal location of the map, the navigator, and the robot
game = GreedyGame(data, map.number, navigator, robot)

# This loop runs until the robot found the goal.
while True:
    found_goal = game.tick()
    print(" ")
    print(f"after one iteration of the game -> {game.getIteration()}: Robot at: ({robot.xLoc}, {robot.yLoc}), Score = {game.getScore()}")
    if found_goal:
        print(f"Found goal at time step: {game.getIteration()}!")
        break
print(f"Final Score: {game.score}")

# Show how much of the world has been explored
im = Image.fromarray(np.uint8(game.exploredMap)).show()

# Create the world estimating network
uNet = WorldEstimatingNetwork()

# Create the digit classification network
classNet = DigitClassificationNetwork()

# This loop shows how you can create a mask, an grid of 0s and 1s
# where 0s represent unexplored areas and 1s represent explored areas
# This mask is used by the world estimating network
mask = np.zeros((28, 28))
for x in range(0, 28):
    for y in range(0, 28):
        if game.exploredMap[x, y] != 128:
            mask[x, y] = 1

# Creates an estimate of what the world looks like
image = uNet.runNetwork(game.exploredMap, mask)

# Show the image of the estimated world
Image.fromarray(image).show()


# Use the classification network on the estimated image
# to get a guess of what "world" we are in (e.g., what the MNIST digit of the world)
char = classNet.runNetwork(image)
# get the most likely digit
print(char.argmax())

# Get the next map to test your algorithm on.
map.getNewMap()
