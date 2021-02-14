"""
Authors: Masafumi Endo
Date: 02/13/2021
Version: 2.0
Objective: main function to run greedy navigator
"""

import gzip
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from RobotClass import Robot
from GreedyGameClass import GreedyGame
from GreedyNavigator import GreedyNavigator
from networkFolder.functionList import Map, WorldEstimatingNetwork, DigitClassificationNetwork

def run_greedy(map, specified_prob):

    # get map
    data = map.map
    # init robot and navigator
    robot = Robot(0, 0)
    path_x = np.array(robot.xLoc)
    path_y = np.array(robot.yLoc)
    navigator = GreedyNavigator()
    # init game class w/ user-specified probability
    game = GreedyGame(data, map.number, navigator, robot, specified_prob)
    # init networks
    uNet = WorldEstimatingNetwork()
    classNet = DigitClassificationNetwork()

    # print the number of the current map
    print(map.number)

    # run game until the robot found the goal
    while True:
        found_goal = game.tick()
        print(" ")
        print(f"after one iteration of the game -> {game.getIteration()}: Robot at: ({robot.xLoc}, {robot.yLoc}), Score = {game.getScore()}")
        # add path
        path_x = np.append(path_x, robot.xLoc)
        path_y = np.append(path_y, robot.yLoc)
        if found_goal:
            print(f"Found goal at time step: {game.getIteration()}!")
            break
    print(f"Final Score: {game.score}")

    # create mask
    mask = np.zeros((28, 28))
    for x in range(0, 28):
        for y in range(0, 28):
            if game.exploredMap[y, x] != 128:
                mask[y, x] = 1

    # estimate what the world looks like
    image = uNet.runNetwork(game.exploredMap, mask)
    # estimate what "world" we are in
    char = classNet.runNetwork(image)
    # get the most likely digit
    print(char.argmax())

    # show ground truth, explored, and predicted map
    fig, (ax1, ax2, ax3) = plt.subplots(ncols=3)
    pos = ax1.imshow(game.truthMap)
    pos = ax2.imshow(game.exploredMap)
    pos = ax3.imshow(image, cmap='gray')
    plt.show()

    # show explored map with robot's trajectory
    fig, (ax1, ax2) = plt.subplots(ncols=2)
    ax1.plot(path_x, path_y, color='red')
    ax1.imshow(game.exploredMap, cmap='gray')
    ax2.imshow(image, cmap='gray')
    plt.show()

if __name__ == '__main__':
    # init map and prob
    map = Map()
    specified_prob = 0.95

    # run exploration
    run_greedy(map, specified_prob)

    # # run exploration
    # map.getNewMap()
    # map.getNewMap()
    # run_greedy(map, specified_prob)