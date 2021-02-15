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
    # ground truth
    fig, ax = plt.subplots()
    ax.imshow(game.truthMap, cmap='gray')
    plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.95)
    plt.savefig('../fig/map_gt_' + str(map.number) + '_greedy.png')
    plt.show()
    # explored w/ a robot's trajectory
    fig, ax = plt.subplots()
    ax.plot(path_x, path_y, color='red')
    ax.imshow(game.exploredMap, cmap='gray')
    plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.95)
    plt.savefig('../fig/map_explore_' + str(map.number) + '_greedy.png')
    plt.show()
    # predicted
    fig, ax = plt.subplots()
    ax.imshow(image, cmap='gray')
    plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.95)
    plt.savefig('../fig/map_pred_' + str(map.number) + '_greedy.png')
    plt.show()

if __name__ == '__main__':
    # init map and prob
    map = Map()
    specified_prob = 0.95

    # run exploration
    run_greedy(map, specified_prob)

    # run exploration
    map.getNewMap()
    map.getNewMap()
    run_greedy(map, specified_prob)