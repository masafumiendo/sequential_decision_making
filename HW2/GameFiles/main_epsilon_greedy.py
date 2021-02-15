"""
Authors: Masafumi Endo
Date: 02/13/2021
Version: 2.0
Objective: main function to run epsilon-greedy navigator
"""

import time
import gzip
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from PIL import Image
from RobotClass import Robot
from EpsilonGreedyGameClass import EpsilonGreedyGame
from GreedyNavigator import GreedyNavigator
from networkFolder.functionList import Map, WorldEstimatingNetwork, DigitClassificationNetwork

flag_show = False

def run_greedy(map, epsilon, specified_prob):

    # get map
    data = map.map
    # init robot and navigator
    robot = Robot(0, 0)
    path_x = np.array(robot.xLoc)
    path_y = np.array(robot.yLoc)
    navigator = GreedyNavigator()
    # init game class w/ user-specified probability
    game = EpsilonGreedyGame(data, map.number, navigator, robot, epsilon, specified_prob)
    # init networks
    uNet = WorldEstimatingNetwork()
    classNet = DigitClassificationNetwork()

    # print the number of the current map
    print(map.number)

    # run game until the robot found the goal
    start = time.time()
    while True:
        found_goal = game.tick()
        print(" ")
        print(f"after one iteration of the game -> {game.getIteration()}: Robot at: ({robot.xLoc}, {robot.yLoc}), Score = {game.getScore()}")
        # add path
        path_x = np.append(path_x, robot.xLoc)
        path_y = np.append(path_y, robot.yLoc)
        if found_goal:
            print(f"Found goal at time step: {game.getIteration()}!")
            print(f"Number of stucked: {game.num_stuck}")
            break
    print(f"Final Score: {game.score}")

    elapsed_time = time.time() - start

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

    if flag_show:
        # show ground truth, explored, and predicted map
        # ground truth
        fig, ax = plt.subplots()
        ax.imshow(game.truthMap, cmap='gray')
        plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.95)
        plt.savefig('../fig/map_gt_' + str(map.number) + '_epsilon_greedy.png')
        plt.show()
        # explored w/ a robot's trajectory
        fig, ax = plt.subplots()
        ax.plot(path_x, path_y, color='red')
        ax.imshow(game.exploredMap, cmap='gray')
        plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.95)
        plt.savefig('../fig/map_explore_' + str(map.number) + '_epsilon_greedy.png')
        plt.show()
        # predicted
        fig, ax = plt.subplots()
        ax.imshow(image, cmap='gray')
        plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.95)
        plt.savefig('../fig/map_pred_' + str(map.number) + '_epsilon_greedy.png')
        plt.show()

    return game.score, elapsed_time

if __name__ == '__main__':
    # init map and prob
    map = Map() # 7
    epsilon = [0, 0.01, 0.1, 0.2]
    specified_prob = 0.8
    reward_list = np.zeros((len(epsilon), 10))
    runtime_list = np.zeros((len(epsilon), 10))

    for row, epsilon_ in enumerate(epsilon):
        for column in range(10):
            # run exploration
            reward, runtime = run_greedy(map, epsilon_, specified_prob)
            reward_list[row][column] = reward
            runtime_list[row][column] = runtime

    if flag_show:
        sns.set()
        fig, ax = plt.subplots(figsize=(12, 8))
        bp = ax.boxplot((reward_list[0], reward_list[1], reward_list[2], reward_list[3]))
        ax.set_xticklabels(['0', '0.01', '0.1', '0.2'])
        ax.yaxis.grid(True)
        plt.xlabel('epsilon', fontsize=18)
        plt.ylabel('reward', fontsize=18)
        plt.tick_params(labelsize=18)
        plt.savefig('../fig/boxplot_reward_' + str(map.number) + '_greedy.png')
        plt.show()

        sns.set()
        fig, ax = plt.subplots(figsize=(12, 8))
        bp = ax.boxplot((runtime_list[0], runtime_list[1], runtime_list[2], runtime_list[3]))
        ax.set_xticklabels(['0', '0.01', '0.1', '0.2'])
        ax.yaxis.grid(True)
        plt.xlabel('epsilon', fontsize=18)
        plt.ylabel('running time [sec]', fontsize=18)
        plt.tick_params(labelsize=18)
        plt.savefig('../fig/boxplot_runtime_' + str(map.number) + '_greedy.png')
        plt.show()
