"""
Authors: Masafumi Endo
Date: 02/03/2021
Version: 1.0
Objective: Modify "GameClass" from the original class
"""

import numpy as np
from networkFolder.functionList import WorldEstimatingNetwork, DigitClassificationNetwork


class GreedyGame:
    def __init__(self, truthMap, digit, navigator, robot):
        """
        Inputs:
        map_gt: The map to be explored
        digit: the number 0-9 that the map is actually of
        navigator: a class with a getAction(robot, map) function that generates actions
                   for the robot to take given the current map
                   actions should have the form "left", "right", "up", or "down"
        robot: the Robot object from RobotClass.py
        """
        self.truthMap = truthMap
        self.navigator = navigator
        self.robot = robot
        self._digit = digit
        self._goal, self._wrong_goals = self._get_goal(digit)

        self.exploredMap = np.ones(truthMap.shape) * 128
        self._updateMap(self.robot, self.exploredMap, self.truthMap)
        self.score = 0
        self.iterations = 0

        self.uNet = WorldEstimatingNetwork()
        self.classNet = DigitClassificationNetwork()

    def tick(self):
        self.iterations += 1
        # Generate an action for the robot
        action = self.navigator.getAction(self.robot, self.exploredMap)
        # Move the robot
        self.robot.move(action)
        # Update the explored map based on robot position
        self._updateMap(self.robot, self.exploredMap, self.truthMap)

        # creates an estimate of what the world looks like after moving
        mask = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if self.exploredMap[x, y] != 128:
                    mask[x, y] = 1
        image = self.uNet.runNetwork(self.exploredMap, mask)
        # creates an estimate of what world we are in
        char = self.classNet.runNetwork(image)
        estimated_digit = char.argmax()

        # check if we are at the goal and correctly estimate the digit based on exploration
        if self.robot.getLoc() == self._goal:
            if estimated_digit == self._digit:
                self.score += 100
                return True
            else:
                self.score -= 1
                return False
        # if not, the robot has to do exploration
        else:
            self.score -= 1
            return False

    def resetGame(self):
        self.iterations = 0
        self.score = 0

    def getScore(self):
        return self.score

    def getIteration(self):
        return self.iterations

    def _updateMap(self, robot, exploredMap, truthMap):
        # The robot can see all the square around it
        # So iterate through all nearby squares
        for x in range(-1, 2):
            for y in range(-1, 2):
                # if the robot is off the map, do nothing
                if robot.getLoc()[0] + x > 27 or robot.getLoc()[0] + x < 0 or robot.getLoc()[1] + y > 27 or \
                        robot.getLoc()[1] + y < 0:
                    continue
                # Otherwise update the explored map with the actual value of the map
                else:
                    exploredMap[robot.getLoc()[0] + x, robot.getLoc()[1] + y] = truthMap[
                        robot.getLoc()[0] + x, robot.getLoc()[1] + y]

    def _get_goal(self, digit):
        """
            Returns a tuple containing
            - the goal location based on the digit
            - the remaining goal locations

            Note: You should not need to use this function in your code
        """
        goals = [(0, 27), (27, 27), (27, 0)]
        if digit in range(0, 3):
            goal = goals.pop(0)
        elif digit in range(3, 6):
            goal = goals.pop(1)
        elif digit in range(6, 10):
            goal = goals.pop(2)
        else:
            raise ValueError("Bad digit input: " + str(digit))

        return goal, goals