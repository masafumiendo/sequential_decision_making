"""
Authors: Masafumi Endo
Date: 02/01/2021
Objective: Implement a greedy solver for the discrete problem
"""

import numpy as np
from random import randint


class GreedyNavigator:
    def __init__(self):
        # The random navigator doesn't have any data members
        # But a more complex navigator may need to keep track of things
        # so you can create data members in this constructor
        self.my_variable = 0
        pass

    def getAction(self, robot, map):
        """ Randomly selects a valid direction for the robot to travel

            The RandomNavigator completely ignores the incoming map of what has been seen so far.
            Maybe a smarter agent would take this additional info into account...
        """
        direction = None

        while direction is None:

            randNumb = randint(0, 3)

            if randNumb == 0:
                if robot.getLoc()[0] - 1 >= 0:
                    direction = 'left'
            if randNumb == 1:
                if robot.getLoc()[0] + 1 <= 27:
                    direction = 'right'
            if randNumb == 2:
                if robot.getLoc()[1] + 1 <= 27:
                    direction = 'down'
            if randNumb == 3:
                if robot.getLoc()[1] - 1 >= 0:
                    direction = 'up'

        return direction