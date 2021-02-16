__author__ = 'Caleytown'

class Robot:
    def __init__(self, xLoc, yLoc, xLim=(0,27), yLim=(0,27)):
        self.xLim = xLim
        self.yLim = yLim
        self.start_loc = xLoc, yLoc
        self.resetRobot()

    def resetRobot(self):
        self.xLoc = self.start_loc[0]
        self.yLoc = self.start_loc[1]

    def getLoc(self):
        return (self.xLoc,self.yLoc)

    def move(self,direction):
        """ Move the robot while respecting bounds"""
        if direction == 'left':
            self.xLoc = max(self.xLoc-1, self.xLim[0])
        elif direction =='right':
            self.xLoc = min(self.xLoc+1, self.xLim[1])
        elif direction == 'down':
            self.yLoc = min(self.yLoc+1, self.yLim[1])
        # Up by default
        elif direction == 'up':
            self.yLoc = max(self.yLoc-1, self.yLim[0])
        else:
            raise ValueError(f"Robot received invalid direction: {direction}!")

