import math


class Position:
    def __init__(self, a=0, b=0, isCartesian=True):
        """Position class for storing x,y coordinates in either cartesian or polar coordinates.

        Args:
            a (int, optional): Either x or r. Defaults to 0.
            b (int, optional): Either y or theta. Defaults to 0.
            isCartesian (bool, optional): tells coordinate to either accept xy or r-theta. Defaults to True.
        """
        if isCartesian:
            self.x = a
            self.y = b
        else:
            self.x, self.y = self.ToCartesian(a, b)

    def ToCartesian(self, r, theta):
        """Returns a tuple of the cartesian coordinates of the polar coordinates.

        Args:
            r (float): Radius of the polar coordinates.
            theta (float: radians): Angle of the polar coordinates.

        Returns:
            Tuple: (x,y) cartesian coordinates.
        """
        return r * math.cos(theta), r * math.sin(theta)

    def ToPolar(self, x, y):
        """Returns a tuple of the polar coordinates of the cartesian coordinates.

        Args:
            x (float): x coordinate of the cartesian coordinates.
            y (float): y coordinate of the cartesian coordinates.

        Returns:
            Tuple: (r, theta) polar coordinates.
        """
        return math.sqrt(x**2 + y**2), math.atan2(y, x)

    def __str__(self):
        return f"({self.x}, {self.y})"

    def Get(self):
        """Returns the cartesian coordinates of the position.

        Returns:
            Tuple: (x,y) cartesian coordinates.
        """
        return self.x, self.y

    def GetPolar(self):
        """Returns the polar coordinates of the position.

        Returns:
            Tuple: (r, theta) polar coordinates.
        """
        return self.ToPolar(self.x, self.y)


class Rock:
    def __init__(self, pos=Position(0, 0), diameter=0.5):
        """A rock object that stores its position and diameter.

        Args:
            pos (Position, optional): Location of the rock. Defaults to Position(0,0).
            diameter (int, optional): Size of rock in diameter. Defaults to 5.
        """
        self.pos = pos
        self.diameter = diameter

    def __str__(self):
        return f"Rock at {self.pos} with diameter {self.diameter}"

    def Get(self):
        return self.pos, self.diameter

    def IsIn(self, pos=Position(0, 0)):
        """Checks if a position is inside the rock.

        Args:
            pos (Position, optional): Check if point is inside the rock. Defaults to Position(0,0).

        Return:
            bool: True if inside, False if outside.
        """
        return (pos.x - self.pos.x) ** 2 + (pos.y - self.pos.y) ** 2 < (self.diameter / 2) ** 2


class Bot:
    def __init__(self, pos=Position(0, 0), angle=0, DeadAngles=[]):
        """A robot object that stores its position, angle, and dead angles of the lidar.

        Args:
            pos (Position, optional): Location of the robot. Defaults to Position(0, 0).
            angle (int, optional): Starting Angle. Defaults to 0.
            DeadAngles (list [[a,b]], optional): Angles the lidar cannot reach because of interference from the bot. Defaults to [[]].
        """

        self.pos = pos
        self.angle = angle
        self.DeadAngles = DeadAngles

    def __str__(self):
        return f"Bot at {self.pos} with angle {self.angle}"

    def IsDead(self, angle):
        """Checks if an angle is in the dead angles of the robot.

        Args:
            angle (Radian): Angle to check.

        Returns:
            Bool: True if in dead angle, False if not.
        """
        for deadRange in self.DeadAngles:
            if (
                deadRange[0] <= (angle - self.angle) % (math.pi * 2)
                and (angle - self.angle) % (math.pi * 2) <= deadRange[1]
            ):
                return True
        return False
