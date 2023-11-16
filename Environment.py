import Common, random, math, threading
import guizero


class Environment:
    def __init__(
        self,
        SideSize=12,
        RobotPos=Common.Position(0, 0),
        RobotAngle=0,
        RockCount=5,
        RockDiameter=0,
        RobotDeadAngles=[],
    ):
        """The environment object that stores the size of the environment, the robot, and the rocks.
        Args:
            SideSize (int, optional): Size of the environment. Defaults to 12.
            RobotPos (Common.Position, optional): Starting Location of the robot. Defaults to Common.Position(0, 0).
            RobotAngle (int, optional): Starting angle of the robot. Defaults to 0.
            RockCount (int, optional): number of rocks on the environment. Defaults to 5.
            RockDiameter (float, optional): Size of the rocks. Defaults to random(0-1).
            RobotDeadAngles (list [[a,b]], optional): Angles the lidar cannot reach because of interference from the bot. Defaults to [[]].
        """
        self.SideSize = SideSize
        self.Robot = Common.Bot(RobotPos, RobotAngle, RobotDeadAngles)
        self.Rocks = [
            Common.Rock(
                Common.Position(
                    random.random() * self.SideSize / 2 * self.RandomSign(),
                    random.random() * self.SideSize / 2 * self.RandomSign(),
                ),
                (RockDiameter if RockDiameter > 0 else random.random()),
            )
            for i in range(RockCount)
        ]  # List of rocks at random locations made with list comprehension

    def __str__(self):
        return f"Environment with size {self.SideSize} and {len(self.Rocks)} rocks"

    def RandomSign(self):
        """Returns a random sign, either 1 or -1.

        Returns:
            int: 1 or -1 randomly.
        """
        return 1 if random.random() > 0.5 else -1

    def UpdateRobot(self, pos, angle):
        """Update the robot's position and angle. Superfluous because felids are public but it's here.

        Args:
            pos (Position): New position of the robot.
            angle (Radian): New angle of the robot.
        """
        self.Robot.pos = pos
        self.Robot.angle = angle

    def ScanLidar(self, PointCount=365, accuracy=0.001, randomize=0.01, StartStopAngle=[]):
        """Scan the lidar of the robot.

        Args:
            PointCount (int, optional): How many points to scan. Defaults to 365.
            accuracy (float, optional): How small scan iterations are. Defaults to 0.001.
            randomize (float, positive float): How much to randomize the scan. Defaults to 0.01.

        Returns:
            list: List of the Points scanned, Position Objects.
        """

        points = []

        for i in range(PointCount):  # for each point
            if StartStopAngle == []:  # if no start and stop angle
                angle = i / PointCount * 2 * math.pi  # angle is proportion of 2pi times point count
            else:
                angle = StartStopAngle[0] + i / PointCount * (
                    StartStopAngle[1] - StartStopAngle[0]
                )  # angle is proportion of start and stop angle times point count

            if self.Robot.IsDead(angle):  # if angle is in dead angle, skip
                continue

            for rock in self.Rocks:  # for each rock
                if rock.IsIn(self.Robot.pos):  # if ray is in rock
                    points.append(
                        Common.Position(
                            self.Robot.pos.x + random.random() * randomize,
                            self.Robot.pos.y + random.random() * randomize,
                        )
                    )  # add point to list with random noise added. This noise is controllable by the initialization variables.
                    continue

            RayPosition = Common.Position(
                self.Robot.pos.x, self.Robot.pos.y
            )  # start at robot position and project outwards (like a ray)
            InRock = False  # loop breaking variable
            while (
                abs(RayPosition.x) < self.SideSize / 2
                and abs(RayPosition.y) < self.SideSize / 2
                and not InRock
            ):  # while ray is in the environment and not in a rock
                """RayPosition = Common.Position(
                    RayPosition.x + math.cos(angle) * accuracy * 10,
                    RayPosition.y + math.sin(angle) * accuracy * 10,
                )  # move ray forward at angle by accuracy * 10 amount"""

                RayPosition.x += math.cos(angle) * accuracy * 10
                RayPosition.y += math.sin(angle) * accuracy * 10

                for rock in self.Rocks:  # for each rock
                    if rock.IsIn(RayPosition):  # if ray is in rock
                        InRock = True  # break loop on next iteration
                        while rock.IsIn(RayPosition):  # while ray is in rock
                            """RayPosition = Common.Position(
                                RayPosition.x - math.cos(angle) * accuracy,
                                RayPosition.y - math.sin(angle) * accuracy,
                            )  # move ray backwards by accuracy amount until it is just out of the rock
                            """
                            RayPosition.x -= math.cos(angle) * accuracy
                            RayPosition.y -= math.sin(angle) * accuracy
                            # edge finding
                        break

                if random.random() > 0.99999:  # add random noise
                    # if this is triggered the point is added to the list at its current position
                    # this simulates extreme random noise as points can appear anywhere on the ray length.
                    break

            if not InRock:  # if ray is not in rock, it hit the border of the environment
                # find if the ray hit the border on the x or y axis and move it to the border exactly
                if abs(RayPosition.x) > self.SideSize / 2:
                    RayPosition = Common.Position(
                        self.SideSize / 2 * math.copysign(1, RayPosition.x), RayPosition.y
                    )  # if the abs(x) is greater than the side size, move it to the side size (with sign)
                if abs(RayPosition.y) > self.SideSize / 2:
                    RayPosition = Common.Position(
                        RayPosition.x, self.SideSize / 2 * math.copysign(1, RayPosition.y)
                    )  # if the abs(y) is greater than the side size, move it to the side size (with sign)

            points.append(
                Common.Position(
                    RayPosition.x + random.random() * randomize,
                    RayPosition.y + random.random() * randomize,
                )  # add point to list with random noise added. This noise is controllable by the initialization variables.
            )

        # these points are relative to the field, so they must be converted to the robot's position and angle
        RobotDataPoints = []
        for point in points:
            r, theta = Common.Position(
                point.x - self.Robot.pos.x, point.y - self.Robot.pos.y
            ).GetPolar()  # take the difference between the point and the robot's position and convert to polar (translation transformation)
            RobotDataPoints.append(
                Common.Position(r, theta + self.Robot.angle, False)
            )  # add the point to the list with the robot's angle added to the point's angle (rotation transformation)

        return points, RobotDataPoints
