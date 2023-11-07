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

        (ALL UNITS ARE IN FEET)
        Args:
            SideSize (int, optional): Size of the environment. Defaults to 12.
            RobotPos (_type_, optional): Starting Location of the robot. Defaults to Common.Position(0, 0).
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
        ]  # List of rocks

    def __str__(self):
        return f"Environment with size {self.SideSize} and {len(self.Rocks)} rocks"

    def RandomSign(self):
        return 1 if random.random() > 0.5 else -1

    def UpdateRobot(self, pos, angle):
        self.Robot.pos = pos
        self.Robot.angle = angle

    def ScanLidar(self, PointCount=365, accuracy=0.001, randomize=0.01, StartStopAngle=[]):
        """Scan the lidar of the robot.

        Args:
            PointCount (int, optional): How many points to scan. Defaults to 365.
            accuracy (float, optional): How small scan iterations are. Defaults to 0.001.
            randomize (float, 0- inf): How much to randomize the scan. Defaults to 0.01.

        Returns:
            list: List of the Points scanned, Position Objects.
        """

        points = []

        for i in range(PointCount):
            if StartStopAngle == []:
                angle = i / PointCount * 2 * math.pi
            else:
                angle = StartStopAngle[0] + i / PointCount * (StartStopAngle[1] - StartStopAngle[0])

            if self.Robot.IsDead(angle):
                continue

            raypos = self.Robot.pos
            inrock = False
            while (
                abs(raypos.x) < self.SideSize / 2
                and abs(raypos.y) < self.SideSize / 2
                and not inrock
            ):
                raypos = Common.Position(
                    raypos.x + math.cos(angle) * accuracy * 10,
                    raypos.y + math.sin(angle) * accuracy * 10,
                )

                for rock in self.Rocks:
                    if rock.IsIn(raypos) and not inrock:
                        inrock = True
                        while rock.IsIn(raypos):
                            raypos = Common.Position(
                                raypos.x - math.cos(angle) * accuracy,
                                raypos.y - math.sin(angle) * accuracy,
                            )
                        break

                if random.random() > 0.99999:  # add random noise
                    break

            points.append(
                Common.Position(
                    raypos.x + random.random() * randomize,
                    raypos.y + random.random() * randomize,
                )
            )

        RobotDataPoints = [
            Common.Position(
                (point.x**2 + self.Robot.pos.x**2) ** 0.5,
                (point.y**2 + self.Robot.pos.y**2) ** 0.5,
            )
            for point in points
        ]  ## Convert to robot data points

        return points, RobotDataPoints


def redraw(canvas):
    global inst
    points = inst.points
    canvas.clear()
    canvas.rectangle(0, 0, inst.SideSize, inst.SideSize, color="black")

    canvas.oval(
        inst.Robot.pos.x + inst.SideSize / 2 - 5,
        inst.Robot.pos.y * -1 + inst.SideSize / 2 - 5,
        inst.Robot.pos.x + inst.SideSize / 2 + 5,
        inst.Robot.pos.y * -1 + inst.SideSize / 2 + 5,
        color="green",
    )

    for point in points:
        if point.y > 0:
            canvas.oval(
                point.x + inst.SideSize / 2 - 3,
                point.y * -1 + inst.SideSize / 2 - 3,
                point.x + inst.SideSize / 2 + 3,
                point.y * -1 + inst.SideSize / 2 + 3,
                color="red",
            )
        else:
            canvas.oval(
                point.x + inst.SideSize / 2 - 3,
                point.y * -1 + inst.SideSize / 2 - 3,
                point.x + inst.SideSize / 2 + 3,
                point.y * -1 + inst.SideSize / 2 + 3,
                color="blue",
            )
