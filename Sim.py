import Common, Environment, guizero, threading, math, time, datetime


class LidarSim:
    def __init__(
        self,
        env=Environment.Environment(),
        ShowGui=True,
        GuiScale=150,
        ScanThreads=1,
        PointCount=800,
    ):
        self.env = env
        self.ShowGui = ShowGui
        self.GuiScale = GuiScale
        self.ScanThreads = ScanThreads
        self.PointCount = PointCount

        self.AbsoluteLidarData = []
        self.RobotLidarData = []

        self.FrameTime = 0

        self.ViewThread = threading.Thread(target=self.OpenGui)
        self.ViewThread.start()

        self.LidarScanThreads = []
        self.LidarScanData = [[[], []] for i in range(self.ScanThreads)]
        for i in range(self.ScanThreads):
            startangle = i / self.ScanThreads * 2 * math.pi
            endangle = (i + 1) / self.ScanThreads * 2 * math.pi
            self.LidarScanThreads.append(
                threading.Thread(
                    target=self.LidarThread,
                    args=(
                        [
                            [startangle, endangle],
                            i,
                            int(round(self.PointCount / self.ScanThreads, 0)),
                        ]
                    ),
                    daemon=True,
                )
            )
            self.LidarScanThreads[i].start()

        self.LidarCoordinator = threading.Thread(target=self.LidarCoordinatorThread, daemon=True)
        self.LidarCoordinator.start()

    def OpenGui(self):
        if self.ShowGui:
            TotalWidth = self.env.SideSize * self.GuiScale
            self.app = guizero.App(
                title="Lidar", width=TotalWidth + 200, height=TotalWidth + 100, layout="grid"
            )
            self.canvas = guizero.Drawing(
                self.app, width=TotalWidth, height=TotalWidth, grid=[0, 0]
            )
            self.slider = guizero.Slider(self.app, start=1, end=10, horizontal=False, grid=[1, 0])
            self.FrameTimeText = guizero.Text(self.app, text="0", grid=[1, 1])

            self.app.repeat(50, self.RedrawPoints)
            self.app.display()

    def RedrawPoints(self):
        def RobotPoint(offset=0):
            RobotPoint1 = Common.Position(
                1.5, self.env.Robot.angle + offset, False
            )  # 1.5 is the radius of the robot corners, make top right corner offset
            RobotPoint1 = Common.Position(
                (RobotPoint1.x + self.env.Robot.pos.x + self.env.SideSize / 2) * self.GuiScale,
                (-1 * (RobotPoint1.y + self.env.Robot.pos.y) + self.env.SideSize / 2)
                * self.GuiScale,
            )  # convert to env coordinates
            return RobotPoint1

        try:
            self.canvas.clear()
            self.canvas.rectangle(
                0,
                0,
                self.env.SideSize * self.GuiScale,
                self.env.SideSize * self.GuiScale,
                color="black",
            )

            RobotPoint1 = RobotPoint(math.pi / 4)
            RobotPoint2 = RobotPoint(3 * math.pi / 4)
            RobotPoint3 = RobotPoint(5 * math.pi / 4)
            RobotPoint4 = RobotPoint(7 * math.pi / 4)

            self.canvas.line(
                RobotPoint1.x, RobotPoint1.y, RobotPoint2.x, RobotPoint2.y, color="red"
            )
            self.canvas.line(
                RobotPoint2.x, RobotPoint2.y, RobotPoint3.x, RobotPoint3.y, color="red"
            )
            self.canvas.line(
                RobotPoint3.x, RobotPoint3.y, RobotPoint4.x, RobotPoint4.y, color="red"
            )
            self.canvas.line(
                RobotPoint4.x, RobotPoint4.y, RobotPoint1.x, RobotPoint1.y, color="blue"
            )  # blue is the "up - front"

            ScaleFactor = self.slider.value / 100
            for point in self.AbsoluteLidarData:
                self.canvas.oval(
                    (point.x + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                    (point.y * -1 + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                    (point.x + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                    (point.y * -1 + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                    color="white",
                )
        except Exception as e:
            print(e)

        self.FrameTimeText.value = self.FrameTime

    def LidarThread(self, StartStopAngles, ThreadNumber, PointCount):
        while True:
            while len(self.LidarScanData[ThreadNumber][0]) != 0:
                time.sleep(0.001)  # wait for all data to be generated to restart
            print(str(ThreadNumber) + "working")
            (
                self.LidarScanData[ThreadNumber][0],
                self.LidarScanData[ThreadNumber][1],
            ) = self.env.ScanLidar(PointCount, self.env.SideSize / 1000, 0.05, StartStopAngles)

    def LidarCoordinatorThread(self):
        while True:
            wait = True
            start = time.time()
            while wait:
                done = True
                for lidar in self.LidarScanData:
                    if len(lidar[0]) > 0 and done:
                        continue
                    else:
                        done = False
                if done:
                    wait = False

                time.sleep(0.001)
            end = time.time()
            self.AbsoluteLidarData = []
            self.RobotLidarData = []
            for i in range(self.ScanThreads):
                self.AbsoluteLidarData += self.LidarScanData[i][0]
                self.RobotLidarData += self.LidarScanData[i][1]

                self.LidarScanData[i][0] = []
                self.LidarScanData[i][1] = []
            self.FrameTime = str(datetime.timedelta(seconds=end - start))


inst = LidarSim(
    ScanThreads=1,
    ShowGui=True,
    GuiScale=20,
    env=Environment.Environment(
        RobotPos=Common.Position(-10, 1),
        RobotAngle=1.2,
        SideSize=30,
        RockDiameter=1.5,
        RockCount=15,
    ),
    PointCount=800,
)
