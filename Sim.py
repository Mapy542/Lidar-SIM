import Common, Environment, guizero, multiprocessing, math, time, datetime, threading


class LidarSim:
    def __init__(
        self,
        env=Environment.Environment(),
        ShowGui=True,
        GuiScale=150,
        ShowDeadAngles=True,
        ScanThreads=1,
        PointCount=800,
    ):
        self.env = env
        self.ShowGui = ShowGui
        self.GuiScale = GuiScale
        self.ShowDeadAngles = ShowDeadAngles
        self.ScanThreads = ScanThreads
        self.PointCount = PointCount

        self.AbsoluteLidarData = []
        self.RobotLidarData = []

        self.FrameTime = 0

        self.ViewThread = threading.Thread(target=self.OpenGui)
        self.ViewThread.start()

        self.SendQueue = multiprocessing.JoinableQueue()
        self.ReturnQueue = multiprocessing.Queue()
        self.LidarScanThreads = []
        self.LidarScanData = [[[], []] for i in range(self.ScanThreads)]
        for i in range(self.ScanThreads):
            startangle = i / self.ScanThreads * 2 * math.pi
            endangle = (i + 1) / self.ScanThreads * 2 * math.pi
            self.LidarScanThreads.append(
                multiprocessing.Process(
                    target=LidarThread,
                    args=(
                        [
                            [startangle, endangle],
                            i,
                            int(round(self.PointCount / self.ScanThreads, 0)),
                            self.SendQueue,
                            self.ReturnQueue,
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
                self.app, width=TotalWidth, height=TotalWidth, grid=[0, 0, 1, 5]
            )
            self.slider = guizero.Slider(self.app, start=1, end=10, horizontal=False, grid=[1, 0])
            self.FrameTimeText = guizero.Text(self.app, text="0", grid=[1, 1])

            self.ViewMode = guizero.CheckBox(self.app, text="Absolute", grid=[1, 2])
            self.ViewMode.value = True

            self.app.repeat(10, self.RedrawPoints)

            self.app.when_key_pressed = self.TakeKeyStroke

            self.app.display()

    def RedrawPoints(self):
        if self.ViewMode.value:
            self.AbsolutePerspective()
        else:
            self.RobotPerspective()

    def AbsolutePerspective(self):
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

        self.canvas.line(RobotPoint1.x, RobotPoint1.y, RobotPoint2.x, RobotPoint2.y, color="red")
        self.canvas.line(RobotPoint2.x, RobotPoint2.y, RobotPoint3.x, RobotPoint3.y, color="red")
        self.canvas.line(RobotPoint3.x, RobotPoint3.y, RobotPoint4.x, RobotPoint4.y, color="red")
        self.canvas.line(
            RobotPoint4.x, RobotPoint4.y, RobotPoint1.x, RobotPoint1.y, color="blue"
        )  # blue is the "up - front"

        if self.ShowDeadAngles:
            for angleRange in self.env.Robot.DeadAngles:
                LowX, LowY = Common.Position(
                    self.env.SideSize * 1.5,
                    (angleRange[0] + self.env.Robot.angle) % (math.pi * 2),
                    False,
                ).Get()
                LowerBoundaryPoint = Common.Position(
                    (LowX + self.env.SideSize / 2 + self.env.Robot.pos.x) * self.GuiScale,
                    (-1 * LowY + self.env.SideSize / 2 - self.env.Robot.pos.y) * self.GuiScale,
                )
                HighX, HighY = Common.Position(
                    self.env.SideSize * 1.5,
                    (angleRange[1] + self.env.Robot.angle) % (math.pi * 2),
                    False,
                ).Get()
                HigherBoundaryPoint = Common.Position(
                    (HighX + self.env.SideSize / 2 + self.env.Robot.pos.x) * self.GuiScale,
                    (-1 * HighY + self.env.SideSize / 2 - self.env.Robot.pos.y) * self.GuiScale,
                )

                RobotPoint = Common.Position(
                    (self.env.Robot.pos.x + self.env.SideSize / 2) * self.GuiScale,
                    (-1 * self.env.Robot.pos.y + self.env.SideSize / 2) * self.GuiScale,
                )

                self.canvas.line(
                    RobotPoint.x,
                    RobotPoint.y,
                    LowerBoundaryPoint.x,
                    LowerBoundaryPoint.y,
                    color="green",
                )
                self.canvas.line(
                    RobotPoint.x,
                    RobotPoint.y,
                    HigherBoundaryPoint.x,
                    HigherBoundaryPoint.y,
                    color="green",
                )

        ScaleFactor = self.slider.value / 100
        for point in self.AbsoluteLidarData:
            self.canvas.oval(
                (point.x + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="white",
            )

        self.FrameTimeText.value = self.FrameTime

    def RobotPerspective(self):
        def RobotPoint(offset=0):
            RobotPoint1 = Common.Position(
                1.5, offset, False
            )  # 1.5 is the radius of the robot corners, make top right corner offset
            RobotPoint1 = Common.Position(
                (RobotPoint1.x + self.env.SideSize / 2) * self.GuiScale,
                (-1 * (RobotPoint1.y) + self.env.SideSize / 2) * self.GuiScale,
            )  # convert to env coordinates
            return RobotPoint1

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

        self.canvas.line(RobotPoint1.x, RobotPoint1.y, RobotPoint2.x, RobotPoint2.y, color="red")
        self.canvas.line(RobotPoint2.x, RobotPoint2.y, RobotPoint3.x, RobotPoint3.y, color="red")
        self.canvas.line(RobotPoint3.x, RobotPoint3.y, RobotPoint4.x, RobotPoint4.y, color="red")
        self.canvas.line(
            RobotPoint4.x, RobotPoint4.y, RobotPoint1.x, RobotPoint1.y, color="blue"
        )  # blue is the "up - front"

        if self.ShowDeadAngles:
            for angleRange in self.env.Robot.DeadAngles:
                LowX, LowY = Common.Position(
                    self.env.SideSize * 1.5,
                    (angleRange[0]) % (math.pi * 2),
                    False,
                ).Get()
                LowerBoundaryPoint = Common.Position(
                    (LowX + self.env.SideSize / 2) * self.GuiScale,
                    (-1 * LowY + self.env.SideSize / 2) * self.GuiScale,
                )
                HighX, HighY = Common.Position(
                    self.env.SideSize * 1.5,
                    (angleRange[1]) % (math.pi * 2),
                    False,
                ).Get()
                HigherBoundaryPoint = Common.Position(
                    (HighX + self.env.SideSize / 2) * self.GuiScale,
                    (-1 * HighY + self.env.SideSize / 2) * self.GuiScale,
                )

                RobotPoint = Common.Position(
                    (self.env.SideSize / 2) * self.GuiScale,
                    (self.env.SideSize / 2) * self.GuiScale,
                )

                self.canvas.line(
                    RobotPoint.x,
                    RobotPoint.y,
                    LowerBoundaryPoint.x,
                    LowerBoundaryPoint.y,
                    color="green",
                )
                self.canvas.line(
                    RobotPoint.x,
                    RobotPoint.y,
                    HigherBoundaryPoint.x,
                    HigherBoundaryPoint.y,
                    color="green",
                )

        ScaleFactor = self.slider.value / 100
        for point in self.RobotLidarData:
            self.canvas.oval(
                (point.x + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="white",
            )

        self.FrameTimeText.value = self.FrameTime

    def TakeKeyStroke(self, event):
        code = event.keycode
        if code == 37:
            self.env.Robot.pos.x -= 0.1
        elif code == 38:
            self.env.Robot.pos.y += 0.1
        elif code == 39:
            self.env.Robot.pos.x += 0.1
        elif code == 40:
            self.env.Robot.pos.y -= 0.1
        elif code == 65:
            self.env.Robot.angle += 0.1
        elif code == 68:
            self.env.Robot.angle -= 0.1

    def LidarCoordinatorThread(self):
        while True:
            for i in range(self.ScanThreads):
                self.SendQueue.put(self.env)
            start = time.time()

            self.SendQueue.join()
            end = time.time()

            self.AbsoluteLidarData = []
            self.RobotLidarData = []
            for i in range(self.ReturnQueue.qsize()):
                ThreadNumber, AbsoluteScanData, RobotScanData = self.ReturnQueue.get()
                self.AbsoluteLidarData += AbsoluteScanData
                self.RobotLidarData += RobotScanData
            self.FrameTime = str(datetime.timedelta(seconds=end - start))[5:]


def LidarThread(StartStopAngles, ThreadNumber, PointCount, SendQueue, ReturnQueue):
    while True:
        if SendQueue.empty():
            time.sleep(0.001)
            continue
        JobEnv = SendQueue.get()
        AbsoluteScanData, RobotScanData = JobEnv.ScanLidar(
            PointCount, JobEnv.SideSize / 1000, 0.05, StartStopAngles
        )
        ReturnQueue.put([ThreadNumber, AbsoluteScanData, RobotScanData])
        SendQueue.task_done()


if __name__ == "__main__":
    inst = LidarSim(
        ScanThreads=8,
        ShowGui=True,
        GuiScale=20,
        ShowDeadAngles=True,
        env=Environment.Environment(
            RobotPos=Common.Position(0, 0),
            RobotAngle=0,
            SideSize=30,
            RockDiameter=1.5,
            RockCount=15,
            RobotDeadAngles=[
                # [math.pi / 6, math.pi / 3],
            ],
        ),
        PointCount=800,
    )
