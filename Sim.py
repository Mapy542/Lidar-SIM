import Common, Environment, DigitalProcessing, guizero, multiprocessing, math, time, datetime, threading


class LidarSim:
    def __init__(
        self,
        env=Environment.Environment(),
        ShowGui=True,
        GuiScale=150,
        ShowDeadAngles=True,
        ScanThreads=1,
        PointCount=800,
        Processor=DigitalProcessing.LidarDataProcessor(),
    ):
        """The LidarSim object is the main object for the lidar simulation. It handles the gui, the lidar threads, and the processing threads.

        Args:
            env (Environment, optional): A custom environment can be passed in. Defaults to Environment.Environment().
            ShowGui (bool, optional): Whether to create and update the gui. Defaults to True.
            GuiScale (int, optional): Scale factor for the GUI. 1 is 1 foot to 1 px. Defaults to 150.
            ShowDeadAngles (bool, optional): If true, render green lines to show dead angles. Defaults to True.
            ScanThreads (int, optional): Number of threads to initialize for ray casting more is faster usually. Defaults to 1.
            PointCount (int, optional): Total number of lidar points to calculate. Above 1000, gui fails. Defaults to 800.
            Processor (DigitalProcessing.LidarDataProcessor, optional): The class that will do the actual processing. Defaults to DigitalProcessing.LidarDataProcessor().
        """
        self.env = env
        self.ShowGui = ShowGui
        self.GuiScale = GuiScale
        self.ShowDeadAngles = ShowDeadAngles
        self.ScanThreads = ScanThreads
        self.PointCount = PointCount

        self.Processor = Processor

        self.AbsoluteLidarData = []
        self.RobotLidarData = []

        self.FrameTime = 0

        self.ViewThread = threading.Thread(target=self.OpenGui)
        self.ViewThread.start()

        self.SendQueue = multiprocessing.JoinableQueue()
        self.ReturnQueue = multiprocessing.Queue()
        self.LidarScanThreads = []
        self.LidarScanData = [[[], []] for i in range(self.ScanThreads)]
        for i in range(
            self.ScanThreads
        ):  # create the lidar threads and start them (each responsible for a portion of the lidar 1/ScanThreads of the lidar)
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

        # start the coordinator threads and processes

        self.LidarCoordinator = threading.Thread(target=self.LidarCoordinatorThread, daemon=True)
        self.LidarCoordinator.start()

        self.ProcessorInfoQueue = multiprocessing.JoinableQueue()
        self.ProcessorReturnQueue = multiprocessing.Queue()

        self.ProcessorCoordinator = threading.Thread(
            target=self.ProcessQueueCoordinator, daemon=True
        )
        self.ProcessorCoordinator.start()

        self.ProcessorMultiProcess = multiprocessing.Process(
            target=ProcessThread,
            args=(self.Processor, self.ProcessorInfoQueue, self.ProcessorReturnQueue),
            daemon=True,
            name="ProcessorThread",
        )
        self.ProcessorMultiProcess.start()

    def OpenGui(self):
        # Initialize the gui and all the gui elements
        # it updates every 10ms
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

            self.ViewMode = guizero.Combo(
                self.app,
                options=["Robot", "Absolute", "Processed"],
                grid=[1, 2],
                selected="Processed",
            )

            self.app.repeat(10, self.RedrawPoints)

            self.app.when_key_pressed = self.TakeKeyStroke

            self.app.display()

    def RedrawPoints(self):
        # redraws the points on the gui
        # calls different functions based on the view mode
        if self.ViewMode.value == "Absolute":
            self.AbsolutePerspective()
        elif self.ViewMode.value == "Robot":
            self.RobotPerspective()
        elif self.ViewMode.value == "Processed":
            self.ProcessedPerspective()

    def AbsolutePerspective(self):
        # draws the points from the lidar in the absolute perspective (field reference frame)
        def RobotPoint(offset=0):
            # returns the point of the robot corner offset by the angle
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
        )  # clear canvas and draw the field as black

        RobotPoint1 = RobotPoint(math.pi / 4)  # make all four corners of the robot
        RobotPoint2 = RobotPoint(3 * math.pi / 4)
        RobotPoint3 = RobotPoint(5 * math.pi / 4)
        RobotPoint4 = RobotPoint(7 * math.pi / 4)

        # draw the lines of the robot
        self.canvas.line(RobotPoint1.x, RobotPoint1.y, RobotPoint2.x, RobotPoint2.y, color="red")
        self.canvas.line(RobotPoint2.x, RobotPoint2.y, RobotPoint3.x, RobotPoint3.y, color="red")
        self.canvas.line(RobotPoint3.x, RobotPoint3.y, RobotPoint4.x, RobotPoint4.y, color="red")
        self.canvas.line(
            RobotPoint4.x, RobotPoint4.y, RobotPoint1.x, RobotPoint1.y, color="blue"
        )  # blue is the "right - front"

        if self.ShowDeadAngles:  # draw the dead angles if enabled
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

        ScaleFactor = (
            self.slider.value / 100
        )  # scale factor for the points make them bigger or smaller based on the slider
        for point in self.AbsoluteLidarData:  # draw the points
            # the radius is the scale factor
            # the center of the env is 0,0 but the canvas is 0,0 in the top left so we have to convert the coordinates
            # the y is inverted so we have to multiply by -1
            self.canvas.oval(
                (point.x + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="white",
            )

        self.FrameTimeText.value = self.FrameTime

    def RobotPerspective(self):
        # draws the points from the lidar in the robot perspective (robot reference frame)
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

    def ProcessedPerspective(self):
        # draws the points from the lidar in the processed perspective (robot reference frame)
        # usable points in the Process.AcceptableData are green
        # unusable points in the Process.IllegalData are red
        # points of interest in the Process.POI are blue NOT IMPLEMENTED YET
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
        for point in self.Processor.AcceptableData:
            self.canvas.oval(
                (point.x + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="green",
            )

        for point in self.Processor.IllegalData:
            self.canvas.oval(
                (point.x + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.env.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="red",
            )

        self.FrameTimeText.value = self.FrameTime

    def TakeKeyStroke(self, event):
        # takes a key stroke and moves the robot
        # used to move the robot around the area
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
        # this thread is responsible for sending the lidar threads the environment and collecting the data
        # this class can only communicate to the multiprocessing threads through queues
        while True:
            for i in range(
                self.ScanThreads
            ):  # que jobs for each thread, it passes the up-to-date environment to the threads
                self.SendQueue.put(self.env)
            start = time.time()  # start the timer

            self.SendQueue.join()  # wait for all the threads to finish
            end = time.time()  # stop the timer

            self.AbsoluteLidarData = []
            self.RobotLidarData = []
            for i in range(self.ReturnQueue.qsize()):  # get the data from the threads
                ThreadNumber, AbsoluteScanData, RobotScanData = self.ReturnQueue.get()
                self.AbsoluteLidarData += AbsoluteScanData
                self.RobotLidarData += RobotScanData
            self.FrameTime = str(datetime.timedelta(seconds=end - start))[
                5:
            ]  # calculate the time it took to run the lidar

    def ProcessQueueCoordinator(self):
        # this thread is responsible for sending the lidar data to the processing thread and collecting the data
        # this class can only communicate to the multiprocessing threads through queues
        while True:
            self.ProcessorInfoQueue.put(
                self.RobotLidarData
            )  # send the lidar data to the processing thread
            self.ProcessorInfoQueue.join()  # wait for the processing thread to finish

            (
                self.Processor.AcceptableData,
                self.Processor.IllegalData,
                self.Processor.POI,
            ) = (
                self.ProcessorReturnQueue.get()
            )  # get the data from the processing thread for the gui


def ProcessThread(Processor, ProcessorInfoQueue, ProcessorReturnQueue):
    # this is the processing thread
    # it takes the lidar data and processes it
    # is is a separate process from the main process so it is encapsulated with limited access to the environment (no cheating)
    # it has the full performance of a python interpreter so it can be used to do more complex processing
    while True:
        if ProcessorInfoQueue.empty():
            time.sleep(0.001)
            continue
        Processor.RobotLidarData = ProcessorInfoQueue.get()
        # add any more functions that need to be run here
        Processor.AcceptableProcess()
        ProcessorReturnQueue.put((Processor.AcceptableData, Processor.IllegalData, Processor.POI))

        ProcessorInfoQueue.task_done()  # tell the coordinator thread that it is done


def LidarThread(StartStopAngles, ThreadNumber, PointCount, SendQueue, ReturnQueue):
    # this is the lidar thread
    # it takes the environment and calculates the lidar data
    # it is a separate process from the main process so it is able to run in parallel with the main process
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
