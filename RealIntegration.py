import Common, DigitalProcessing, math, guizero
import serial, serial.tools.list_ports, threading


class RealLidar:
    def __init__(
        self,
        SerialCom=None,
        BaudRate=115200,
        Processor=DigitalProcessing.LidarDataProcessor(),
        ShowGui=True,
        GuiScale=20,
        SideSize=30,
    ):
        """Initializes the real lidar object. This object is a wrapper for the serial port and the lidar data processor.

        Args:
            SerialCom (String, optional): The port of the arduino. Defaults to None. (Connects to the first port found)
            BaudRate (int, optional): The serial connection speed. Defaults to 115200.
            Processor (LidarDataProcessor, optional): Algorithm to process incoming lidar data. Defaults to DigitalProcessing.LidarDataProcessor().
            ShowGui (bool, optional): Whether or not to show the gui. Defaults to True.
            GuiScale (int, optional): The scale of the gui. Defaults to 20.
            SideSize (int, optional): The size of the environment canvas. Defaults to 30.
        """
        self.ShowGui = ShowGui
        self.GuiScale = GuiScale
        self.SideSize = SideSize

        if SerialCom == None:
            SerialCom = serial.tools.list_ports.comports()[0].device

        try:
            self.Serial = serial.Serial(port=SerialCom, baudrate=BaudRate, timeout=1)
        except:
            print("Could not connect to serial port.")
            return

        self.Processor = Processor

        self.RobotLidarData = []  # List of points from the lidar

        self.ScanThread = threading.Thread(target=self.ReadData, daemon=True)
        self.ScanThread.start()

        self.ViewThread = threading.Thread(target=self.OpenGui)
        self.ViewThread.start()

    def ReadData(self):
        points = []
        while True:
            if not self.Serial.is_open:  # if the serial port is closed, exit the thread
                return

            buffer = self.Serial.read_until(b"\xfa")  # read until the start byte is found

            if buffer == b"":
                print("No data")
                continue

            buffer = buffer[:-1]  # remove the start byte
            buffer = [i for i in buffer]

            if len(buffer) < 9:
                print("Buffer too short.")
                continue

            index = buffer[0] - 0xA0
            print("Index: " + str(index))

            for j in range(0, 4):
                # QuadrantOffset = math.floor(len(points) / 90) * 90
                # print("Quadrant offset: " + str(QuadrantOffset))
                angle = ((index * 4 + j) * math.pi / 180) % (2 * math.pi)
                lowDist = buffer[1 + 2 * j]
                highDist = buffer[2 + 2 * j]

                if highDist & 0xE0:
                    points.append(Common.Position(angle, 0, False))
                    print("Error at angle " + str(angle) + ".")
                    continue

                dist = highDist & 0x1F
                dist <<= 8
                dist |= lowDist
                dist /= 20
                print("Angle: " + str(angle * 180 / math.pi) + ", Distance: " + str(dist))

                if angle == 0:
                    self.RobotLidarData = points.copy()
                    points = []
                    print("Points sent to queue.")

                points.append(Common.Position(dist, angle, False))

    def OpenGui(self):
        # Initialize the gui and all the gui elements
        # it updates every 10ms
        if self.ShowGui:
            TotalWidth = self.SideSize * self.GuiScale
            self.app = guizero.App(
                title="Lidar", width=TotalWidth + 200, height=TotalWidth + 100, layout="grid"
            )
            self.canvas = guizero.Drawing(
                self.app, width=TotalWidth, height=TotalWidth, grid=[0, 0, 1, 5]
            )
            self.slider = guizero.Slider(self.app, start=1, end=10, horizontal=False, grid=[1, 0])

            self.ViewMode = guizero.Combo(
                self.app,
                options=["Robot", "Processed"],
                grid=[1, 2],
                selected="Robot",
            )

            self.app.repeat(10, self.RedrawPoints)

            self.app.display()

    def RedrawPoints(self):
        # redraws the points on the gui
        # calls different functions based on the view mode
        if self.ViewMode.value == "Robot":
            self.RobotPerspective()
        elif self.ViewMode.value == "Processed":
            self.ProcessedPerspective()

    def RobotPerspective(self):
        # draws the points from the lidar in the robot perspective (robot reference frame)
        def RobotPoint(offset=0):
            RobotPoint1 = Common.Position(
                1.5, offset, False
            )  # 1.5 is the radius of the robot corners, make top right corner offset
            RobotPoint1 = Common.Position(
                (RobotPoint1.x + self.SideSize / 2) * self.GuiScale,
                (-1 * (RobotPoint1.y) + self.SideSize / 2) * self.GuiScale,
            )  # convert to env coordinates
            return RobotPoint1

        self.canvas.clear()
        self.canvas.rectangle(
            0,
            0,
            self.SideSize * self.GuiScale,
            self.SideSize * self.GuiScale,
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

        ScaleFactor = self.slider.value / 100
        for point in self.RobotLidarData:
            self.canvas.oval(
                (point.x + self.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="white",
            )

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
                (RobotPoint1.x + self.SideSize / 2) * self.GuiScale,
                (-1 * (RobotPoint1.y) + self.SideSize / 2) * self.GuiScale,
            )  # convert to env coordinates
            return RobotPoint1

        self.canvas.clear()
        self.canvas.rectangle(
            0,
            0,
            self.SideSize * self.GuiScale,
            self.SideSize * self.GuiScale,
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

        ScaleFactor = self.slider.value / 100
        for point in self.Processor.AcceptableData:
            self.canvas.oval(
                (point.x + self.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="green",
            )

        for point in self.Processor.IllegalData:
            self.canvas.oval(
                (point.x + self.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.SideSize / 2 - ScaleFactor) * self.GuiScale,
                (point.x + self.SideSize / 2 + ScaleFactor) * self.GuiScale,
                (point.y * -1 + self.SideSize / 2 + ScaleFactor) * self.GuiScale,
                color="red",
            )