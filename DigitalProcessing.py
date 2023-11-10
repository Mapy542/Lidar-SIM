class LidarDataProcessor:
    def __init__(self):
        """This class is intended to process the data from the lidar and return a list of points that are acceptable for mapping and points of interest.
        This is where noise removal and stuff like that will be done.
        Any other functions the need to be run should be added to the ProcessThread function in Sim.py
        """
        self.RobotLidarData = []  # List of points from the lidar

        self.AcceptableData = (
            []
        )  # List of points that can be used for mapping and points of interest
        self.IllegalData = (
            []
        )  # List of points that are the border of the map, not allowed to be used
        self.POI = []  # List of points of interest, such as rocks, robots, etc.

    def AcceptableProcess(self, NewData=[]):
        if NewData != []:
            self.RobotLidarData = NewData

            # my first attempt to compare points to find the border of the map
            # its real bad maybe the hough transform should be used to detect lines of the border and remove those points.

        self.IllegalData = []
        self.AcceptableData = []

        for i in range(len(self.RobotLidarData) - 1):
            slope = (self.RobotLidarData[i].y - self.RobotLidarData[i - 1].y) / (
                self.RobotLidarData[i].x - self.RobotLidarData[i - 1].x
            )

            slope2 = (self.RobotLidarData[i + 1].y - self.RobotLidarData[i].y) / (
                self.RobotLidarData[i + 1].x - self.RobotLidarData[i].x
            )

            if abs(slope - slope2) < 1:
                self.AcceptableData.append(self.RobotLidarData[i])
            else:
                self.IllegalData.append(self.RobotLidarData[i])

    def FindPOI(self):
        pass  # TODO: Find points of interest, such as rocks.
