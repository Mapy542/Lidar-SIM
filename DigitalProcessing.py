class LidarDataProcessor:
    def __init__(self):
        self.RobotLidarData = []

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
