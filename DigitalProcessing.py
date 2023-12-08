import math, Common, time, copy


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

        if len(self.RobotLidarData) < 2:
            time.sleep(0.01)
            return

        self.IllegalData = []
        self.AcceptableData = []
        self.POI = []

        # README for Wall Detection removal algorithm:

        """ClusteredPoints = self.DetectClusters(
            self.RobotLidarData, 0.7
        )  # Takes all the points and clusters them based on jumps in distance between points

        ClusteredPoints = [
            cluster for cluster in ClusteredPoints if len(cluster) > 5
        ]  # remove clusters with less than 5 points

        if len(ClusteredPoints) < 2:
            time.sleep(0.1)
            return

        \"\"\"for i in range(len(ClusteredPoints)):
            if i % 2 == 0:
                self.IllegalData.extend(ClusteredPoints[i])
            else:
                self.AcceptableData.extend(ClusteredPoints[i])\"\"\"

        ClusterLinearRegressions = []
        self.POI = []
        colors = ["red", "blue", "green", "yellow", "purple", "orange"]
        for index, cluster in enumerate(ClusteredPoints):
            XLinearRegression = self.LinearRegression(cluster)
            SwappedCluster = self.SwapXY(cluster)
            YLinearRegression = self.LinearRegression(SwappedCluster)

            XGoodnessOfFit = self.GoodnessOfFit(cluster, XLinearRegression[0], XLinearRegression[1])
            YGoodnessOfFit = self.GoodnessOfFit(
                SwappedCluster, YLinearRegression[0], YLinearRegression[1]
            )

            if XGoodnessOfFit > YGoodnessOfFit:
                ClusterLinearRegressions.append(XGoodnessOfFit)
                self.POI.append(
                    Common.Line(
                        Common.Position(-300, XLinearRegression[0] * -300 + XLinearRegression[1]),
                        Common.Position(300, XLinearRegression[0] * 300 + XLinearRegression[1]),
                        colors[index % len(colors)],
                    )
                )
                for point in cluster:
                    self.POI.append(Common.POIPoint(point, colors[index % len(colors)]))
            else:
                ClusterLinearRegressions.append(YGoodnessOfFit)
                self.POI.append(
                    Common.Line(
                        Common.Position(-300, YLinearRegression[0] * -300 + YLinearRegression[1]),
                        Common.Position(300, YLinearRegression[0] * 300 + YLinearRegression[1]),
                        colors[index % len(colors)],
                    )
                )
                for point in cluster:
                    self.POI.append(Common.POIPoint(point, colors[index % len(colors)]))

        # ClusterLinearRegressionMean = self.Mean(ClusterLinearRegressions)
        print("len: " + str(len(ClusteredPoints)))
        \"\"\"for i in range(len(ClusteredPoints)):
            if ClusterLinearRegressions[i] > 0.75:  # likely a line
                self.IllegalData.extend(ClusteredPoints[i])
            else:
                self.AcceptableData.extend(ClusteredPoints[i])  # likely not a line\"\"\"
        """
        HullPoints = self.ConvexHullPoints(self.RobotLidarData)

        m = (HullPoints[0][1].y - HullPoints[0][0].y) / (HullPoints[0][1].x - HullPoints[0][0].x)
        b = HullPoints[0][0].y - m * HullPoints[0][0].x

        Deviations = [
            self.DeviationFromLine(
                self.RobotLidarData,
                (line[1].y - line[0].y) / (line[1].x - line[0].x),
                line[0].y - (line[1].y - line[0].y) / (line[1].x - line[0].x) * line[0].x,
            )
            for line in HullPoints
        ]

        CopyRobotLidarData = copy.copy(self.RobotLidarData)
        j = 0
        while j < len(Deviations):
            deviation = Deviations[j]
            i = 0
            while i < len(CopyRobotLidarData):
                # print(deviation[i])
                if deviation[i] < 0.15:
                    CopyRobotLidarData.pop(i)
                    for k in range(len(Deviations)):
                        Deviations[k].pop(i)
                else:
                    i += 1
            j += 1

        self.AcceptableData = CopyRobotLidarData
        self.IllegalData = [x for x in self.RobotLidarData if not x in self.AcceptableData]

        # self.IllegalData = SumHallPoints
        colors = ["red", "orange", "yellow", "green", "blue", "purple"]

        for index, line in enumerate(HullPoints):
            AdjustedLine = copy.copy(line)
            AdjustedLine[0].y *= -1
            self.POI.append(Common.Line(line[0], line[1], "blue"))

    # InverseHullPoints = [x for x in self.RobotLidarData if not x in SumHallPoints]
    # self.AcceptableData = InverseHullPoints

    def DetectClusters(self, Points, WindowScale=0.1):
        """Takes a list of points and returns a list of lists of points that are clustered together.
            Uses sliding window algorithm to detect clusters.

        Args:
            Points (Position): List of points to be clustered.
            WindowScale (float, optional): The scale of the window to be used for clustering. Defaults to 0.1, 10% of the dataset.

        Returns:
            List: List of lists of points that are clustered together.
        """

        WindowIndex = 0

        ClusterEndPoints = []

        while WindowIndex + int(len(Points) * WindowScale) < len(Points):
            Window = Points[
                WindowIndex : WindowIndex + int(len(Points) * WindowScale)
            ]  # Get the window of points

            WindowRadii = []
            for i in range(len(Window) - 1):
                WindowRadii.append(
                    math.sqrt(
                        (Window[i + 1].x - Window[i].x) ** 2 + (Window[i + 1].y - Window[i].y) ** 2
                    )
                )  # Get the radii of the window

            WindowRadii.append(
                math.sqrt((Window[0].x - Window[-1].x) ** 2 + (Window[0].y - Window[-1].y) ** 2)
            )

            """WindowRadiiDerivative = []
            for i in range(len(WindowRadii) - 1):
                WindowRadiiDerivative.append(
                    WindowRadii[i + 1] - WindowRadii[i]
                )  # Get the derivative of the radii

            # add the last point to the derivative list
            WindowRadiiDerivative.append(WindowRadii[0] - WindowRadii[-1])"""

            WindowRadiiDerivative = WindowRadii

            AbsoluteWindowRadiiDerivative = [
                abs(derivative) for derivative in WindowRadiiDerivative
            ]  # Get the absolute value of the derivative

            AbsoluteWindowRadiiDerivativeMean = (
                self.Mean(AbsoluteWindowRadiiDerivative) * 1.5
            )  # Get the mean of the absolute value of the derivative

            for i in range(len(Window)):
                if AbsoluteWindowRadiiDerivative[i] > AbsoluteWindowRadiiDerivativeMean:
                    ClusterEndPoints.append(
                        WindowIndex + i
                    )  # If the derivative is greater than the mean, it is likely a cluster end point

            WindowIndex += 1

        ClusterEndPoints = list(set(ClusterEndPoints))  # Remove duplicates

        Clusters = []
        for i in range(len(ClusterEndPoints) - 1):
            Clusters.append(
                Points[ClusterEndPoints[i] + 1 : ClusterEndPoints[i + 1]]
            )  # Get the clusters

        try:
            if ClusterEndPoints[0] > 2:
                Clusters.insert(
                    0, Points[: ClusterEndPoints[0]]
                )  # Get the first cluster if it exists
        except IndexError:
            pass

        return Clusters

    def LinearRegression(self, Points):
        """Takes a list of points and returns a line of best fit.

        Args:
            Points (Positions): List of points to be used for linear regression.

        Returns:
            Tuple: Tuple containing the slope and y-intercept of the line of best fit.
        """

        XPoints = [point.x for point in Points]
        YPoints = [point.y for point in Points]

        XMean = self.Mean(XPoints)
        YMean = self.Mean(YPoints)

        XDiffs = [x - XMean for x in XPoints]
        YDiffs = [y - YMean for y in YPoints]

        SlopeNumerator = sum([x * y for x, y in zip(XDiffs, YDiffs)])

        SlopeDenominator = sum([x**2 for x in XDiffs])

        Slope = SlopeNumerator / SlopeDenominator

        YIntercept = YMean - Slope * XMean

        return Slope, YIntercept

    def Mean(self, nums):
        """Returns the mean of a list of numbers.

        Args:
            nums (list): List of numbers to be averaged.

        Returns:
            float: Mean of the list of numbers.
        """
        return sum(nums) / len(nums)

    def Median(self, nums):
        """Returns the median of a list of numbers.

        Args:
            nums (list): List of numbers to be averaged.

        Returns:
            float: Median of the list of numbers.
        """
        nums.sort()
        if len(nums) % 2 == 0:
            return (nums[len(nums) // 2] + nums[len(nums) // 2 - 1]) / 2
        else:
            return nums[len(nums) // 2]

    def StandardDeviation(self, nums):
        """Returns the standard deviation of a list of numbers.

        Args:
            nums (list): List of numbers to be averaged.

        Returns:
            float: Standard deviation of the list of numbers.
        """
        mean = self.Mean(nums)
        return math.sqrt(sum([(num - mean) ** 2 for num in nums]) / len(nums))

    def SwapXY(self, points):
        """Returns a list of the points given with the x and y values swapped.

        Args:
            points (Positions): A list of positions

        Returns:
            list: A list of positions with the x and y swapped.
        """
        SwappedPoints = []
        for point in points:
            SwappedPoints.append(Common.Position(point.y, point.x))

        return SwappedPoints

    def DeviationFromLine(self, points, slope, YIntercept):
        """Returns the deviation of a list of points from a line.
            Not actually used in acceptable data test

        Args:
            points (positions): A list of positions to get the deviation from
            slope (float): Slope of the line.
            YIntercept (float): Y intercept of the line

        Returns:
            list: A list of distances between the points and the line
        """

        Distances = []
        for point in points:
            d = abs(-1 * slope * point.x + point.y - YIntercept) / math.sqrt((-1 * slope) ** 2 + 1)
            Distances.append(d)

        return Distances

    def GoodnessOfFit(self, points, slope, YIntercept):
        """Returns the goodness of fit of a line to a list of points.

        Args:
            points (positions): A list of positions to get the goodness of fit from
            slope (float): Slope of the line.
            YIntercept (float): Y intercept of the line

        Returns:
            float: The goodness of fit of the line to the points (-1 to 1)
        """
        # sum of (y - predicted y)**2 / sum of (y - mean y)**2
        MeanY = self.Mean([point.y for point in points])
        R = sum([(point.y - slope * point.x - YIntercept) ** 2 for point in points]) / sum(
            [(point.y - MeanY) ** 2 for point in points]
        )
        return R

    def FindPOI(self):
        pass  # TODO: Find points of interest, such as rocks.

    def ConvexHullPoints(self, Points=[]):
        """Returns the points that make up the convex hull of a set of points.

        Args:
            Points (list, optional): List of Common.Positions. Defaults to [].

        Returns:
            list: List of Common.Positions that make up the convex hull.
        """

        def FurthestPointCalc(Points, LinePointOne, LinePointTwo):
            FurthestPoint = copy.copy(Points[0])
            m = (LinePointTwo.y - LinePointOne.y) / (LinePointTwo.x - LinePointOne.x)
            b = LinePointOne.y - m * LinePointOne.x
            FurthestPointDeviation = self.DeviationFromLine([FurthestPoint], m, b)[0]
            for point in Points:
                Deviation = self.DeviationFromLine([point], m, b)[0]
                if Deviation > FurthestPointDeviation:
                    FurthestPoint = copy.copy(point)
                    FurthestPointDeviation = Deviation
            return FurthestPoint

        def ExtrapolateHull(Points, LinePointOne, LinePointTwo):
            # print(len(Points))
            if len(Points) == 0:  # no points left to extrapolate, return hull polygon section
                return [[LinePointOne, LinePointTwo]]
            FurthestPoint = FurthestPointCalc(Points, LinePointOne, LinePointTwo)
            # print(FurthestPoint)
            NewPoints = []
            for point in Points:
                if (
                    not point == FurthestPoint
                    and not point == LinePointOne
                    and not point == LinePointTwo
                ):
                    if LinePointOne.y == LinePointTwo.y:
                        # horizontal bottom, don't include bottom points in raycast
                        if not self.RayCastIntersectContains(
                            [[LinePointOne, FurthestPoint], [FurthestPoint, LinePointTwo]], point
                        ):
                            NewPoints.append(copy.copy(point))
                    else:
                        # slanted bottom, include bottom points in raycast
                        if not self.RayCastIntersectContains(
                            [
                                [LinePointOne, FurthestPoint],
                                [FurthestPoint, LinePointTwo],
                                [LinePointTwo, LinePointOne],
                            ],
                            point,
                        ):
                            NewPoints.append(copy.copy(point))

            m = (FurthestPoint.y - LinePointOne.y) / (FurthestPoint.x - LinePointOne.x)
            b = LinePointOne.y - m * LinePointOne.x

            FilteredNewPoints = [
                point
                for point in NewPoints
                if point.x < FurthestPoint.x and point.y > m * point.x + b
            ]
            InverseFilteredNewPoints = [
                point
                for point in NewPoints
                if point.x > FurthestPoint.x and point.y < m * point.x + b
            ]

            x = ExtrapolateHull(FilteredNewPoints, LinePointOne, FurthestPoint)
            # print("x " + str(x))
            y = ExtrapolateHull(InverseFilteredNewPoints, FurthestPoint, LinePointTwo)
            # print("y " + str(y))
            # print("furthest pint: " + str(FurthestPoint) + " x: " + str(x) + " y: " + str(y))

            x.extend(y)  # combine the two lists of polygon sections
            return x

        LeftMostPoint = copy.copy(Points[0])
        for point in Points:
            if point.x < LeftMostPoint.x:
                LeftMostPoint = copy.copy(point)
            elif point.x == LeftMostPoint.x and point.y < LeftMostPoint.y:
                LeftMostPoint = copy.copy(point)

        RightMostPoint = copy.copy(Points[0])
        for point in Points:
            if point.x > RightMostPoint.x:
                RightMostPoint = copy.copy(point)
            elif point.x == RightMostPoint.x and point.y > RightMostPoint.y:
                RightMostPoint = copy.copy(point)

        # print(LeftMostPoint)
        # print(RightMostPoint)

        # Upper
        m = (RightMostPoint.y - LeftMostPoint.y) / (RightMostPoint.x - LeftMostPoint.x)
        b = LeftMostPoint.y - m * LeftMostPoint.x
        UpperPoints = [copy.copy(point) for point in Points if point.y > m * point.x + b]
        LowerPoints = [copy.copy(point) for point in Points if point.y < m * point.x + b]

        # print(UpperPoints)
        # print(LowerPoints)

        UpperHull = ExtrapolateHull(UpperPoints, LeftMostPoint, RightMostPoint)
        LowerHull = ExtrapolateHull(LowerPoints, RightMostPoint, LeftMostPoint)

        # print("upper hull: " + str(UpperHull))
        # print("lower hull: " + str(LowerHull))

        LowerHull[-1][1] = UpperHull[-1][0]
        LowerHull[0][0] = UpperHull[0][1]

        UpperHull.extend(LowerHull)

        # for line in UpperHull:  # mirror the hull over the y axis
        # line[0].x *= -1
        # line[1].x *= -1

        """i = 0
        while i < len(UpperHull) - 1:
            if UpperHull[i][0] == UpperHull[i + 1][1] and UpperHull[i][1] == UpperHull[i + 1][0]:
                UpperHull.pop(i)  # inverse lines, remove one
                UpperHull[i - 1][1] = UpperHull[i][0]  # combine the two lines

            i += 1"""

        # for line in LowerHull:
        # print("[" + str(line[0]) + ", " + str(line[1]) + "],")

        return UpperHull

    def RayCastIntersectContains(self, polygon=[[]], point=Common.Position()):
        """Returns whether or not a point is inside a polygon.

        Args:
            polygon (list, optional): List of lists of points defining each line in order. Defaults to [[]].
            point (_type_, optional): Point to detect position. Defaults to Common.Position().

        Returns:
            Bool: Whether or not the point is inside the polygon.
        """

        intersectionCount = 0
        for line in polygon:
            if (line[0].y < point.y and line[1].y > point.y) or (
                line[0].y > point.y and line[1].y < point.y
            ):
                # line exists on the same y plane as the point.
                if line[0].x > point.x and line[1].x > point.x:
                    # line is to the right of the point.
                    intersectionCount += 1
                    continue
                else:
                    m = (line[1].y - line[0].y) / (line[1].x - line[0].x)
                    b = line[0].y - m * line[0].x

                    if m > 0:  # line is increasing
                        if point.y > m * point.x + b:  # horizontal ray intersects line
                            intersectionCount += 1
                            continue
                    else:  # line is decreasing
                        if point.y < m * point.x + b:  # horizontal ray intersects line
                            intersectionCount += 1
                            continue

        return (
            intersectionCount % 2 == 1
        )  # odd number of intersections means the point is inside the polygon
