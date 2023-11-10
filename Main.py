import Environment, Sim, Common, math, random

if __name__ == "__main__":  # initialize the object only once.
    Simulation = Sim.LidarSim(  # all units of measurement are in feet for perceptibility
        ScanThreads=6,
        GuiScale=20,  # pixels per foot
        ShowDeadAngles=True,
        env=Environment.Environment(
            RobotPos=Common.Position(
                0, 0
            ),  # initial robot position with random angle and position for extra localization challenge
            RobotAngle=0,
            SideSize=30,
            RockDiameter=1.5,
            RockCount=15,
            RobotDeadAngles=[
                [math.pi / 6, math.pi / 3],
            ],
        ),
        PointCount=800,
    )

# Simulation initializes all the threads and then ends. this must be modified to run without GUI.
# the gui thread is the only thread that is non-daemon, so the program will end when the gui is closed.

# processing of data should be done in the DigitalProcessing file.
# any additional functions that need to be called should be done in Sim.py ProcessThread() function. this may be changed in the future to a process thread function in the LidarDataProcessor class.
# note that the digital processors intentionally only has access to the robot lidar data, but maybe it could be modified to also receive the robots current position and angle under the assumption other localization systems exist.
