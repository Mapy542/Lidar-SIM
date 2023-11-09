import Environment, Sim, Common

if __name__ == "__main__":
    inst = Sim.LidarSim(
        ScanThreads=6,
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
