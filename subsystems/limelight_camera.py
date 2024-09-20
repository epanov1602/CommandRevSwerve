from networktables import NetworkTables
from commands2 import Subsystem

class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str, teamNumber = None) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        # As a client to connect to a robot
        if teamNumber:
            NetworkTables.initialize(server=f'roborio-{teamNumber}-frc.local')
        self.table = NetworkTables.getTable(self.cameraName)
        self.pipeline = self.table.getEntry("pipeline")
        self.ledMode = self.table.getEntry("ledMode")
        self.camMode = self.table.getEntry("camMode")
        self.percentTimeSeen = self.table.getEntry("percentSeen")
        self.tx = self.table.getEntry("tx")
        self.ty = self.table.getEntry("ty")
        self.ta = self.table.getEntry("ta")
        self.hb = self.table.getEntry("hb")

    def getA(self) -> float:
        return self.ta.getDouble(0.0)

    def getX(self) -> float:
        return self.tx.getDouble(0.0)

    def getY(self) -> float:
        return self.ty.getDouble(0.0)

    def periodic(self) -> None:
        pass

def _fix_name(name: str):
    if not name:
      name = "limelight"
    return name
