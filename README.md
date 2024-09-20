## Step-by-step video which shows what's possible
https://www.youtube.com/watch?v=K2Aj0S4-aKI

## Other Rev examples to explore
https://github.com/robotpy/robotpy-rev/tree/main/examples


# Code snippet for adding a camera

***this goes to `subsystems/limelight_camera.py`***
```python
from networktables import NetworkTables
from wpilib import RobotController, Timer
from commands2 import Subsystem

class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        # networktables hostname?
        hostname = f'roborio-{RobotController.getTeamNumber()}-frc.local'
        print(f"Connecting to Networktables server on {hostname}")
        NetworkTables.initialize(server=hostname)

        # connect to networktables server as a client
        self.table = NetworkTables.getTable(self.cameraName)
        self.pipeline = self.table.getEntry("pipeline")
        self.ledMode = self.table.getEntry("ledMode")
        self.camMode = self.table.getEntry("camMode")
        self.percentTimeSeen = self.table.getEntry("percentSeen")
        self.tx = self.table.getEntry("tx")
        self.ty = self.table.getEntry("ty")
        self.ta = self.table.getEntry("ta")
        self.hb = self.table.getEntry("hb")
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def getA(self) -> float:
        return self.ta.getDouble(0.0)

    def getX(self) -> float:
        return self.tx.getDouble(0.0)

    def getY(self) -> float:
        return self.ty.getDouble(0.0)

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        heartbeat = self.hb.getInteger(0)
        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now
        heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
           print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING"))
        self.heartbeating = heartbeating


def _fix_name(name: str):
    if not name:
      name = "limelight"
    return name
```

