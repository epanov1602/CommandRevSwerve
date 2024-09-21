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

**at the end of `pyproject.toml` you should have this (we need to add `pynetworkatables`, to talk with the camera)**

```python
# Other pip packages to install
requires = [
    "pynetworktables"
]
```

**in `robotcontainer.py`, in the __init__ method, we need to add one camera to the list of robot subsystems**
```
    def __init__(self) -> None:
        # The robot's subsystems
        self.camera = LimelightCamera("limelight-pickup")
```

, and for this to work don't forget to add this at the top of `robotcontainer.py`
```
from subsystems.limelight_camera import LimelightCamera
```

**finally, let's add an example of how to use camera in `robotcontainer.py`**
At the end of `configureButtonBindings()` function, add code for joystick button "B" to make robot slowly turn to the object:
```
        def turn_to_object():
            x = self.camera.getX()
            print(f"x={x}")
            turn_speed = -0.005 * x
            self.robotDrive.rotate(turn_speed)

        bButton = JoystickButton(self.driverController, wpilib.XboxController.Button.kB)
        bButton.whileTrue(commands2.RunCommand(turn_to_object, self.robotDrive))
        bButton.onFalse(commands2.InstantCommand(lambda: self.robotDrive.drive(0, 0, 0, False, False)))
```
