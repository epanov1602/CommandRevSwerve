# Code snippet for adding a camera

- **this goes to `subsystems/limelight_camera.py`**
```python
from wpilib import RobotController, Timer
from commands2 import Subsystem
from ntcore import NetworkTableInstance


class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        #self.pipelineIndexRequest = self.table.getIntegerTopic("pipeline").publish()

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def getSecondsSinceLastHeartbeat(self) -> float:
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        heartbeat = self.getHB()
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

- **at the end of `pyproject.toml` you should have this (we need to add `pyntcore`, to talk with the camera)**

```python
# Other pip packags to install
requires = [
    "pyntcore"
]
```

- **in `robotcontainer.py`, in the __init__ method, we need to add one camera to the list of robot subsystems**
```python
    def __init__(self) -> None:
        # The robot's subsystems
        self.camera = LimelightCamera("limelight-pickup")
```

, and for this to work don't forget to add this at the top of `robotcontainer.py`
```python
from subsystems.limelight_camera import LimelightCamera
```

- **finally, let's add an example of how to use camera in `robotcontainer.py`**

At the end of `configureButtonBindings()` function, add code for joystick button "B" to make robot slowly turn to the object:
```python
        def turn_to_object():
            x = self.camera.getX()
            print(f"x={x}")
            turn_speed = -0.005 * x
            self.robotDrive.rotate(turn_speed)
            # if you want your robot to slowly chase that object... replace this line above with: self.robotDrive.arcadeDrive(0.1, turn_speed)

        bButton = JoystickButton(self.driverController, wpilib.XboxController.Button.kB)
        bButton.whileTrue(commands2.RunCommand(turn_to_object, self.robotDrive))
        bButton.onFalse(commands2.InstantCommand(lambda: self.robotDrive.drive(0, 0, 0, False, False)))
```

- **last move: if your drivetrain does not have a `rotate()` function, add this to `drivesubsystem.py`**
```python
    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param speed: rotation speed 
        """
        self.arcadeDrive(0, rotSpeed)
```
