# Code snippet for adding a camera (Limelight or PhotonVision)

<details>
<summary>Example code for Limelight camera</summary>

- **this goes to `subsystems/limelight_camera.py`**
```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import Timer
from commands2 import Subsystem
from ntcore import NetworkTableInstance


class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        # "cl" and "tl" are additional latencies in milliseconds

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def hasDetection(self):
        if self.getX() != 0.0 and self.heartbeating:
            return True

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

</details>

<details>
<summary>Example code for PhotonVision camera</summary>

- **this goes to `subsystems/photon_vision_camera.py`**
  (this code is different from code of a more advanced `PhotonCamera` in `photonlibpy` library from PhotonVision team)

```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import Timer
from commands2 import Subsystem
from ntcore import NetworkTableInstance


class PhotonVisionCamera(Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        assert cameraName, "cameraName must be specified"
        self.cameraName = cameraName

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable("photonvision").getSubTable(self.cameraName)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getIntegerTopic("pipelineIndexRequest").publish()
        self.pipelineIndex = self.table.getIntegerTopic("pipelineIndexState").getEntry(-1)

        self.driverModeRequest = self.table.getBooleanTopic("driverModeRequest").publish()
        self.driverMode = self.table.getBooleanTopic("driverMode").getEntry(False)

        self.tx = self.table.getDoubleTopic("targetYaw").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("targetPitch").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("targetArea").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("heartbeat").getEntry(0)
        self.hasTarget = self.table.getBooleanTopic("hasTarget").getEntry(False)
        self.latencyMillis = self.table.getDoubleTopic("latencyMillis").getEntry(0)

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(index)

    def getPipeline(self) -> int:
        return self.pipelineIndex.get(-1)

    def setDriverMode(self, enabled: bool):
        self.driverModeRequest.set(enabled)

    def getDriverMode(self) -> bool:
        return self.driverMode.get(False)

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def hasDetection(self):
        if self.hasTarget.get() and self.heartbeating:
            return True

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

```
</details>

<details>
<summary>
Example code for PhotonVision camera that **will only give you tag IDs that you specified** (for example, "only detect tag 3 and 8")
</summary>



- **this goes to the end of your `pyproject.toml`** (to make sense of tags IDs, install official PhotonVision library compatible with camera)

```python
# Other pip packages to install
requires = [
   "pyntcore",
   "photonlibpy==2024.3.1",  # or you must use "photonlibpy==2025.0.0b" if you installed PhotonVision 2025.0.0b on your actual camera, instead of 2024.3.1
]
```

- **this goes to the end of your `requirements.txt`** (to make sense of tags, you have to install official PhotonVision library compatible with camera)

```bash
# Other pip packages to install
pyntcore
photonlibpy==2024.3.1  # or you must use "photonlibpy==2025.0.0b" if you installed PhotonVision 2025.0.0b on your actual camera, instead of 2024.3.1
```


- **this goes to `subsystems/photon_tag_camera.py`**

```python

#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import Timer
from commands2 import Subsystem

from photonlibpy.photonCamera import PhotonCamera

class PhotonTagCamera(Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        assert cameraName, "cameraName must be specified"
        self.cameraName = cameraName
        self.camera = PhotonCamera(cameraName)
        self.onlyTagIds = None

        self.tx = 0.0
        self.ty = 0.0
        self.ta = 0.0
        self.hb = 0

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def setOnlyTagIds(self, onlyTagIds=()):
        onlyTagIds = set(onlyTagIds) if onlyTagIds else None
        if onlyTagIds != self.onlyTagIds:
            self.heartbeating = False  # assume camera not ready after this change
        self.onlyTagIds = onlyTagIds

    def setPipeline(self, index: int):
        self.camera.setPipelineIndex(index)

    def getPipeline(self) -> int:
        return self.camera.getPipelineIndex()

    def setDriverMode(self, enabled: bool):
        self.camera.setDriverMode(enabled)

    def getDriverMode(self) -> bool:
        return self.camera.getDriverMode()

    def getA(self) -> float:
        return self.ta

    def getX(self) -> float:
        return self.tx

    def getY(self) -> float:
        return self.ty

    def getHB(self) -> float:
        return self.hb

    def hasDetection(self):
        return self.heartbeating and (self.tx != 0 or self.ty != 0)

    def getSecondsSinceLastHeartbeat(self) -> float:
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()

        # 1. if camera is connected, process the camera results
        if self.camera.isConnected():
            result = self.camera.getLatestResult()
            resultTime = result.getTimestamp()
            if resultTime != self.lastHeartbeatTime:
                self.lastHeartbeatTime = resultTime
                self.ta, self.tx, self.ty = 0, 0, 0
                self.hb += 1
                biggest = 0
                for obj in result.getTargets():
                    if self.onlyTagIds and obj.getFiducialId() not in self.onlyTagIds:
                        continue  # if we are only supposed to look at a few tag IDs, skip other tags
                    area = obj.getArea()
                    if area > 0 and area >= biggest:  # between multiple detections, choose the biggest
                        self.tx = obj.getYaw()
                        self.ty = obj.getPitch()
                        self.ta = area
                        biggest = area

        # 2. report the state of the camera
        heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING") + ("" if self.camera.isConnected() else " and IS NOT CONNECTED"))
        self.heartbeating = heartbeating


```
</details>

# Adding such camera to your robot

- **at the end of `pyproject.toml` you should have this (we need to add `pyntcore`, to talk with the camera)**

```python
# Other pip packags to install
requires = [
    "pyntcore"
]
```

- **in `robotcontainer.py`, in the __init__ method, add a Limelight camera to your subsystems (do you have Limelight?)**
```python
    def __init__(self) -> None:
        # The robot's subsystems
        from subsystems.limelight_camera import LimelightCamera
        self.camera = LimelightCamera("limelight-pickup")  # name of your camera goes in parentheses

```

- **or, in `robotcontainer.py`, in the __init__ method, add a PhotonVision camera to your subsystems (do you have PhotonVision?)**
```python
    def __init__(self) -> None:
        # The robot's subsystems
        from subsystems.photon_vision_camera import PhotonVisionCamera
        self.camera = PhotonVisionCamera("front-camera")  # name of your camera goes in parentheses
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

        bButton = self.driverController.button(XboxController.Button.kB)
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
