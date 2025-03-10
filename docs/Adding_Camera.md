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
   "photonlibpy",
]
```

- **this goes to the end of your `requirements.txt`** (to make sense of tags, you have to install official PhotonVision library compatible with camera)

```bash
# Other pip packages to install
pyntcore
photonlibpy
```


- **this goes to `subsystems/photon_tag_camera.py`**

```python

#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import Timer, Field2d
from commands2 import Subsystem

from photonlibpy.photonCamera import PhotonCamera
from wpimath.geometry import Pose2d, Pose3d, Transform2d, Rotation2d
import math

from robotpy_apriltag import AprilTagFieldLayout

U_TURN = Rotation2d.fromDegrees(180)
SIM_MAX_TAG_DISTANCE = 6  # meters
SIM_MAX_TAG_AREA = 11  # percent
SIM_CAMERA_FOV = 50  # degrees horizontal
SIM_MAX_POSE_YAW = 60  # if tag is more than 60 degrees rotated, we don't see it


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
        if onlyTagIds:
            print(f"camera {self.cameraName} only looking at tags {onlyTagIds}")
        else:
            print(f"camera {self.cameraName} looking at ALL tags")

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
            resultTime = result.getTimestampSeconds()
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


class PhotonTagCameraSim(Subsystem):
    def __init__(self, cameraName: str, aprilTagFieldLayout, cameraLocationOnDrivetrain: Pose2d, drivetrain):
        """
        :param drivetrain: drivetrain of the robot
        :param aprilTagFieldLayout: should be of type robotpy_apriltag.AprilTagFieldLayout
        :param cameraLocationOnDrivetrain: camera position on robot frame
        (for example if camera is in the front right corner of of a 0.8*0.8m robot looking 10 degrees left,
         set this it to Pose2d(Translation2d(0.4, -0.4), Rotation2d.fromDegrees(10)) )
        """
        super().__init__()

        assert cameraName, "cameraName must be specified"
        self.cameraName = cameraName
        self.onlyTagIds = None

        self.drivetrain = drivetrain
        self.field2d = None
        self.locationOnDrivetrain = Transform2d(Pose2d(0, 0, 0), cameraLocationOnDrivetrain)
        self.aprilTagLayout = aprilTagFieldLayout

        if hasattr(self.drivetrain, "field"):
            self.field2d: Field2d = self.drivetrain.field
            for tag in self.aprilTagLayout.getTags():
                self.drawTag(tag.ID, None)

        self.tx = 0.0
        self.ty = 0.0
        self.ta = 0.0
        self.hb = 0
        self.visibleTags = set()

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False
        self.pipelineIndex = -1
        self.driverMode = False

    def setOnlyTagIds(self, onlyTagIds=()):
        onlyTagIds = set(onlyTagIds) if onlyTagIds else None
        if onlyTagIds != self.onlyTagIds:
            self.heartbeating = False  # assume camera not ready after this change
        self.onlyTagIds = onlyTagIds
        if onlyTagIds:
            print(f"camera {self.cameraName} only looking at tags {onlyTagIds}")
        else:
            print(f"camera {self.cameraName} looking at ALL tags")

    def setPipeline(self, index: int):
        self.pipelineIndex = index

    def getPipeline(self) -> int:
        return self.pipelineIndex

    def setDriverMode(self, enabled: bool):
        self.driverMode = enabled

    def getDriverMode(self) -> bool:
        return self.driverMode

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

        cameraPose: Pose2d = self.drivetrain.getPose()
        cameraPose = cameraPose.transformBy(self.locationOnDrivetrain)

        # 1. if camera is connected, process the camera results
        self.lastHeartbeatTime = now
        self.ta, self.tx, self.ty = 0, 0, 0
        self.hb += 1
        biggest = 0

        visibleTags = set()
        for tag in self.aprilTagLayout.getTags():
            if self.onlyTagIds and tag.ID not in self.onlyTagIds:
                continue  # if we are only supposed to look at a few tag IDs, skip other tags

            directionToTag = Transform2d(cameraPose, tag.pose.toPose2d())
            vectorToTag = directionToTag.translation()
            if vectorToTag.norm() > SIM_MAX_TAG_DISTANCE:
                continue  # too far
            if abs(vectorToTag.angle().degrees()) > SIM_CAMERA_FOV / 2:
                continue  # won't fit into our 50 degree field of view

            facingUs = directionToTag.rotation().rotateBy(U_TURN - vectorToTag.angle())
            if abs(facingUs.degrees()) > SIM_MAX_POSE_YAW:
                continue  # tag not really facing us

            ty, tx = 0.1, -vectorToTag.angle().degrees()
            ta = 1.0 / math.pow(vectorToTag.norm(), 2)  # 1% of the screen => 1m away
            if ta / SIM_MAX_TAG_AREA > math.pow(1 - abs(tx) / (SIM_CAMERA_FOV / 2), 2):
                continue  # tag too close, we likely won't detect it well or at all
            ta *= facingUs.cos()

            # print(f"tag{tag.ID} at {vectorToTag.angle().degrees()}, visible at {facingUs.degrees()}")
            visibleTags.add(tag.ID)

            if ta > 0 and ta >= biggest:  # between multiple detections, choose the biggest
                self.tx = tx
                self.ty = ty
                self.ta = ta
                biggest = ta

        # 2. report the state of the camera
        heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING"))
        self.heartbeating = heartbeating

        # 3. display the tags which just appeared, hide those which disappeared
        if hasattr(self.drivetrain, "field"):
            field2d: Field2d = self.drivetrain.field
            for tagId in visibleTags:
                if tagId not in self.visibleTags:
                    self.drawTag(tagId, self.aprilTagLayout.getTagPose(tagId))
            for tagId in self.visibleTags:
                if tagId not in visibleTags:
                    self.drawTag(tagId, None)
        self.visibleTags = visibleTags


    def drawTag(self, tagId: int, pose3d: Pose3d):
        if self.field2d is None:
            return

        if pose3d is None:
            poses = []
        else:
            pose = pose3d.toPose2d()
            poses = [
                pose.transformBy(Transform2d(-0.09, 0, 0)),
                pose.transformBy(Transform2d(-0.08, 0, 0)),
                pose.transformBy(Transform2d(-0.07, 0, 0)),
                pose.transformBy(Transform2d(-0.06, 0, 0)),
                pose.transformBy(Transform2d(-0.05, 0, 0)),
                pose.transformBy(Transform2d(-0.04, 0, 0)),
                pose.transformBy(Transform2d(-0.03, 0, 0)),
                pose,
                pose.transformBy(Transform2d(0, 0.2, 0)),
                pose.transformBy(Transform2d(0, -0.2, 0)),
            ]

        name = f"{self.cameraName}-tag{tagId}"
        self.field2d.getObject(name).setPoses(poses)

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
