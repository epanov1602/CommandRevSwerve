# code taken from YouTube video: https://www.youtube.com/watch?v=wza2zjz49co

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
