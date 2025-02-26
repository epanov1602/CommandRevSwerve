
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

