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

#    def setPipeline(self, index: int):
#        self.pipelineIndexRequest.set(index)

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
