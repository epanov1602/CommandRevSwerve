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
