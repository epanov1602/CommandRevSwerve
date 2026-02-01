
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from typing import Tuple

from wpilib import Timer, SmartDashboard
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpimath.geometry import Rotation2d
from wpinet import PortForwarder


class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str, isUsb0=False) -> None:
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
        self.ticked = False

        self.takingSnapshotsWhenNoDetection = 0.0
        self.snapshotRequest = self.table.getIntegerTopic("snapshot").publish()
        self.snapshotRequestValue = self.table.getIntegerTopic("snapshot").getEntry(0).get()
        self.lastSnapshotRequestTime = 0.0

        # localizer state
        self.localizerSubscribed = False
        self.cameraPoseSetRequest, self.robotOrientationSetRequest, self.imuModeRequest = None, None, None

        # port forwarding in case this is connected over USB
        if isUsb0:
            for port in [1180, 5800, 5801, 5802, 5803, 5804, 5805, 5806, 5807, 5808, 5809]:
                PortForwarder.getInstance().add(port, "172.29.0.1", port)


    def addLocalizer(self):
        if self.localizerSubscribed:
            return

        self.localizerSubscribed = True
        # if we want MegaTag2 localizer to work, we need to be publishing two things (to the camera):
        #   1. what robot's yaw is ("yaw=0 degrees" means "facing North", "yaw=90 degrees" means "facing West", etc.)
        #   2. where is this camera sitting on the robot (e.g. y=-0.2 meters to the right, x=0.1 meters fwd from center)
        self.robotOrientationSetRequest = self.table.getDoubleArrayTopic("robot_orientation_set").publish()
        self.cameraPoseSetRequest = self.table.getDoubleArrayTopic("camerapose_robotspace_set").publish()
        self.imuModeRequest = self.table.getIntegerTopic("imumode_set").publish()  # this is only for Limelight 4

        # and we can then receive the localizer results from the camera back
        self.botPose = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([])
        self.botPoseFlipped = self.table.getDoubleArrayTopic("botpose_orb_wpired").getEntry([])


    def setCameraPoseOnRobot(self, x, y, z, pitchDegrees, rollDegrees, yawDegrees):
        """
        Only for localization: angles should be in degrees, xyz in meters
        """
        if self.cameraPoseSetRequest is not None:
            self.cameraPoseSetRequest.set([x, -y, z, pitchDegrees, rollDegrees, yawDegrees])
            self.imuModeRequest.set(0)  # TODO: try 4, this can be a better choice for Limelight 4
            # 0 - use external imu (the only option available on Limelight 3)
            # 1 - use external imu, seed internal imu
            # 2 - use internal
            # 3 - use internal with MT1 assisted convergence
            # 4 - use internal IMU with external IMU assisted convergence


    def updateRobotHeading(self, now: float, heading: Rotation2d):
        """
        Only for localization
        """
        if self.robotOrientationSetRequest is not None:
            self.robotOrientationSetRequest.set([heading.degrees() % 360, 0.0, 0.0, 0.0, 0.0, 0.0])

    def getXYAPositionEstimate(self, flipped=False) -> Tuple[float, float, float, float]:
        """
        :return: estimated X, Y, area taken by the biggest tag, tag count
        """
        pose = self.botPoseFlipped.get() if flipped else self.botPose.get()
        if len(pose) >= 11:
            # Translation (X,Y,Z), Rotation(Roll,Pitch,Yaw) in degrees, total latency ms (cl+tl)
            # , tag count, tag span, average tag distance from camera, average tag area (percentage of image)
            x, y, z, roll, pitch, yaw, latency, count, span, distance, area = pose[0:11]
            return x, y, area, count

        else:
            return 0.0, 0.0, 0.0, 0.0


    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))
        self.heartbeating = False  # wait until the next heartbeat before saying self.haveDetection == true

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
        self.ticked = False
        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now
            self.ticked = True
        heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING"))
        self.heartbeating = heartbeating

        if heartbeating and self.takingSnapshotsWhenNoDetection and not self.hasDetection():
            if now > self.lastSnapshotRequestTime + self.takingSnapshotsWhenNoDetection:
                self.snapshotRequestValue += 1
                self.snapshotRequest.set(self.snapshotRequestValue)
                self.lastSnapshotRequestTime = now + self.takingSnapshotsWhenNoDetection

    def startTakingSnapshotsWhenNoDetection(self, secondsBetweenSnapshots=1.0):
        self.takingSnapshotsWhenNoDetection = secondsBetweenSnapshots

    def stopTakingSnapshotsWhenNoDetection(self):
        self.takingSnapshotsWhenNoDetection = 0.0


def _fix_name(name: str):
    if not name:
        name = "limelight"
    return name
