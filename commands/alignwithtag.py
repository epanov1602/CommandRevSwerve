#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import math
import commands2

from commands.aimtodirection import AimToDirectionConstants, AimToDirection
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d
from wpilib import Timer, SmartDashboard

from commands.swervetopoint import SwerveToSide
from constants import DriveConstants


class AlignWithTag(commands2.Command):
    TOLERANCE_METERS = 0.025  # one inch tolerance for alignment
    KP_MULT = 0.3

    def __init__(self,
                 camera,
                 drivetrain,
                 specificHeadingDegrees=None,
                 speed=0.2,
                 reverse=False,
                 pushForwardSeconds=0.0,
                 pushForwardSpeed=0.1,
                 detectionTimeoutSeconds=2.0,
                 cameraMinimumFps=4.0):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specificHeadingDegrees: do you want the robot to face in a very specific direction
        :param speed: if the camera is on the back of the robot, please use negative speed
        :param pushForwardSeconds: if you want the robot to push forward at the end of alignment
        :param reverse: set it =True if the camera is on the back of the robot (not front)
        :param detectionTimeoutSeconds: if no detection within this many seconds, assume the tag is lost
        :param cameraMinimumFps: what is the minimal number of **detected** frames per second expected from this camera
        """
        super().__init__()
        assert hasattr(camera, "getX"), "camera must have `getX()` to give us the object coordinate (in degrees)"
        assert hasattr(camera, "getA"), "camera must have `getA()` to give us object size (in % of screen)"
        assert hasattr(camera, "getSecondsSinceLastHeartbeat"), "camera must have a `getSecondsSinceLastHeartbeat()`"
        assert hasattr(drivetrain, "drive"), "drivetrain must have a `drive()` function, because we need a swerve drive"
        assert pushForwardSeconds >= 0, f"pushForwardSeconds={pushForwardSeconds}, but must be >=0"
        if pushForwardSeconds > 0:
            assert pushForwardSpeed > 0, f"if pushForwardSeconds={pushForwardSeconds} is not zero, pushForwardSpeed must be positive (not {pushForwardSpeed})"

        self.drivetrain = drivetrain
        self.camera = camera

        self.addRequirements(drivetrain)
        self.addRequirements(camera)

        self.reverse = reverse
        self.speed = min((1.0, abs(speed)))  # ensure that the speed is between 0.0 and 1.0
        self.pushForwardSeconds = pushForwardSeconds
        self.pushForwardSpeed = pushForwardSpeed

        assert detectionTimeoutSeconds > 0, f"non-positive detectionTimeoutSeconds={detectionTimeoutSeconds}"
        self.detectionTimeoutSeconds = detectionTimeoutSeconds

        assert cameraMinimumFps > 0, f"non-positive cameraMinimumFps={cameraMinimumFps}"
        self.frameTimeoutSeconds = 1.0 / cameraMinimumFps

        # setting the target angle in a way that works for all cases
        self.targetDegrees = specificHeadingDegrees
        if specificHeadingDegrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(specificHeadingDegrees):
            self.targetDegrees = lambda: specificHeadingDegrees

        # state
        self.targetDirection = None
        self.lastSeenObjectTime = None
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.everSawObject = False
        self.tAlignedToTag = 0.0  # time when aligned to the tag and desired direction for the first time
        self.lostTag = ""
        self.finished = ""


    def initialize(self):
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())
        self.tAlignedToTag = 0.0  # time when aligned to the tag and desired direction
        self.lostTag = False
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenObjectTime = Timer.getFPGATimestamp()
        self.everSawObject = False
        self.finished = ""
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")


    def isFinished(self) -> bool:
        if self.finished:
            return True

        now = Timer.getFPGATimestamp()

        # bad ways to finish
        if self.lostTag:
            self.finished = self.lostTag
        elif now > self.lastSeenObjectTime + self.detectionTimeoutSeconds + self.pushForwardSeconds:
            delay = now - self.lastSeenObjectTime
            self.finished = f"not seen {int(1000 * delay)}ms"

        # good ways to finish
        elif self.tAlignedToTag != 0 and now > self.tAlignedToTag + self.pushForwardSeconds:
            self.finished = "algnd+ppushed" if self.pushForwardSeconds > 0 else "aligned"

        if not self.finished:
            return False

        SmartDashboard.putString("command/c" + self.__class__.__name__, self.finished)
        return True


    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")


    def execute(self):
        now = Timer.getFPGATimestamp()
        if self.camera.hasDetection():
            x = self.camera.getX()
            a = self.camera.getA()
            if x != 0 and a > 0:
                self.lastSeenObjectTime = now
                self.lastSeenObjectSize = a
                self.lastSeenObjectX = x
                self.everSawObject = True

        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 2. use that to calculate the turn speed
        turnSpeed = self.getTurnSpeed(degreesRemaining)

        # 3. if the robot heading is almost aligned, start swerving right or left (for centering on that tag precisely)
        swerveSpeed = self.getSwerveSpeed(degreesRemaining)

        # 4. if we just aligned and no forward move is needed
        if self.tAlignedToTag != 0 and self.pushForwardSeconds == 0:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "no need to push")
            self.drivetrain.stop()
            return

        # 6. if aligned, push forward
        fwdSpeed = 0.0
        if self.tAlignedToTag != 0:
            fwdSpeed = (now - self.tAlignedToTag) * DriveConstants.kMagnitudeSlewRate
            if abs(fwdSpeed) > self.pushForwardSpeed:
                fwdSpeed = self.pushForwardSpeed
            if self.reverse:
                fwdSpeed = -fwdSpeed

        self.drivetrain.drive(fwdSpeed, swerveSpeed, turnSpeed, fieldRelative=False, rateLimit=False)


    def getTurnSpeed(self, degreesRemaining):
        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AlignWithTag.KP_MULT * AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed
        return turnSpeed


    def getSwerveSpeed(self, degreesRemaining):
        now = Timer.getFPGATimestamp()
        objectXDegrees = self.lastSeenObjectX
        objectSizePercent = self.lastSeenObjectSize

        timeSinceLastHeartbeat = self.camera.getSecondsSinceLastHeartbeat()
        if timeSinceLastHeartbeat > self.frameTimeoutSeconds:
            self.lostTag = f"no camera heartbeat > {int(1000 * timeSinceLastHeartbeat)}ms"
            return 0.0

        timeSinceLastDetection = now - self.lastSeenObjectTime
        if timeSinceLastDetection > self.detectionTimeoutSeconds:
            if self.tAlignedToTag == 0:
                self.lostTag = f"not aligned and no detection for {int(1000 * timeSinceLastDetection)}ms"
                return 0.0
            if timeSinceLastDetection > self.detectionTimeoutSeconds + self.pushForwardSeconds:
                self.lostTag = f"aligned but no detection for {int(1000 * timeSinceLastDetection)}ms"
                return 0.0

        if self.tAlignedToTag != 0 and timeSinceLastDetection > 0.5 * self.frameTimeoutSeconds:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "not fresh")
            return 0.0  # the last detection we know is not fresh, no need to use it

        if not (objectSizePercent > 0):
            SmartDashboard.putString("command/c" + self.__class__.__name__, "not yet acquired")
            return 0.0  # the object is not there, perhaps temporarily?

        swerveSpeed, objectXMeters = self.calculateSwerveLeftSpeed(objectSizePercent, objectXDegrees)
        if self.tAlignedToTag == 0 and abs(swerveSpeed) <= GoToPointConstants.kMinTranslateSpeed:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "90% aligned")
            print(f"AlignWithTag: almost done, since swerve speed {swerveSpeed} is already small")
            if abs(objectXMeters) <= AlignWithTag.TOLERANCE_METERS:
                SmartDashboard.putString("command/c" + self.__class__.__name__, "99% aligned")
                print(f"AlignWithTag: objectXMeters={objectXMeters} is small enough too")
                if abs(degreesRemaining) <= AimToDirectionConstants.kAngleToleranceDegrees:
                    SmartDashboard.putString("command/c" + self.__class__.__name__, "aligned")
                    print(f"AlignWithTag: degreesRemaining={degreesRemaining} is small, we are done aligning")
                    self.tAlignedToTag = Timer.getFPGATimestamp()

        return swerveSpeed


    def calculateSwerveLeftSpeed(self, objectSizePercent, objectXDegrees):
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        distanceMeters = math.sqrt(0.2 * 0.2 / (1.33 * 0.01 * objectSizePercent))

        # trigonometry: how many meters on the left is our object? (if negative, then it's on the right)
        objectXMeters = -distanceMeters * Rotation2d.fromDegrees(objectXDegrees).tan()
        # if the camera is on the back of the robot, then right and left are swapped
        if self.reverse:
            objectXMeters = -objectXMeters

        # how fast should we swerve to the right or left? use proportional control!
        swerveSpeed = self.speed
        proportionalSpeed = AlignWithTag.KP_MULT * GoToPointConstants.kPTranslate * abs(objectXMeters)
        if GoToPointConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)
        if proportionalSpeed < swerveSpeed:
            swerveSpeed = proportionalSpeed
        if swerveSpeed < GoToPointConstants.kMinTranslateSpeed:
            swerveSpeed = GoToPointConstants.kMinTranslateSpeed

        # if the object is on the right, swerve to the right (negative swerve speed)
        if objectXMeters < 0:
            swerveSpeed = -swerveSpeed

        return swerveSpeed, objectXMeters
