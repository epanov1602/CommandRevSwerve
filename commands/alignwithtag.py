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
from wpilib import Timer

from subsystems.limelight_camera import LimelightCamera


class AlignWithTag(commands2.Command):
    FINISH_ALIGNMENT_EXTRA_SECONDS = 0.5  # at least 0.5 seconds to finish the alignment
    TOLERANCE_METERS = 0.025  # one inch tolerance for alignment

    def __init__(self,
                 camera,
                 drivetrain,
                 specificHeadingDegrees=None,
                 speed=0.2,
                 reverse=False,
                 pushForwardSeconds=0.0,
                 pushForwardSpeed=0.1):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specificHeadingDegrees: do you want the robot to face in a very specific direction
        :param speed: if the camera is on the back of the robot, please use negative speed
        :param pushForwardSeconds: if you want the robot to push forward at the end of alignment
        :param reverse: set it =True if the camera is on the back of the robot (not front)
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

        # setting the target angle in a way that works for all cases
        self.targetDegrees = specificHeadingDegrees
        if specificHeadingDegrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(specificHeadingDegrees):
            self.targetDegrees = lambda: specificHeadingDegrees

        # state
        self.targetDirection = None
        self.alignedToTag = False
        self.pushForwardCommand = None
        self.haveNotSeenObjectSinceTime = None
        self.everSawObject = False


    def initialize(self):
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())
        self.alignedToTag = False
        self.pushForwardCommand = None
        self.haveNotSeenObjectSinceTime = Timer.getFPGATimestamp()
        self.everSawObject = False


    def isFinished(self) -> bool:
        now = Timer.getFPGATimestamp()
        if now > self.haveNotSeenObjectSinceTime + 3.0:
            print("AlignSwerveWithTag: finished because have not seen the object for >3 seconds")
            return True
        if self.alignedToTag and self.pushForwardCommand is None:
            print("AlignSwerveWithTag: finished because aligned to the tag and don't need to push forward")
            return True
        if self.alignedToTag and self.pushForwardCommand is not None and self.pushForwardCommand.isFinished():
            print("AlignSwerveWithTag: finished because aligned to the tag and the forward push is finished")
            return True


    def end(self, interrupted: bool):
        if self.pushForwardCommand is not None:
            self.pushForwardCommand.end(interrupted)
        self.drivetrain.arcadeDrive(0, 0)


    def execute(self):
        if self.camera.hasDetection():
            self.haveNotSeenObjectSinceTime = Timer.getFPGATimestamp()
            self.everSawObject = True

        # 0. are we pushing forward already?
        if self.alignedToTag and self.pushForwardCommand is not None:
            self.pushForwardCommand.execute()
            return

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
        #swerveSpeed = 0
        #if abs(degreesRemaining) < 4 * AimToDirectionConstants.kAngleToleranceDegrees or abs(turnSpeed) < AimToDirectionConstants.kMinTurnSpeed:
        swerveSpeed = self.getSwerveLeftSpeed(degreesRemaining)

        # 4. if we just aligned the heading and the swerve axis and should be pushing forward, make that push
        if not self.alignedToTag:
            self.drivetrain.drive(0, swerveSpeed, turnSpeed, fieldRelative=False, rateLimit=False)
        elif self.pushForwardCommand is None and self.pushForwardSeconds > 0:
            self.pushForwardCommand = self.getPushForwardCommand()
            self.pushForwardCommand.initialize()


    def getTurnSpeed(self, degreesRemaining):
        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed
        return turnSpeed


    def getPushForwardCommand(self):
        speed = self.pushForwardSpeed if not self.reverse else -self.pushForwardSpeed
        command = AimToDirection(degrees=None, drivetrain=self.drivetrain, fwd_speed=speed)
        return command.withTimeout(self.pushForwardSeconds)


    def getSwerveLeftSpeed(self, degreesRemaining):
        swerveSpeed = 0.0
        secondsSinceHeartbeat = self.camera.getSecondsSinceLastHeartbeat()
        objectSizePercent = self.camera.getA()
        objectXDegrees = self.camera.getX()
        if secondsSinceHeartbeat > 0.25:
            print(f"AlignSwerveWithTag: camera not usable (dead or too few frames per second), we see {secondsSinceHeartbeat} seconds since last hearbeat")
        elif objectXDegrees == 0 or objectSizePercent <= 0:
            print(f"AlignSwerveWithTag: invalid camera detection (objectX, objectSize) = ({objectXDegrees}, {objectSizePercent})")
            if not self.alignedToTag and self.everSawObject:
                now = Timer.getFPGATimestamp()
                if now > self.haveNotSeenObjectSinceTime + 1.5:
                    print(f"AlignSwerveWithTag: blind {now - self.haveNotSeenObjectSinceTime} seconds, too close??")
                    self.alignedToTag = True
        else:
            swerveSpeed, objectXMeters = self.calculateSwerveLeftSpeed(objectSizePercent, objectXDegrees)
            if not self.alignedToTag and abs(swerveSpeed) <= GoToPointConstants.kMinTranslateSpeed:
                print(f"AlignSwerveWithTag: almost done, since swerve speed {swerveSpeed} is already small")
                if abs(objectXMeters) <= AlignWithTag.TOLERANCE_METERS:
                    print(f"AlignSwerveWithTag: objectXMeters={objectXMeters} is small enough too")
                    if abs(degreesRemaining) <= AimToDirectionConstants.kAngleToleranceDegrees:
                        print(f"AlignSwerveWithTag: degreesRemaining={degreesRemaining} is small, we are done aligning")
                        self.alignedToTag = True
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
        proportionalSpeed = 0.33 * GoToPointConstants.kPTranslate * abs(objectXMeters)
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