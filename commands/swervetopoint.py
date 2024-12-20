#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2
import typing

from subsystems.drivesubsystem import DriveSubsystem
from commands.aimtodirection import AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d, Translation2d, Pose2d


class SwerveToPoint(commands2.Command):
    def __init__(self, x, y, headingDegrees, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True) -> None:
        self.targetPose = Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees))
        self.speed = speed
        self.stop = slowDownAtFinish
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        self.initialPosition = self.drivetrain.getPose().translation()
        self.initialDistance = self.initialPosition.distance(self.targetPose.translation())

    def execute(self):
        # 1. to which direction we should be pointing?
        currentPose = self.drivetrain.getPose()
        currentPoint = currentPose.translation()
        targetDirection = (self.targetPose - currentPose).translation().angle()
        degreesRemaining = (self.targetPose.rotation() - currentPose.rotation()).degrees()

        # 2. proportional control for turning ("rotation"):
        # if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = abs(self.speed)
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        # if we need to be turning *right* while driving, use negative rotation speed
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed

        # 3. same for driving ("translation"): if almost finished driving, drive slower (avoid overshooting)
        distanceRemaining = self.targetPose.translation().distance(currentPoint)
        proportionalTransSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        translateSpeed = abs(self.speed)  # if we don't plan to stop at the end, go at max speed
        if translateSpeed > proportionalTransSpeed and self.stop:
            translateSpeed = proportionalTransSpeed
        if translateSpeed < GoToPointConstants.kMinTranslateSpeed:
            translateSpeed = GoToPointConstants.kMinTranslateSpeed

        # 4. make a vector with this speed but pointing in target direction
        translateSpeedXY = Translation2d(translateSpeed, 0).rotateBy(targetDirection)

        # 5. drive with the decided (X, Y, Rotation) speed
        self.drivetrain.drive(translateSpeedXY.x, translateSpeedXY.y, turnSpeed, fieldRelative=True, rateLimit=False)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        currentPose = self.drivetrain.getPose()
        currentPosition = currentPose.translation()
        currentDirection = currentPose.rotation()

        # 2. did we overshoot?
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)
        if distanceFromInitialPosition >= self.initialDistance:  # we overshot in distance
            if not self.stop:
                return True  # case 1: overshot in distance and did not mean to stop at this point
            distanceFromTargetDirectionDegrees = (self.targetPose.rotation() - currentDirection).degrees()
            if abs(distanceFromTargetDirectionDegrees) < AimToDirectionConstants.kAngleToleranceDegrees:
                return True  # case 2: overshot in distance and target direction is correct

