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
from wpimath.geometry import Rotation2d, Translation2d


class GoToPointConstants:
    kPTranslate = 2.0
    kMinTranslateSpeed = 0.07  # moving forward slower than this is unproductive
    kApproachRadius = 0.1  # within this radius from target location, try to point in initial direction
    kOversteerAdjustment = 0.5

class GoToPoint(commands2.Command):
    def __init__(self, x, y, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True) -> None:
        """
        Go to a point with (X, Y) coordinates. Whether this is the end of your trajectory or not.
        :param x:
        :param y:
        :param drivetrain:
        :param speed: between -1.0 and +1.0 (you can use negative speed to drive backwards)
        :param slowDownAtFinish:
        """
        self.targetPosition = Translation2d(x, y)
        self.speed = speed
        self.stop = slowDownAtFinish
        self.initialDirection = None
        self.initialDistance = None
        self.pointingInGoodDirection = False
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        self.initialPosition = self.drivetrain.getPose().translation()
        initialDirection = self.targetPosition - self.initialPosition
        self.initialDirection = Rotation2d(initialDirection.x, initialDirection.y)
        if self.speed < 0:
            self.initialDirection = self.initialDirection.rotateBy(GoToPoint.REVERSE_DIRECTION)
        self.initialDistance = self.initialPosition.distance(self.targetPosition)
        self.pointingInGoodDirection = False

    def execute(self):
        # 1. to which direction we should be pointing?
        currentPose = self.drivetrain.getPose()
        currentDirection = currentPose.rotation()
        currentPoint = currentPose.translation()
        targetDirectionVector = self.targetPosition - currentPoint
        targetDirection = Rotation2d(targetDirectionVector.x, targetDirectionVector.y)
        if self.speed < 0:
            targetDirection = targetDirection.rotateBy(GoToPoint.REVERSE_DIRECTION)
        degreesRemaining = (targetDirection - currentDirection).degrees()
        distanceRemaining = self.targetPosition.distance(currentPoint)

        # 2. if we are pointing in a very wrong direction (more than 45 degrees away), rotate away without moving
        if degreesRemaining > 45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, abs(self.speed))
            return
        elif degreesRemaining < -45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, -abs(self.speed))
            return

        self.pointingInGoodDirection = True

        # 3. otherwise, drive forward but with an oversteer adjustment (better way is to use RAMSETE unicycle)
        if distanceRemaining < GoToPointConstants.kApproachRadius:
            targetDirection = self.initialDirection  # avoid wiggling the direction when almost there
            degreesRemaining = (targetDirection - currentDirection).degrees()
        elif GoToPointConstants.kOversteerAdjustment != 0:
            deviationFromInitial = (targetDirection - self.initialDirection).degrees()
            adjustment = GoToPointConstants.kOversteerAdjustment * deviationFromInitial
            if adjustment > 20: adjustment = 20  # avoid oscillations by capping the adjustment at 20 degrees
            if adjustment < -20: adjustment = -20  # avoid oscillations by capping the adjustment at 20 degrees
            targetDirection = targetDirection.rotateBy(Rotation2d.fromDegrees(adjustment))
            degreesRemaining = (targetDirection - currentDirection).degrees()
            #SmartDashboard.putNumber("z-heading-target", targetDirection.degrees())

        # 4. now when we know the desired direction, we can compute the turn speed
        rotateSpeed = abs(self.speed)
        proportionalRotateSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if rotateSpeed > proportionalRotateSpeed:
            rotateSpeed = proportionalRotateSpeed

        # 5. but if not too different, then we can drive while turning
        proportionalTransSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        translateSpeed = abs(self.speed)  # if we don't plan to stop at the end, go at max speed
        if translateSpeed > proportionalTransSpeed and self.stop:
            translateSpeed = proportionalTransSpeed
        if translateSpeed < GoToPointConstants.kMinTranslateSpeed:
            translateSpeed = GoToPointConstants.kMinTranslateSpeed
        if self.speed < 0:
            translateSpeed = -translateSpeed  # negative translation speed if supposed to go in reverse

        # 6. if we need to be turning *right* while driving, use negative rotation speed
        if degreesRemaining < 0:
            self.drivetrain.arcadeDrive(translateSpeed, -rotateSpeed)
        else:  # otherwise, use positive
            self.drivetrain.arcadeDrive(translateSpeed, +rotateSpeed)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        # 1. did we reach the point where we must move very slow?
        currentPose = self.drivetrain.getPose()
        currentPosition = currentPose.translation()
        distanceRemaining = self.targetPosition.distance(currentPosition)
        translateSpeed = GoToPointConstants.kPTranslate * distanceRemaining

        # 1. have we reached the point where we are moving very slowly?
        tooSlowNow = translateSpeed < 0.125 * GoToPointConstants.kMinTranslateSpeed and self.stop

        # 2. did we overshoot?
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)
        if distanceFromInitialPosition >= self.initialDistance or tooSlowNow:
            return True  # we overshot or driving too slow

    REVERSE_DIRECTION = Rotation2d.fromDegrees(180)