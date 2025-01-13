#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2
import math

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d, Translation2d

from commands.aimtodirection import AimToDirectionConstants
from constants import AutoConstants

class GoToPointConstants:
    kPTranslate = 0.25  # make it 0.2 to be conservative?  # you will need to calibrate this one to your robot
    kUseSqrtControl = AutoConstants.kUseSqrtControl

    kMinTranslateSpeed = 0.035  # moving forward slower than this is unproductive
    kApproachRadius = 0.2  # within this radius from target location, try to point in desired direction
    kOversteerAdjustment = 0.5


class GoToPoint(commands2.Command):
    def __init__(self, x, y, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True, finishDirection=None) -> None:
        """
        Go to a point with (X, Y) coordinates. Whether this is the end of your trajectory or not.
        :param x:
        :param y:
        :param drivetrain:
        :param speed: between -1.0 and +1.0 (you can use negative speed to drive backwards)
        :param finishDirection: Rotation2d for robot direction at the finish point, example: Rotation2d.fromDegrees(-70)
        :param slowDownAtFinish:
        """
        super().__init__()
        self.targetPosition = Translation2d(x, y)
        self.initialPosition = None
        self.speed = speed
        self.stop = slowDownAtFinish
        self.desiredEndDirection = None
        self.initialDistance = None
        self.pointingInGoodDirection = False
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.finishDirection = finishDirection
        if self.speed < 0 and self.finishDirection is not None:
            self.finishDirection = self.finishDirection.rotateBy(GoToPoint.REVERSE_DIRECTION)

    def initialize(self):
        self.initialPosition = self.drivetrain.getPose().translation()
        if self.finishDirection is not None:
            self.desiredEndDirection = self.finishDirection
        else:
            initialDirection = self.targetPosition - self.initialPosition
            self.desiredEndDirection = Rotation2d(initialDirection.x, initialDirection.y)
        if self.speed < 0:
            self.desiredEndDirection = self.desiredEndDirection.rotateBy(GoToPoint.REVERSE_DIRECTION)
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
        degreesRemaining = _optimize((targetDirection - currentDirection).degrees())
        rotateSpeed = min([abs(self.speed), AimToDirectionConstants.kP * abs(degreesRemaining)])

        # 2. if we are pointing in a very wrong direction (more than 45 degrees away), rotate away without moving
        if degreesRemaining > 45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, rotateSpeed)
            return
        elif degreesRemaining < -45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, -rotateSpeed)
            return

        self.pointingInGoodDirection = True

        # 3. otherwise, drive forward but with an oversteer adjustment (better way is to use RAMSETE unicycle)
        distanceRemaining = self.targetPosition.distance(currentPoint)
        if distanceRemaining < GoToPointConstants.kApproachRadius:
            targetDirection = self.desiredEndDirection  # avoid wiggling the direction when almost there
            degreesRemaining = _optimize((targetDirection - currentDirection).degrees())

        elif GoToPointConstants.kOversteerAdjustment != 0:
            deviationFromInitial = _optimize((targetDirection - self.desiredEndDirection).degrees())
            adjustment = GoToPointConstants.kOversteerAdjustment * deviationFromInitial
            if adjustment > 30: adjustment = 30  # avoid oscillations by capping the adjustment at 30 degrees
            if adjustment < -30: adjustment = -30  # avoid oscillations by capping the adjustment at 30 degrees
            targetDirection = targetDirection.rotateBy(Rotation2d.fromDegrees(adjustment))
            degreesRemaining = _optimize((targetDirection - currentDirection).degrees())
            # SmartDashboard.putNumber("z-heading-target", targetDirection.degrees())

        # 4. now when we know the desired direction, we can compute the turn speed
        rotateSpeed = abs(self.speed)
        proportionalRotateSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalRotateSpeed = math.sqrt(0.5 * proportionalRotateSpeed)  # will match the non-sqrt value when 50% max speed
        if rotateSpeed > proportionalRotateSpeed:
            rotateSpeed = proportionalRotateSpeed

        # 5. but if not too different, then we can drive while turning
        proportionalTransSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        if GoToPointConstants.kUseSqrtControl:
            proportionalTransSpeed = math.sqrt(0.5 * proportionalTransSpeed)

        translateSpeed = abs(self.speed)  # if we don't plan to stop at the end, go at max speed
        if translateSpeed > proportionalTransSpeed and self.stop:
            translateSpeed = proportionalTransSpeed  # if we plan to stop at the end, slow down when close
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
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)

        if not self.stop and distanceFromInitialPosition > self.initialDistance - GoToPointConstants.kApproachRadius:
            return True  # close enough

        distanceRemaining = self.targetPosition.distance(currentPosition)
        translateSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        if GoToPointConstants.kUseSqrtControl:
            translateSpeed = math.sqrt(0.5 * translateSpeed)

        # 1. have we reached the point where we are moving very slowly?
        tooSlowNow = translateSpeed < 0.125 * GoToPointConstants.kMinTranslateSpeed and self.stop

        # 2. did we overshoot?
        if distanceFromInitialPosition >= self.initialDistance or tooSlowNow:
            return True  # we overshot or driving too slow

    REVERSE_DIRECTION = Rotation2d.fromDegrees(180)


def _optimize(degrees):
    while degrees > 180:  # for example, if we have 350 degrees to turn left, we probably want -10 degrees right
        degrees -= 360

    while degrees < -180:  # for example, if we have -350 degrees to turn right, we probably want +10 degrees left
        degrees += 360

    return degrees