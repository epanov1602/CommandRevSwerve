#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2
import math

from subsystems.drivesubsystem import DriveSubsystem
from commands.aimtodirection import AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d, Translation2d, Pose2d


class SwerveToPoint(commands2.Command):
    def __init__(self, x, y, headingDegrees, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True) -> None:
        super().__init__()
        self.targetPose = None
        self.targetPoint = Translation2d(x, y)
        if isinstance(headingDegrees, Rotation2d):
            self.targetHeading = headingDegrees
        elif headingDegrees is not None:
            self.targetHeading = Rotation2d.fromDegrees(headingDegrees)
        else:
            self.targetHeading = None

        self.speed = speed
        self.stop = slowDownAtFinish
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.initialPosition = None
        self.initialDistance = None
        self.overshot = False

    def initialize(self):
        initialPose = self.drivetrain.getPose()
        self.initialPosition = initialPose.translation()

        targetHeading = initialPose.rotation() if self.targetHeading is None else self.targetHeading
        self.targetPose = Pose2d(self.targetPoint, targetHeading)

        self.initialDistance = self.initialPosition.distance(self.targetPose.translation())
        self.overshot = False

    def execute(self):
        currentXY = self.drivetrain.getPose()
        xDistance, yDistance = self.targetPose.x - currentXY.x, self.targetPose.y - currentXY.y
        totalDistance = self.targetPose.translation().distance(currentXY.translation())

        totalSpeed = GoToPointConstants.kPTranslate * totalDistance
        if GoToPointConstants.kUseSqrtControl:
            totalSpeed = math.sqrt(0.5 * totalSpeed)

        if totalSpeed > abs(self.speed):
            totalSpeed = abs(self.speed)
        if totalSpeed < GoToPointConstants.kMinTranslateSpeed:
            totalSpeed = GoToPointConstants.kMinTranslateSpeed

        # distribute the total speed between x speed and y speed
        xSpeed, ySpeed = 0, 0
        if totalDistance > 0:
            xSpeed = totalSpeed * xDistance / totalDistance
            ySpeed = totalSpeed * yDistance / totalDistance

        degreesLeftToTurn = self.getDegreesLeftToTurn()
        turningSpeed = 0.5 * abs(degreesLeftToTurn) * AimToDirectionConstants.kP
        if AimToDirectionConstants.kUseSqrtControl:
            turningSpeed = math.sqrt(0.5 * turningSpeed)  # will match the non-sqrt value when 50% max speed
        if turningSpeed > abs(self.speed):
            turningSpeed = abs(self.speed)
        if degreesLeftToTurn < 0:
            turningSpeed = -turningSpeed

        # now rotate xSpeed and ySpeed into robot coordinates
        speed = Translation2d(x=xSpeed, y=ySpeed).rotateBy(-self.drivetrain.getHeading())

        self.drivetrain.drive(speed.x, speed.y, turningSpeed, fieldRelative=False, rateLimit=False)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        currentPose = self.drivetrain.getPose()
        currentPosition = currentPose.translation()

        # did we overshoot?
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)
        if not self.stop and distanceFromInitialPosition > self.initialDistance - 3 * GoToPointConstants.kApproachRadius:
            return True  # close enough

        if distanceFromInitialPosition > self.initialDistance:
            self.overshot = True

        if self.overshot:
            distanceFromTargetDirectionDegrees = self.getDegreesLeftToTurn()
            if abs(distanceFromTargetDirectionDegrees) < 3 * AimToDirectionConstants.kAngleToleranceDegrees:
                return True  # case 2: overshot in distance and target direction is correct

    def getDegreesLeftToTurn(self):
        # can we get rid of this function by using Rotation2d? probably we can

        currentHeading = self.drivetrain.getPose().rotation()
        degreesLeftToTurn = (self.targetPose.rotation() - currentHeading).degrees()

        # if we have +350 degrees left to turn, this really means we have -10 degrees left to turn
        while degreesLeftToTurn > 180:
          degreesLeftToTurn -= 360

        # if we have -350 degrees left to turn, this really means we have +10 degrees left to turn
        while degreesLeftToTurn < -180:
          degreesLeftToTurn += 360

        return degreesLeftToTurn


class SwerveToSide(commands2.Command):
    def __init__(self, metersToTheLeft: float, drivetrain: DriveSubsystem, speed=1.0) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.speed = speed
        self.metersToTheLeft = metersToTheLeft
        self.subcommand = None

    def initialize(self):
        position = self.drivetrain.getPose()
        heading = position.rotation()
        target = position.translation() + Translation2d(x=0, y=self.metersToTheLeft).rotateBy(heading)
        self.subcommand = SwerveToPoint(
            x=target.x, y=target.y, headingDegrees=heading.degrees(), drivetrain=self.drivetrain, speed=self.speed
        )
        self.subcommand.initialize()

    def isFinished(self) -> bool:
        return self.subcommand.isFinished()

    def execute(self):
        return self.subcommand.execute()

    def end(self, interrupted: bool):
        self.subcommand.end(interrupted)
