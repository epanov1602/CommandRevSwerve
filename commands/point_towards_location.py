#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import commands2

from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Translation2d

from subsystems.drivesubsystem import DriveSubsystem


class PointTowardsLocation(commands2.Command):
    """
    One can use this command to have their swerve robot keep pointing towards some location as it moves around.
    Example (this can go into robotcontaier.py, inside of configureButtonBindings() function):

        ```
            from commands.point_towards_location import PointTowardsLocation

            # create a command for keeping the robot nose pointed towards the hub
            keepPointingTowardsHub = PointTowardsLocation(
                drivetrain=self.robotDrive,
                location=Translation2d(x=4.59, y=4.025),
                locationIfRed=Translation2d(x=11.88, y=4.025),
            )

            # setup a condition for when to do this: do it when the joystick right trigger is pressed by more than 50%
            whenRightTriggerPressed = self.driverController.axisGreaterThan(
                XboxController.Axis.kRightTrigger, threshold=0.5
            )

            # connect the command to its trigger
            whenRightTriggerPressed.whileTrue(keepPointingTowardsHub)

        ```
    """

    def __init__(self, drivetrain: DriveSubsystem, location: Translation2d, locationIfRed: Translation2d):
        super().__init__()
        self.location, self.locationIfRed = location, locationIfRed
        self.drivetrain = drivetrain  # not calling addRequirement, on purpose

        self.activeTargetLocation: Translation2d | None = None
        self.active = False


    def initialize(self):
        self.active = False
        color = DriverStation.getAlliance()
        if color == DriverStation.Alliance.kRed:
            self.activeTargetLocation = self.locationIfRed
            SmartDashboard.putString("command/c" + self.__class__.__name__, "assuming red alliance")
        else:
            self.activeTargetLocation = self.location
            SmartDashboard.putString("command/c" + self.__class__.__name__, "assuming blue or unknown alliance")


    def execute(self):
        # heading override already in place?
        if self.active:
            return

        # try to place that heading override now
        if self.drivetrain.startOverrideToFaceThisPoint(self.activeTargetLocation):
            self.active = True
            SmartDashboard.putString(
                "command/c" + self.__class__.__name__,
                f"pointing to x, y: {self.activeTargetLocation.x}, {self.activeTargetLocation.y}")


    def end(self, interrupted: bool):
        if self.active:
            self.drivetrain.stopOverrideToFaceThisPoint(self.activeTargetLocation)
        SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")


    def isFinished(self) -> bool:
        return False  # never finish, wait for user to stop this command

