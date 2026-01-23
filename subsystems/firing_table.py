#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Subsystem
from wpilib import DriverStation
from wpimath.geometry import Translation2d, Rotation2d

from subsystems.drivesubsystem import DriveSubsystem


class FiringTable(Subsystem):
    """
    Tracks how far the goal is, recommends shooter speed (RPM), firing angle and direction
    """
    def __init__(
        self,
        drivetrain: DriveSubsystem,
        shooterLocationOnDrivetrain: Translation2d,
        goalIfBlue: Translation2d,
        goalIfRed: Translation2d
    ) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.shooterLocationOnDrivetrain = shooterLocationOnDrivetrain
        self.goalIfBlue = goalIfBlue
        self.goalIfRed = goalIfRed
        self.goal = None
        self.vectorToGoal: Translation2d | None = None


    def recommendedShooterRpm(self):
        if self.vectorToGoal is None:
            return 0.0
        distance = self.distance()

        # TODO: calibrate and improve this lookup table for firing speeds (in RPM)
        if distance < 3.0:
            return 3000.
        elif distance < 4.0:
            return 4000.
        elif distance < 5.0:
            return 5000.
        else:
            return 6000.


    def recommendedFiringAngleDegrees(self) -> float | None:
        if self.vectorToGoal is None:
            return None
        distanceMeters = self.distance()

        # TODO: calibrate and improve this lookup table for firing angles (in degrees)
        return 45 + 30 / distanceMeters


    def recommendedTurretDirection(self) -> Rotation2d | None:
        """
        If the goal is 30 degrees left, while the drivetrain is 10 degrees left... the turret should turn +20 degrees
        """
        if self.vectorToGoal is None:
            return None
        drivetrainPose = self.drivetrain.getPose()
        return self.vectorToGoal.angle() - drivetrainPose.rotation()


    def vector(self) -> Translation2d | None:
        return self.vectorToGoal

    def distance(self) -> float:
        return self.vectorToGoal.norm() if self.vectorToGoal is not None else 0

    def direction(self) -> Rotation2d:
        return self.vectorToGoal.angle() if self.vectorToGoal is not None else Rotation2d(0)

    def periodic(self):
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kRed:
            self.goal = self.goalIfRed
        else:
            self.goal = self.goalIfBlue
        pose = self.drivetrain.getPose()
        shooter = pose.translation() + self.shooterLocationOnDrivetrain.rotateBy(pose.rotation())
        self.vectorToGoal = self.goal - shooter
