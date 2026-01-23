#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Subsystem
from wpilib import DriverStation
from wpimath.geometry import Translation2d, Rotation2d
from subsystems.drivesubsystem import DriveSubsystem
from constants import LookupTable


# TODO : calibrate this lookup table on a real robot, and add more points
RECOMMENDED_SHOOTER_RPM_BY_DISTANCE = LookupTable({
    1.0 : 2000,  # if distance is 1m, spin at 2000 rpm
    2.0 : 3000,  # if distance is 2m, spin at 3000 rpm
    12.0 : 6000,  # if distance is 12m, spin at 6000 rpm
})

# TODO : calibrate this lookup table on a real robot, and add more points
RECOMMENDED_SHOOTER_HOOD_ANGLE_BY_DISTANCE = LookupTable({
    1.0 : 70.0,  # if distance is 1m, hood angle 70 degrees
    2.0 : 60.0,  # if distance is 2m, hood angle 60 degrees
    12.0 : 6000,  # if distance is 12m, hood angle 45 degrees
})



class FiringTable(Subsystem):
    """
    Tracks how far the goal is, recommends shooter speed (RPM), firing angle and direction
    """
    def __init__(
        self,
        drivetrain: DriveSubsystem,
        shooterLocationOnDrivetrain: Translation2d,
        goalIfBlue: Translation2d,
        goalIfRed: Translation2d,
        minimumRangeMeters: 0.0,
        maximumRangeMeters: 0.0,
    ) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.shooterLocationOnDrivetrain = shooterLocationOnDrivetrain
        self.goalIfBlue = goalIfBlue
        self.goalIfRed = goalIfRed
        self.minimumRangeMeters = minimumRangeMeters
        self.maximumRangeMeters = maximumRangeMeters

        self.goal = None
        self.vectorToGoal: Translation2d | None = None


    def recommendedShooterRpm(self):
        if self.vectorToGoal is None:
            return 0.0
        distanceMeters = self.distance()
        return RECOMMENDED_SHOOTER_RPM_BY_DISTANCE.interpolate(distanceMeters)


    def recommendedFiringAngleDegrees(self) -> float | None:
        if self.vectorToGoal is None:
            return None
        distanceMeters = self.distance()
        return RECOMMENDED_SHOOTER_HOOD_ANGLE_BY_DISTANCE.interpolate(distanceMeters)


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

