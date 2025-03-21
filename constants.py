# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""


import math

import rev
from wpimath import units
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from rev import SparkBase, SparkBaseConfig, ClosedLoopConfig


class NeoMotorConstants:
    kFreeSpeedRpm = 5676


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.7
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 2.4  # radians per second
    kMagnitudeSlewRate = 3.6  # percent per second (1 = 100%)
    kRotationalSlewRate = 4.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(26.5)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # set it to True if you were using a ruler for zeroing and want to ignore the offsets below
    kAssumeZeroOffsets = True

    # set the above to == False, if you are using Rev zeroing tool (and you have to tinker with offsets below)
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 1
    kRearLeftDrivingCanId = 2
    kFrontRightDrivingCanId = 3
    kRearRightDrivingCanId = 4

    kFrontLeftTurningCanId = 5
    kRearLeftTurningCanId = 6
    kFrontRightTurningCanId = 7
    kRearRightTurningCanId = 8

    kLeadElevationCanId = 10
    kFollowElevationCanId = 11
    kArmLeadMotorCanId = 18
    kIntakeLeadMotorCanId = 19
    kIntakeRangefinderCanId = 33

    kGyroReversed = -1  # can be +1 if not flipped (affects field-relative driving)


def getSwerveDrivingMotorConfig() -> SparkBaseConfig:
    drivingConfig = SparkBaseConfig()
    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    drivingConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    return drivingConfig


def getSwerveTurningMotorConfig(turnMotorInverted: bool) -> SparkBaseConfig:
    turningConfig = SparkBaseConfig()
    turningConfig.inverted(turnMotorInverted)
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)
    turningConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    turningConfig.closedLoop.pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
    turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    turningConfig.closedLoop.positionWrappingEnabled(True)
    turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor)
    return turningConfig


class ModuleConstants:
    # WATCH OUT:
    #  - one or both of two constants below need to be flipped from True to False (by trial and error)
    #  , depending which swerve module you have (MK4i, MK4n, MAXSwerve, WCP, ThriftyBot, etc.)
    kTurningEncoderInverted = False
    kTurningMotorInverted = True

    kWheelDiameterMeters = units.inchesToMeters(4) * 0.875 * 1.05  # MK4i/MK4n: 4 inches, MAXSwerve: 3 inches
    # ^^ might need to be multiplied by 0.93 instead of 1.00 if we believe Eric's calibration

    # Calculations required for driving motor conversion factors and feed forward
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDrivingMotorReduction = 6.5  # MK4i/MK4n: 6.5, MAXSwerve: 4.714 for a 14 tooth pinion (greater for 13 or 12 tooth)

    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction  # meters

    kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1  # can be dialed down if you see oscillations in the turning motor
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkBase.IdleMode.kBrake
    kTurningMotorIdleMode = SparkBase.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp

    kDrivingMinSpeedMetersPerSecond = 0.01


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1
    kDriveDeadband = 0.05


class AutoConstants:
    kUseSqrtControl = True  # improves arrival time and precision for simple driving commands

    # below are really trajectory constants
    kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond * 0.9
    kMaxAccelerationMetersPerSecondSquared = 4
    kMaxAngularSpeedRadiansPerSecond = 2 * math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = 2 * math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 2

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )


def makePose(x, y, heading):
    return Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(heading))


class LeftFeeder:
    location = (1.285, 6.917, -54)
    pose = makePose(*location)
    tags = (1, 13)


class RightFeeder:
    location = (1.285, 1.135, +54)
    pose = makePose(*location)
    tags = (2, 12)


class RobotCameraLocations:
    kFrontLeft = Pose2d(Translation2d(x=0.37, y=+0.18), Rotation2d.fromDegrees(0))
    kFrontRight = Pose2d(Translation2d(x=0.37, y=-0.18), Rotation2d.fromDegrees(0))
    kRight = Pose2d(Translation2d(x=0.0, y=-0.30), Rotation2d.fromDegrees(-60))
    kRear = Pose2d(Translation2d(x=+0.05, y=0), Rotation2d.fromDegrees(-180))
