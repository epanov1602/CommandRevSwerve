# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""


import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

import rev
from rev import SparkBase, SparkBaseConfig, ClosedLoopConfig, FeedbackSensor


class NeoMotorConstants:
    kFreeSpeedRpm = 5676


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = 2 * math.tau  # radians per second

    kMagnitudeSlewRate = 9.8  # m/s per second (1g acceleration allowed along X and along Y axis)
    kRotationalSlewRate = 12.0  # rad/s per second

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

    # MOTOR CAN IDs
    kFrontLeftDrivingCanId = 1
    kRearLeftDrivingCanId = 2
    kFrontRightDrivingCanId = 3
    kRearRightDrivingCanId = 4

    kFrontLeftTurningCanId = 5
    kRearLeftTurningCanId = 6
    kFrontRightTurningCanId = 7
    kRearRightTurningCanId = 8

    # CANCODER CAN IDs
    kFrontLeftCancoderCanId = 1
    kRearLeftCancoderCanId = 2
    kFrontRightCancoderCanId = 3
    kRearRightCancoderCanId = 4

    kGyroReversed = -1  # can be +1 if not flipped (affects field-relative driving)
    kGyroIsPigeon = False


def getSwerveDrivingMotorConfig() -> SparkBaseConfig:
    drivingConfig = SparkBaseConfig()
    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    drivingConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    return drivingConfig


def getSwerveTurningMotorConfig(turnMotorInverted: bool, useAbsoluteEncoderGoals: bool = True) -> SparkBaseConfig:
    turningConfig = SparkBaseConfig()
    turningConfig.inverted(turnMotorInverted)
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)

    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)

    turningConfig.encoder.positionConversionFactor(1.0)  # rotations, to match Kraken
    turningConfig.encoder.velocityConversionFactor(1.0 / 60)  # rotations per second

    if useAbsoluteEncoderGoals:
        turningConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        turningConfig.closedLoop.pid(ModuleConstants.kTurningP, 0.0, ModuleConstants.kTurningD)
    else:
        turningConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        turningConfig.closedLoop.pid(ModuleConstants.kTurningP * math.tau, 0.0, ModuleConstants.kTurningD)

    turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    turningConfig.closedLoop.positionWrappingEnabled(True)
    turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor)
    return turningConfig


class ModuleConstants:
    kDrivingMotorIsTalon = True
    kTurningMotorIsTalon = True
    kTurningKalmanGain = 0.05  # if using CANcoder, how quickly to fuse the absolute and relative angles
    kWheelDiameterMeters = 0.0762

    kTurningReductionRatio = 287 / 11.0  # for MK5n
    # (others: MK5i 260/10.0, MK4i 150/7.0, MK4n 18.75/1.0, Thrifty 25/1, MAXSwerve 9424/203.0, Plummer 28/1)

    # WATCH OUT:
    #  - one or both of two constants below need to be flipped from True to False (by trial and error)
    #  , depending which swerve module you have (MK4i, MK4n, Rev, WCP, ThriftyBot, etc)
    kTurningEncoderInverted = False
    kTurningMotorInverted = True

    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 14

    # Calculations required for driving motor conversion factors and feed forward
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

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
    kDriveDeadband = 0.05


class AutoConstants:
    kUseSqrtControl = True  # improves arrival time and precision for simple driving commands

    # below are really trajectory constants
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 0.67

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )
