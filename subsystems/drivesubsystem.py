import math
import typing

import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)
from wpilib import SmartDashboard, Field2d


from constants import DriveConstants, ModuleConstants
import swerveutils
from .maxswervemodule import MAXSwerveModule
from rev import CANSparkMax, CANSparkFlex
import navx


class DriveSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        enabledChassisAngularOffset = 0 if DriveConstants.kAssumeZeroOffsets else 1

        # Create MAXSwerveModules
        self.frontLeft = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=CANSparkFlex,
        )

        self.frontRight = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=CANSparkFlex,
        )

        self.rearLeft = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=CANSparkFlex,
        )

        self.rearRight = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=CANSparkFlex,
        )

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
        )
        self.odometryHeadingOffset = Rotation2d(0)
        self.resetOdometry(Pose2d(0, 0, 0))

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)


    def periodic(self) -> None:
        # Update the odometry in the periodic block
        pose = self.odometry.update(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
        )
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("heading", pose.rotation().degrees())
        self.field.setRobotPose(pose)

    def getHeading(self) -> Rotation2d:
        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.gyro.reset()
        self.odometry.resetPosition(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
            pose,
        )
        self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)


    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
        self.odometry.resetPosition(
            pose.rotation() - self.odometryHeadingOffset,
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
            newPose,
        )
        self.odometryHeadingOffset += dRot

    def arcadeDrive(
        self,
        xSpeed: float,
        rot: float,
        assumeManualInput: bool = False,
    ) -> None:
        if assumeManualInput:
            xSpeed = xSpeed * abs(xSpeed)
            rot = rot * abs(rot)
        self.drive(xSpeed, 0, rot, False, False)

    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param rotSpeed: rotation speed
        """
        self.arcadeDrive(0, rotSpeed)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        rateLimit: bool,
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """

        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if rateLimit:
            # Convert XY to polar for rate limiting
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.hypot(xSpeed, ySpeed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(
                    DriveConstants.kDirectionSlewRate / self.currentTranslationMag
                )
            else:
                directionSlewRate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            currentTime = wpilib.Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.prevTime
            angleDif = swerveutils.angleDifference(
                inputTranslationDir, self.currentTranslationDir
            )
            if angleDif < 0.45 * math.pi:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(
                    inputTranslationMag
                )

            elif angleDif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.currentTranslationMag > 1e-4:
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)
                else:
                    self.currentTranslationDir = swerveutils.wrapAngle(
                        self.currentTranslationDir + math.pi
                    )
                    self.currentTranslationMag = self.magLimiter.calculate(
                        inputTranslationMag
                    )

            else:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(0.0)

            self.prevTime = currentTime

            xSpeedCommanded = self.currentTranslationMag * math.cos(
                self.currentTranslationDir
            )
            ySpeedCommanded = self.currentTranslationMag * math.sin(
                self.currentTranslationDir
            )
            self.currentRotation = self.rotLimiter.calculate(rot)

        else:
            self.currentRotation = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = self.currentRotation * DriveConstants.kMaxAngularSpeed

        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                self.getGyroHeading(),
            )
            if fieldRelative
            else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(
        self,
        desiredStates: typing.Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.rearLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.rearRight.resetEncoders()

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getGyroHeading(self) -> Rotation2d:
        """Returns the heading of the robot.

        :returns: the robot's heading as Rotation2d
        """
        return Rotation2d.fromDegrees(self.gyro.getAngle() * DriveConstants.kGyroReversed)


    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * DriveConstants.kGyroReversed


    def getTurnRateDegreesPerSec(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.getTurnRate() * 180 / math.pi
