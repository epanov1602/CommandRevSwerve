import math
import typing

import wpilib

from commands2 import Subsystem, TimedCommandRobot
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)
from wpilib import SmartDashboard, Field2d

from commands.aimtodirection import AimToDirectionConstants
from constants import DriveConstants, ModuleConstants
from .maxswervemodule import MAXSwerveModule
from rev import SparkMax, SparkFlex
import navx


GYRO_OVERSHOOT_FRACTION = -3.25 / 360
# ^^ our gyro didn't overshoot, it "undershot" by 0.1 degrees in a 360 degree turn


class DriveSubsystem(Subsystem):
    def __init__(self, maxSpeedScaleFactor=None) -> None:
        super().__init__()
        if maxSpeedScaleFactor is not None:
            assert callable(maxSpeedScaleFactor)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        self.gyroOvershootFraction = 0.0
        if not TimedCommandRobot.isSimulation():
            self.gyroOvershootFraction = GYRO_OVERSHOOT_FRACTION

        enabledChassisAngularOffset = 0 if DriveConstants.kAssumeZeroOffsets else 1

        # Create MAXSwerveModules
        self.frontLeft = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=SparkFlex,
            drivingIsTalon=ModuleConstants.kDrivingMotorIsTalon,
        )

        self.frontRight = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=SparkFlex,
            drivingIsTalon=ModuleConstants.kDrivingMotorIsTalon,
        )

        self.rearLeft = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=SparkFlex,
            drivingIsTalon=ModuleConstants.kDrivingMotorIsTalon,
        )

        self.rearRight = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            motorControllerType=SparkFlex,
            drivingIsTalon=ModuleConstants.kDrivingMotorIsTalon,
        )

        # Override for the direction where robot should point
        self.overrideControlsToFaceThisPoint: Translation2d | None = None

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0
        self._lastGyroAngleAdjustment = 0
        self._lastGyroState = "ok"

        self.xSpeedLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.ySpeedLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d(),
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

        self.simPhysics = None


    def periodic(self) -> None:
        if self.simPhysics is not None:
            self.simPhysics.periodic()

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

    def resetOdometry(self, pose: Pose2d, resetGyro=True) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        if resetGyro:
            self.gyro.reset()
            self.gyro.setAngleAdjustment(0)
            self._lastGyroAngleAdjustment = 0
            self._lastGyroAngleTime = 0
            self._lastGyroAngle = 0

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


    def stop(self):
        self.arcadeDrive(0, 0)


    def arcadeDrive(
        self,
        xSpeed: float,
        rot: float,
        assumeManualInput: bool = False,
    ) -> None:
        self.drive(xSpeed, 0, rot, False, False, square=assumeManualInput)


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
        rotSpeed: float,
        fieldRelative: bool,
        rateLimit: bool,
        square: bool = False
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rotSpeed:      Angular rate of the robot.
        :param fieldRelative: Are provided x and y speeds relative to the field?
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        :param square:        Whether to square the inputs (useful for manual control)
        """

        if square:
            norm = math.hypot(xSpeed, ySpeed)
            rotSpeed = rotSpeed * abs(rotSpeed)
            xSpeed = xSpeed * norm
            ySpeed = ySpeed * norm
        if self.overrideControlsToFaceThisPoint:
            rotSpeed = self.calaculateOverrideRotSpeed()

        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedGoal = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedGoal = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        rotSpeedGoal = rotSpeed * DriveConstants.kMaxAngularSpeed

        # field relative conversion must happen before rate limiting, since rate limiting is optional
        if fieldRelative:
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedGoal, ySpeedGoal, rotSpeedGoal, self.getGyroHeading(),
            )
        else:
            targetChassisSpeeds = ChassisSpeeds(xSpeedGoal, ySpeedGoal, rotSpeedGoal)

        # rate limiting has to be applied this way, to keep the rate limiters current (with time)
        slewedX = self.xSpeedLimiter.calculate(targetChassisSpeeds.vx)
        slewedY = self.ySpeedLimiter.calculate(targetChassisSpeeds.vy)
        slewedRot = self.rotLimiter.calculate(targetChassisSpeeds.omega)
        if rateLimit:
            targetChassisSpeeds.vx, targetChassisSpeeds.vy, targetChassisSpeeds.omega = slewedX, slewedY, slewedRot

        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds)

        maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond
        if self.maxSpeedScaleFactor is not None:
            maxSpeed = maxSpeed * self.maxSpeedScaleFactor()
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed)

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
        maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond
        if self.maxSpeedScaleFactor is not None:
            maxSpeed = maxSpeed * self.maxSpeedScaleFactor()
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, maxSpeed)
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


    def getGyroHeading(self) -> Rotation2d:
        """Returns the heading of the robot, tries to be smart when gyro is disconnected

        :returns: the robot's heading as Rotation2d
        """
        now = wpilib.Timer.getFPGATimestamp()
        past = self._lastGyroAngleTime
        state = "ok"

        if not self.gyro.isConnected():
            state = "disconnected"
        else:
            notCalibrating = True
            if self.gyro.isCalibrating():
                notCalibrating = False
                state = "calibrating"
            gyroAngle = self.gyro.getAngle()

            # correct for gyro drift
            if self.gyroOvershootFraction != 0.0 and self._lastGyroAngle != 0 and notCalibrating:
                angleMove = gyroAngle - self._lastGyroAngle
                if abs(angleMove) > 15:  # if less than 10 degrees, adjust (otherwise it's some kind of glitch or reset)
                    print(f"WARNING: big angle move {angleMove} from {self._lastGyroAngle} to {gyroAngle}")
                else:
                    adjustment = -angleMove * self.gyroOvershootFraction
                    self._lastGyroAngleAdjustment += adjustment
                    self.gyro.setAngleAdjustment(max(-359, min(+359, self._lastGyroAngleAdjustment)))
                    # ^^ NavX code doesn't like angle adjustments outside of (-360, +360) range

            self._lastGyroAngle = gyroAngle
            self._lastGyroAngleTime = now

        if state != self._lastGyroState:
            SmartDashboard.putString("gyro", f"{state} after {int(now - past)}s")
            self._lastGyroState = state

        return Rotation2d.fromDegrees(self._lastGyroAngle * DriveConstants.kGyroReversed)


    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * DriveConstants.kGyroReversed


    def startOverrideToFaceThisPoint(self, point: Translation2d) -> bool:
        if self.overrideControlsToFaceThisPoint is not None:
            return False
        self.overrideControlsToFaceThisPoint = point
        return True


    def stopOverrideToFaceThisPoint(self, point: Translation2d):
        if self.overrideControlsToFaceThisPoint == point:
            self.overrideControlsToFaceThisPoint = None
            return True
        return False


    def calaculateOverrideRotSpeed(self):
        # 1. how many degrees we need to turn?
        pose = self.getPose()
        vectorToTarget = self.overrideControlsToFaceThisPoint - pose.translation()
        if not vectorToTarget.squaredNorm() > 0:
            return 0.0
        targetDirection = vectorToTarget.angle()
        degreesRemainingToTurn = (targetDirection - pose.rotation()).degrees()

        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemainingToTurn > 180:
            degreesRemainingToTurn -= 360
        while degreesRemainingToTurn < -180:
            degreesRemainingToTurn += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemainingToTurn)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        rotSpeed = min(proportionalSpeed, 1.0)

        # 3. if need to turn left, return the positive speed, otherwise negative
        return proportionalSpeed if degreesRemainingToTurn > 0 else -proportionalSpeed


class BadSimPhysics(object):
    """
    this is the wrong way to do it, it does not scale!!!
    the right way is shown here: https://github.com/robotpy/examples/blob/main/Physics/src/physics.py
    and documented here: https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
    (but for a swerve drive it will take some work to add correctly)
    """
    def __init__(self, drivetrain: DriveSubsystem, robot: wpilib.RobotBase):
        self.drivetrain = drivetrain
        self.robot = robot
        self.t = 0

    def periodic(self):
        past = self.t
        self.t = wpilib.Timer.getFPGATimestamp()
        if past == 0:
            return  # it was first time

        dt = self.t - past
        if self.robot.isEnabled():
            drivetrain = self.drivetrain

            states = (
                drivetrain.frontLeft.desiredState,
                drivetrain.frontRight.desiredState,
                drivetrain.rearLeft.desiredState,
                drivetrain.rearRight.desiredState,
            )
            speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(states)

            dx = speeds.vx * dt
            dy = speeds.vy * dt

            heading = drivetrain.getHeading()
            trans = Translation2d(dx, dy).rotateBy(heading)
            rot = (speeds.omega * 180 / math.pi) * dt

            g = drivetrain.gyro
            g.setAngleAdjustment(g.getAngleAdjustment() + rot * DriveConstants.kGyroReversed)
            drivetrain.adjustOdometry(trans, Rotation2d())
