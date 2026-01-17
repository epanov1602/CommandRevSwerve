#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import math
import typing
import commands2

from wpilib import SmartDashboard, Timer
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem


class Constants:
    kUseSqrtControl = AutoConstants.kUseSqrtControl
    kDetectionTimeoutSeconds = 1.0  # if detection lost for this many seconds, done

    # for the swerve command:
    kPTranslate = 0.25 / (DriveConstants.kMaxSpeedMetersPerSecond / 4.7)
    kMinLateralSpeed = 0.025  # driving slower than this is unproductive (motor might not even spin)

    # for the tank command:
    kP = 0.001  # 0.002 is the default, but you must calibrate this to your robot
    kMinTurnSpeed = 0.025  # turning slower than this is unproductive for the motor (might not even spin)


class SwerveTowardsObject(commands2.Command):
    """
    One can use this command to approach gamepieces using camera.
    Example (this can go into robotcontaier.py, inside of configureButtonBindings() function):

        ```
            from commands.drive_towards_object import DriveTowardsObject

            # create a command for driving towards the gamepiece, using existing Limelight camera and pipeline 1 inside it
            driveToGamepiece = SwerveTowardsObject(
                drivetrain=self.robotDrive,
                speed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftTrigger),  # speed controlled by "left trigger" stick of the joystick
                maxLateralSpeed=1.0,
                camera=self.frontPickupCamera,
                cameraPipeline=1,  # if pipeline 1 in that camera is setup to do gamepiece detection
            )

            # setup a condition for when to run that command
            whenLeftTriggerPressed = self.driverController.axisGreaterThan(
                XboxController.Axis.kLeftTrigger, threshold=0.1
            )

            # connect the command to its trigger
            whenLeftTriggerPressed.whileTrue(driveToGamepiece)

        ```
    """

    def __init__(
        self,
        drivetrain: DriveSubsystem,
        speed: typing.Callable[[], float],
        camera,
        cameraPipeline: int = -1,
        objectDiameterMeters: float = 0.2,
        maxLateralSpeed=1.0,
        reverse=False,
    ):
        """

        :param drivetrain: robot drivetrain (tank or swerve)
        :param reverse: is camera located on the reverse side of the robot?
        :param speed: speed of driving forward (not turning), probably comes from the joystick
        :param camera: camera for object detection
        :param cameraPipeline: if not None, which pipeline in the camera is setup to detect this type of object
        :param objectDiameterMeters: needed for distance estimation
        :param maxLateralSpeed: maximum turning speed
        """
        super().__init__()

        assert hasattr(drivetrain, 'getHeading'), "drivetrain must have a getHeading method"
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        assert hasattr(camera, 'hasDetection'), "camera must have a hasDetection method"
        assert hasattr(camera, 'getA'), "camera must have a getA method (for comparing with objectSizeWhenNear)"
        assert hasattr(camera, 'getX'), "camera must have a getX method"
        self.camera = camera
        self.addRequirements(camera)

        assert objectDiameterMeters > 0
        self.objectDiameterMeters = objectDiameterMeters
        self.cameraPipeline = cameraPipeline
        self.cameraStartingHeartbeat = None

        self.reverse = reverse
        self.fwdSpeed = speed
        self.maxLateralSpeed = maxLateralSpeed

        self.startTime = 0.0
        self.lastTimeDetected = None
        self.targetLocationXY: Translation2d | None = None


    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.lastTimeDetected = None
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")

        # if we have a Limelight camera and it is not on the pipeline we wanted, switch it
        self.cameraStartingHeartbeat = None
        if hasattr(self.camera, 'getHB') and self.cameraPipeline != -1:
            if hasattr(self.camera, 'getPipeline') and hasattr(self.camera, 'setPipeline'):
                pipelineWas = self.camera.getPipeline()
                if pipelineWas != self.cameraPipeline:
                    self.camera.setPipeline(self.cameraPipeline)
                    self.cameraStartingHeartbeat = self.camera.getHB() + 2


    def execute(self):
        now = Timer.getFPGATimestamp()
        robotPoseXY: Pose2d = self.drivetrain.getPose()

        # 0. do we know the direction to the object? is it on the right or left of us?
        if self.camera.hasDetection():
            x = self.camera.getX()
            a = self.camera.getA()
            if (a > 0) and (self.cameraStartingHeartbeat is None) or (self.camera.getHB() >= self.cameraStartingHeartbeat):
                distanceToTarget = self.calcualteDistanceFromDetectedObject(a)
                SmartDashboard.putNumber("SwerveTowards/distance", distanceToTarget)
                vectorToTarget = Translation2d(x=distanceToTarget, y=0.0).rotateBy(
                    Rotation2d.fromDegrees(-x) + robotPoseXY.rotation()
                )
                if self.reverse:
                    self.targetLocationXY = robotPoseXY.translation() - vectorToTarget
                else:
                    self.targetLocationXY = robotPoseXY.translation() + vectorToTarget
                self.drivetrain.field.getObject("swerve-towards").setPoses(_square(self.targetLocationXY, side=0.1))
        if self.lastTimeDetected is not None and now > self.lastTimeDetected + Constants.kDetectionTimeoutSeconds:
            self.drivetrain.field.getObject("swerve-towards").setPoses([])
            SmartDashboard.putNumber("SwerveTowards/strafe-distance", 0)
            SmartDashboard.putNumber("SwerveTowards/distance", 0)
            self.targetLocationXY = None

        # 1. if direction to the object unknown, stop and wait
        if self.targetLocationXY is None:
            self.drivetrain.drive(xSpeed=0, ySpeed=0, rotSpeed=0, fieldRelative=False, rateLimit=True)
            return

        # 2. how far to we need to strafe to the left? (let's use the coordinates!)
        vectorToTarget = (self.targetLocationXY - robotPoseXY.translation()).rotateBy(-robotPoseXY.rotation())
        SmartDashboard.putNumber("SwerveTowards/strafe-distance", vectorToTarget.y)

        # 3. otherwise, use proportional control to drive towards it
        lateralSpeed = Constants.kPTranslate * abs(vectorToTarget.y)
        if Constants.kUseSqrtControl:
            lateralSpeed = math.sqrt(0.5 * lateralSpeed)
        if lateralSpeed > abs(self.maxLateralSpeed):
            lateralSpeed = abs(self.maxLateralSpeed)
        fwdSpeed = self.fwdSpeed()
        if lateralSpeed < Constants.kMinLateralSpeed and fwdSpeed == 0:
            lateralSpeed = Constants.kMinLateralSpeed  # avoid motors stuck when turn speed is tiny and no forward motion

        # 4. driving time: if target is on the right, then strafe right (otherwise left)
        if vectorToTarget.y > 0:  # y > 0 means strafe left
            self.drivetrain.drive(fwdSpeed, +lateralSpeed, rotSpeed=0.0, fieldRelative=False, rateLimit=True)
        else:  # y < 0 means strafe right
            self.drivetrain.drive(fwdSpeed, -lateralSpeed, rotSpeed=0.0, fieldRelative=False, rateLimit=True)


    def end(self, interrupted: bool):
        self.drivetrain.stop()
        self.drivetrain.field.getObject("swerve-towards").setPoses([])
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")


    def isFinished(self) -> bool:
        return False  # never finishes on its own


    def calcualteDistanceFromDetectedObject(self, objectSizePercent):
        """
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   1.0steradian * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.0 * 0.01)) = 2.0 meters
        # ... then it must be 2.0 meters away! (assuming that the camera field-of-view is 1.0 steradian, not 1.15)
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        """
        distance = math.sqrt(self.objectDiameterMeters * self.objectDiameterMeters / (1.15 * 0.01 * objectSizePercent))
        # note: Arducam w OV9281 is 1.70 steradians, not 1.15

        return distance


class DriveTowardsObject(commands2.Command):
    """
    One can use this command to approach gamepieces using camera.
    Example (this can go into robotcontaier.py, inside of configureButtonBindings() function):

        ```
            from commands.drive_towards_object import DriveTowardsObject

            # create a command for driving towards the gamepiece, using existing Limelight camera and pipeline 1 inside it
            driveToGamepiece = DriveTowardsObject(
                drivetrain=self.robotDrive,
                speed=lambda: self.driverController.getRawAxis(XboxController.Axis.kRightTrigger),  # speed controlled by "right trigger" stick of the joystick
                maxTurnSpeed=1.0,
                camera=self.frontPickupCamera,
                cameraPipeline=1,  # if pipeline 1 in that camera is setup to do gamepiece detection
            )

            # setup a condition for when to run that command
            whenRightTriggerPressed = self.driverController.axisGreaterThan(
                XboxController.Axis.kRightTrigger, threshold=0.1
            )

            # connect the command to its trigger
            whenRightTriggerPressed.whileTrue(driveToGamepiece)

        ```
    """

    def __init__(
        self,
        drivetrain,
        speed: typing.Callable[[], float],
        camera,
        cameraPipeline: int = -1,
        objectSizeWhenNear: float = 10.0,
        maxTurnSpeed=1.0,
    ):
        """

        :param drivetrain: robot drivetrain (tank or swerve) 
        :param speed: speed of driving (not turning)
        :param camera: camera for object detection
        :param cameraPipeline: if not None, which pipeline in the camera is setup to detect this type of object
        :param objectSizeWhenNear: size of object in the camera when it is too close, units = % of screen (geometry says we should be turning much less when object is too close)
        :param maxTurnSpeed: maximum turning speed
        """
        super().__init__()

        assert hasattr(drivetrain, 'getHeading'), "drivetrain must have a getHeading method"
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        assert hasattr(camera, 'hasDetection'), "camera must have a hasDetection method"
        assert hasattr(camera, 'getA'), "camera must have a getA method (for comparing with objectSizeWhenNear)"
        assert hasattr(camera, 'getX'), "camera must have a getX method"
        self.camera = camera
        self.addRequirements(camera)

        assert objectSizeWhenNear > 0
        self.objectSizeWhenNear = objectSizeWhenNear
        self.cameraPipeline = cameraPipeline
        self.cameraStartingHeartbeat = None

        self.fwdSpeed = speed
        self.maxTurnSpeed = maxTurnSpeed

        self.startTime = 0.0
        self.lastTimeDetected = None
        self.targetDirection = None


    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.lastTimeDetected = None
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")

        # if we have a Limelight camera and it is not on the pipeline we wanted, switch it
        self.cameraStartingHeartbeat = None
        if hasattr(self.camera, 'getHB') and self.cameraPipeline != -1:
            if hasattr(self.camera, 'getPipeline') and hasattr(self.camera, 'setPipeline'):
                pipelineWas = self.camera.getPipeline()
                if pipelineWas != self.cameraPipeline:
                    self.camera.setPipeline(self.cameraPipeline)
                    self.cameraStartingHeartbeat = self.camera.getHB() + 2


    def execute(self):
        now = Timer.getFPGATimestamp()

        # 0. do we know the direction to the target
        if self.camera.hasDetection():
            x = self.camera.getX()

            # when object is too close, we should be turning by a smaller angle
            objectSize = self.camera.getA()
            turnFraction = max(0.33, 1.0 - math.sqrt(objectSize / self.objectSizeWhenNear))  # approximation

            if (self.cameraStartingHeartbeat is None) or (self.camera.getHB() >= self.cameraStartingHeartbeat):
                front = self.drivetrain.getHeading()  # which way the robot front is facing?
                self.targetDirection = front.rotateBy(
                    Rotation2d.fromDegrees(-x * turnFraction))  # is object by X degrees to the right from that?
                self.lastTimeDetected = now
        if self.lastTimeDetected is not None and now > self.lastTimeDetected + Constants.kDetectionTimeoutSeconds:
            self.targetDirection = None

        # 1. if direction unknown, stop and wait
        if self.targetDirection is None:
            self.drivetrain.arcadeDrive(0, 0)
            return

        # 2. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()

        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 3. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportionalSpeed = Constants.kP * abs(degreesRemaining)
        if Constants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        turnSpeed = self.maxTurnSpeed
        if turnSpeed > proportionalSpeed:  # turn no faster than the proportional speed is asking
            turnSpeed = proportionalSpeed
        fwdSpeed = self.fwdSpeed()
        if turnSpeed < Constants.kMinTurnSpeed and fwdSpeed == 0:
            turnSpeed = Constants.kMinTurnSpeed  # avoid motors stuck when turn speed is tiny and no forward motion

        # 4. driving time: if target angle is on the right, then turn right (otherwise turn left)
        if degreesRemaining < 0:
            self.drivetrain.arcadeDrive(fwdSpeed, -turnSpeed)  # negative turn speed = right turn
        else:
            self.drivetrain.arcadeDrive(fwdSpeed, +turnSpeed)  # positive turn speed = left turn


    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")


    def isFinished(self) -> bool:
        return False  # never finish on its own


def _square(center: Translation2d, side: float) -> typing.List[Pose2d]:
    zero = Rotation2d(0)
    return [
        Pose2d(center + Translation2d(-side, -side), zero),
        Pose2d(center + Translation2d(0, -side), zero),
        Pose2d(center + Translation2d(+side, -side), zero),
        Pose2d(center + Translation2d(+side, 0), zero),
        Pose2d(center + Translation2d(+side, +side), zero),
        Pose2d(center + Translation2d(0, +side), zero),
        Pose2d(center + Translation2d(-side, +side), zero),
        Pose2d(center + Translation2d(-side, 0), zero),
        Pose2d(center + Translation2d(-side, -side), zero),
    ]
