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
    kPTranslate = 0.25 / (DriveConstants.kMaxSpeedMetersPerSecond / 4.7)
    kUseSqrtControl = AutoConstants.kUseSqrtControl
    kMinLateralSpeed = 0.025
    kDetectionTimeoutSeconds = 1.0


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
                self.drivetrain.field.getObject("swerve-towards").setPoses(square(self.targetLocationXY, side=4))
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
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")


    def isFinished(self) -> bool:
        return False  # never finishes on its own


    def calcualteDistanceFromDetectedObject(self, objectSizePercent):
        """
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        """
        return math.sqrt(self.objectDiameterMeters * self.objectDiameterMeters / (0.57 * 0.01 * objectSizePercent))
        # note: Arducam w OV9281 (and Limelight 3 / 4) is 0.57 sq radians (not 1.70)


def square(center: Translation2d, side: float) -> typing.List[Pose2d]:
    zero = Rotation2d(0)
    return [
        Pose2d(center + Translation2d(-side, -side), zero),
        Pose2d(center + Translation2d(+side, -side), zero),
        Pose2d(center + Translation2d(+side, +side), zero),
        Pose2d(center + Translation2d(-side, +side), zero),
        Pose2d(center + Translation2d(-side, -side), zero),
    ]
