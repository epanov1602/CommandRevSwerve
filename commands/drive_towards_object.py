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
from wpimath.geometry import Rotation2d
from constants import AutoConstants


class Constants:
    kP = 0.001  # 0.002 is the default, but you must calibrate this to your robot
    kMinTurnSpeed = 0.025  # turning slower than this is unproductive for the motor (might not even spin)
    kUseSqrtControl = AutoConstants.kUseSqrtControl
    kDetectionTimeoutSeconds = 1.0  # if detection lost for this many seconds, done


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
