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
from wpimath.geometry import Rotation2d, Translation2d, Pose2d, Transform2d
from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem


class Constants:
    kUseSqrtControl = AutoConstants.kUseSqrtControl
    kDetectionTimeoutSeconds = 1.0  # if detection lost for this many seconds, done

    # for the swerve command:
    kPTranslate = 0.5 / (DriveConstants.kMaxSpeedMetersPerSecond / 4.7)
    kMinLateralSpeed = 0.025  # driving slower than this is unproductive (motor might not even spin)
    kLearnRate = 0.85  # should be 1.0, but really a fudge factor that improves stability of convergence, like "learning rate" elsewhere

    # for the tank command:
    kP = 0.001  # 0.002 is the default, but you must calibrate this to your robot
    kMinTurnSpeed = 0.025  # turning slower than this is unproductive for the motor (might not even spin)


class SwerveTowardsObject(commands2.Command):
    """
    Approaches *one* object (if you want to approach more than one, use ".repeatedly()" as shown below).
    One can use this command to approach gamepieces using camera on the front or back of the robot (back: reverse=True).
    Example (this can go into robotcontaier.py, inside of configureButtonBindings() function):

        ```
            from commands.drive_towards_object import SwerveTowardsObject

            # create a command for driving towards one gamepiece, using existing Limelight camera and pipeline 1 inside it
            driveToGamepiece = SwerveTowardsObject(
                drivetrain=self.robotDrive,
                speed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftTrigger),  # speed controlled by "left trigger" stick of the joystick
                maxLateralSpeed=1.0,
                camera=self.frontPickupCamera,
                cameraLocationOnRobot=Pose2d(x=+0.4, y=-0.2, rotation=Rotation2d.fromDegrees(30)),  # camera located at front-right and tilted 30 degrees to the left
                cameraPipeline=1,  # if pipeline 1 in that camera is setup to do gamepiece detection
                dontSwitchToSmallerObject=True,
            )

            # make a command to repeatedly drive to gamepieces (do it again after one gamepiece reached)
            driveToManyGamepieces = driveToGamepiece.repeatedly()

            # setup a condition for when to run that command
            whenLeftTriggerPressed = self.driverController.axisGreaterThan(
                XboxController.Axis.kLeftTrigger, threshold=0.1
            )

            # connect the command to its trigger
            whenLeftTriggerPressed.whileTrue(driveToManyGamepieces)

        ```
    """

    def __init__(
        self,
        drivetrain: DriveSubsystem,
        speed: typing.Callable[[], float],
        camera,
        cameraLocationOnRobot: Pose2d = Pose2d(x=0, y=0, rotation=Rotation2d.fromDegrees(0)),
        cameraPipeline: int = -1,
        objectDiameterMeters: float = 0.2,
        maxLateralSpeed=1.0,
        dontSwitchToSmallerObject: bool = False,
    ):
        """
        :param drivetrain: robot drivetrain (tank or swerve)
        :param speed: speed of driving forward (not turning), probably comes from the joystick
        :param camera: camera for object detection
        :param cameraLocationOnRobot: where is camera located on robot? ( examples:
          frontRight=Pose2d(x=0.4, y=-0.2, rotation=Rotation2d.fromDegrees(0)),
          rearLeft=Pose2d(x=-0.4, y=0.2, rotation=Rotation2d.fromDegrees(180)
        )
        :param cameraPipeline: if not None, which pipeline in the camera is setup to detect this type of object
        :param objectDiameterMeters: needed for distance estimation
        :param maxLateralSpeed: maximum turning speed
        :param dontSwitchToSmallerObject: if the old (bigger) object disappeared from frame, keep driving until reached
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

        self.cameraOnRobot = cameraLocationOnRobot
        self.dontSwitchToSmallerObject = dontSwitchToSmallerObject

        assert objectDiameterMeters > 0
        self.objectDiameterMeters = objectDiameterMeters
        self.cameraPipeline = cameraPipeline
        self.camStartingHeartbeat = None

        self.fwdSpeed = speed
        self.maxLateralSpeed = maxLateralSpeed

        self.startTime = 0.0
        self.reached = False
        self.lastTimeDetected = None
        self.lastTargetAreaSize: float = 0.0
        self.lastTargetLocationXY: Translation2d | None = None
        self.initialRobotToTarget: Translation2d | None = None
        self.finalRobotToTargetDotProduct = 0.0


    def initialize(self):
        self.lastTargetAreaSize = 0.0
        self.lastTimeDetected = None
        self.lastTargetLocationXY = None
        self.initialRobotToTarget = None

        self.reached = False
        self.startTime = Timer.getFPGATimestamp()
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")

        # if we have a Limelight camera and it is not on the pipeline we wanted, switch it
        self.camStartingHeartbeat = None
        if hasattr(self.camera, 'getHB') and self.cameraPipeline != -1:
            if hasattr(self.camera, 'getPipeline') and hasattr(self.camera, 'setPipeline'):
                pipelineWas = self.camera.getPipeline()
                if pipelineWas != self.cameraPipeline:
                    self.camera.setPipeline(self.cameraPipeline)
                    self.camStartingHeartbeat = self.camera.getHB() + 2


    def execute(self):
        now = Timer.getFPGATimestamp()
        robotXY: Pose2d = self.drivetrain.getPose()

        fwdSpeed = self.fwdSpeed()
        fwdSpeed *= abs(fwdSpeed)
        if fwdSpeed != 0 and abs(fwdSpeed) < Constants.kMinLateralSpeed:
            fwdSpeed = math.copysign(Constants.kMinLateralSpeed, fwdSpeed)

        # 0. do we know the direction to the object? is it on the right or left of us?
        self.updateObjectLocation(now, robotXY)

        # 1. if direction to the object unknown, just drive slowly
        if self.lastTargetLocationXY is None:
            slow = math.copysign(max(abs(fwdSpeed) / 3, Constants.kMinLateralSpeed), fwdSpeed)
            self.drivetrain.drive(slow, 0, 0, fieldRelative=False, rateLimit=True, square=False)
            SmartDashboard.putNumber("SwerveTowardsObject/strafe-distance", float('nan'))
            return

        # 2. how far to we need to strafe to the left? (let's use the coordinates!)
        vectorToTarget = (self.lastTargetLocationXY - robotXY.translation()).rotateBy(-robotXY.rotation())
        SmartDashboard.putNumber("SwerveTowardsObject/strafe-distance", vectorToTarget.y)

        # 3. use proportional control to drive towards it
        lateralSpeed = Constants.kPTranslate * abs(vectorToTarget.y)
        if Constants.kUseSqrtControl:
            lateralSpeed = math.sqrt(0.5 * lateralSpeed)
        if lateralSpeed > abs(self.maxLateralSpeed):
            lateralSpeed = abs(self.maxLateralSpeed)
        if lateralSpeed < Constants.kMinLateralSpeed and fwdSpeed == 0:
            lateralSpeed = Constants.kMinLateralSpeed  # avoid motors stuck when turn speed is tiny and no forward motion

        # 4. driving time: if target is on the right, then strafe right (otherwise left)
        if vectorToTarget.y > 0:  # y > 0 means that the object was on the left
            self.drivetrain.drive(fwdSpeed, +lateralSpeed, 0.0, fieldRelative=False, rateLimit=True, square=False)
        else:  # y < 0 means strafe right
            self.drivetrain.drive(fwdSpeed, -lateralSpeed, 0.0, fieldRelative=False, rateLimit=True, square=False)


    def updateObjectLocation(self, now: float | None, robotXY: Pose2d):
        # 1. do we have a detected object location?
        if self.camera.hasDetection():
            self.lastTimeDetected = now
            x, a = self.camera.getX(), self.camera.getA()
            smallerObject = a < 0.8 * self.lastTargetAreaSize
            if smallerObject and self.dontSwitchToSmallerObject:
                pass  # do not switch to this smaller object that we are seeing now, until the bigger object is reached

            elif (a > 0) and (self.camStartingHeartbeat is None or self.camera.getHB() >= self.camStartingHeartbeat):
                self.lastTargetAreaSize = a
                self.lastTargetLocationXY = self.calculateObjectLocationXY(x, a, robotXY)
                self.drivetrain.field.getObject("swerve-towards").setPoses(_square(self.lastTargetLocationXY, side=0.1))
                if self.initialRobotToTarget is None or smallerObject:
                    self.initialRobotToTarget = self.lastTargetLocationXY - robotXY.translation()
                    self.finalRobotToTargetDotProduct = self.initialRobotToTarget.norm() * DriveConstants.kWheelBase / 2
                    self.finalRobotToTargetDotProduct *= 1.5  # fudge factor
                    SmartDashboard.putString("command/c" + self.__class__.__name__, "acquired")
                    # ^^dotprod(initialRobotToTarget, robotToTarget) will be dropping until ~this value during approach

        # 2. or did we lose or reach the previously detected object?
        reset = False
        if self.lastTimeDetected is not None and now > self.lastTimeDetected + Constants.kDetectionTimeoutSeconds:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "lost")
            reset = True   # no longer seeing the object and timed out
        elif self.lastTargetLocationXY is not None:
            assert self.initialRobotToTarget is not None, "initialRobotToTarget must be set when lastTargetLocationXY is set"
            robotToTarget = self.lastTargetLocationXY - robotXY.translation()
            if robotToTarget.dot(self.initialRobotToTarget) < self.finalRobotToTargetDotProduct:
                SmartDashboard.putString("command/c" + self.__class__.__name__, "reached")
                self.reached = True
                reset = True
                # if reached, reset the lastTargetLocation so the new target can be acquired

        if reset:
            self.drivetrain.field.getObject("swerve-towards").setPoses([])
            SmartDashboard.putNumber("SwerveTowardsObject/distance", 0)
            self.lastTargetAreaSize = 0
            self.lastTimeDetected = None
            self.lastTargetLocationXY = None
            self.initialRobotToTarget = None


    def calculateObjectLocationXY(self, x, a, robotXY: Pose2d):
        distance, direction = self.calcualteDistanceFromDetectedObject(a), Rotation2d.fromDegrees(-x)
        SmartDashboard.putNumber("SwerveTowardsObject/distance", distance)

        distance *= Constants.kLearnRate  # improves convergence, no other reason
        fromCameraToTgt = Translation2d(distance * direction.cos(), distance * direction.sin())
        fromRobotToTgt = fromCameraToTgt.rotateBy(self.cameraOnRobot.rotation()) + self.cameraOnRobot.translation()
        fromRobotToTgtFieldRelative = fromRobotToTgt.rotateBy(robotXY.rotation())

        return robotXY.translation() + fromRobotToTgtFieldRelative


    def end(self, interrupted: bool):
        self.drivetrain.stop()
        self.drivetrain.field.getObject("swerve-towards").setPoses([])
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")


    def isFinished(self) -> bool:
        if self.reached:
            return True
        return False  # otherwise never finishes on its own


    def calcualteDistanceFromDetectedObject(self, objectSizePercent):
        """
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 2.0-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   2.0steradian * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (2.0 * 0.01)) ~= 1.4 meters
        # ... then it must be 1.4 meters away! (assuming that the camera field-of-view is 2.0 steradian)
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        """
        distance = math.sqrt(self.objectDiameterMeters * self.objectDiameterMeters / (1.33 * 0.01 * objectSizePercent))
        # note: Arducam w OV9281 is 1.70 steradians, not 1.33

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
                speed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftTrigger),  # speed controlled by "left trigger" stick of the joystick
                maxTurnSpeed=1.0,
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
