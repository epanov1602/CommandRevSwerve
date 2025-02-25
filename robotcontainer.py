from __future__ import annotations
import math

import commands2
import rev
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import constants

from commands.jerky_trajectory import JerkyTrajectory
from constants import DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.arm import Arm, ArmConstants

from commands.gotopoint import GoToPoint
from commands.reset_xy import ResetXY, ResetSwerveFront



class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The driver's controller
        self.driverController = CommandGenericHID(0)
        self.scoringController = CommandGenericHID(1)

        self.arm = Arm(leadMotorCANId=DriveConstants.kArmLeadMotorCanId, followMotorCANId=None)

        # The robot's subsystems
        from subsystems.limelight_camera import LimelightCamera
        self.camera = LimelightCamera("limelight-aiming")  # name of your camera goes in parentheses

        from subsystems.intake import Intake
        from playingwithfusion import TimeOfFlight

        self.rangefinder = TimeOfFlight(DriveConstants.kIntakeRangefinderCanId)
        self.rangefinder.setRangingMode(TimeOfFlight.RangingMode.kShort, 24)

        self.intake = Intake(
            leaderCanID=DriveConstants.kIntakeLeadMotorCanId,
            followerCanID=None, leaderInverted=True, followerInverted=False,
            recoilSpeed=0.15,
            rangeFinder=self.rangefinder,
            rangeToGamepiece=100  # 100 millimeters to gamepiece at most, and if it is further away then it won't count
        )

        # The robots Elevator
        from rev import LimitSwitchConfig
        from subsystems.elevator import Elevator

        self.elevator = Elevator(
            leadMotorCANId=DriveConstants.kLeadElevationCanId,
            followMotorCANId=DriveConstants.kFollowElevationCanId,
            presetSwitchPositions=(2, 15, 28), motorClass=rev.SparkMax,
            limitSwitchType=LimitSwitchConfig.Type.kNormallyClosed,
            arm=self.arm,
            intake=self.intake
        )

        # make sure the arm respects a possibly tighter safe angle range, depending on current elevator pos
        if self.arm is not None:
            def safeArmAngleRange():
                elevatorPosition = self.elevator.getPosition()
                return constants.safeArmAngleRange(elevatorPosition)

            self.arm.setSafeAngleRangeFunction(safeArmAngleRange)

        self.elevator.setDefaultCommand(
            commands2.RunCommand(lambda: self.elevator.drive(
                self.scoringController.getRawAxis(XboxController.Axis.kRightY)
            ), self.elevator)
        )


        def maxSpeedScaledownFactor():
            if not self.elevator.zeroFound:
                return 0.25  # if elevator does not know its zero, max speed = 25%
            elevatorPosition = self.elevator.getPosition()
            if elevatorPosition > 7.0:
                return 0.1  # if elevator position is above 7 inches, max speed = 10% (maybe needs to be much lower?)
            # otherwise, full 100%
            return 1.0

        self.robotDrive = DriveSubsystem(maxSpeedScaleFactor=maxSpeedScaledownFactor)

        from subsystems.localizer import Localizer

        # tell the localizer to only allow flipped field, if it's known that the alliance color is red
        def fieldShouldBeFlipped(allianceColor: wpilib.DriverStation.Alliance):
            return allianceColor == wpilib.DriverStation.Alliance.kRed

        self.localizer = Localizer(
            drivetrain=self.robotDrive,
            fieldLayoutFile="2025-reefscape.json",
            flippedFromAllianceColor=fieldShouldBeFlipped
        )
        self.localizer.addPhotonCamera("front_camera", directionDegrees=0, positionFromRobotCenter=Translation2d(x=0.3, y=0.0))
        self.localizer.addPhotonCamera("left_camera", directionDegrees=+90, positionFromRobotCenter=Translation2d(x=0.3, y=0.0))

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        from commands.holonomicdrive import HolonomicDrive

        # Configure drivetrain commands
        self.robotDrive.setDefaultCommand(
            # "holonomic" means that it rotates left independently of swerving left = three sticks needed to control
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=True,
                square=True,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (CommandGenericHID or XboxController),
        and then passing it to a JoystickButton.
        """

        resetOdometryButton = self.driverController.povUp()
        resetOdometryButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))

        resetSwerveFrontButton = self.driverController.povDown()
        resetSwerveFrontButton.onTrue(ResetSwerveFront(self.robotDrive))

        # if "start" button pressed, reset X,Y position to the **upper** feeding station (x=1.30, y=6.90, 54 degrees **east**)
        startButton = self.scoringController.button(XboxController.Button.kStart)
        startButton.onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(Pose2d(Translation2d(1.30, 6.90), Rotation2d.fromDegrees(-54)))
            )
        )

        # if "end" button pressed, reset X,Y position to the **lower** feeding station (x=1.30, y=1.15, 54 degrees **west**)
        backButton = self.scoringController.button(XboxController.Button.kBack)
        backButton.onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(Pose2d(Translation2d(1.30, 1.15), Rotation2d.fromDegrees(54)))
            )
        )

        # coordinates above assume robot bumper length=0.9 meters (width does not matter), but if you need to recompute then:
        #  - center of feeding station is x=0.84, y=0.65 (lower) and x=0.84, y=7.40 (upper), heading=+-54 degrees


        from commands.holonomicdrive import HolonomicDrive

        # if someone pushes left trigger of scoring controller more than 50%
        leftTriggerAsButton = self.scoringController.axisGreaterThan(XboxController.Axis.kLeftTrigger, threshold=0.50)
        # ... then the sticks of the scoring controller start driving the robot FPV-style (not field-relative)
        leftTriggerAsButton.whileTrue(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -0.3 * self.scoringController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -0.3 * self.scoringController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -0.3 * self.scoringController.getRawAxis(XboxController.Axis.kRightX),
                deadband=0,
                fieldRelative=False,  # driving FPV (first person view), not field-relative (install an FPV camera on robot?)
                rateLimit=False,
                square=True,
            )
        )

        from commands.intakecommands import IntakeGamepiece, IntakeFeedGamepieceForward, IntakeEjectGamepieceBackward
        from commands.elevatorcommands import MoveElevatorAndArm

        # right bumper = intake new gamepiece
        intakingPosButton = self.scoringController.button(XboxController.Button.kRightBumper)
        goToIntakePositionCmd = MoveElevatorAndArm(elevator=self.elevator, position=0.0, arm=self.arm, angle=42)
        intakeCmd = IntakeGamepiece(self.intake, speed=0.115)  # .onlyIf(goToIntakePositionCmd.succeeded)
        intakingPosButton.whileTrue(goToIntakePositionCmd.andThen(intakeCmd))

        # pull the right trigger = eject to score that gamepiece
        ejectButton = self.scoringController.axisGreaterThan(XboxController.Axis.kRightTrigger, 0.5)
        ejectForwardCmd = IntakeFeedGamepieceForward(self.intake, speed=0.3).withTimeout(0.3)
        ejectButton.whileTrue(ejectForwardCmd)

        # elevator buttons for different levels
        #  - 0
        level0PosButton = self.scoringController.button(XboxController.Button.kA)
        level0PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position=0.0, arm=self.arm, angle=70)
        level0PosButton.onTrue(level0PositionCmd)
        #  - 1
        level1PosButton = self.scoringController.button(XboxController.Button.kB)
        level1PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position= 4.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle)
        level1PosButton.onTrue(level1PositionCmd)
        #  - 2
        level2PosButton = self.scoringController.button(XboxController.Button.kY)
        level2PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position= 13.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle)
        level2PosButton.onTrue(level2PositionCmd)
        #  - 3
        level3PosButton = self.scoringController.button(XboxController.Button.kX)
        level3PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position= 30.0, arm=self.arm, angle=135)
        level3PosButton.onTrue(level3PositionCmd)


    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.addOption("Fallow coral", self.fallowcoralcommand)
        self.chosenAuto.addOption("curved blue right", self.getcurvedbluerightcommand)
        self.chosenAuto.addOption("approach tag", self.getAproachTagCommand)
        self.chosenAuto.addOption("go to midepoint", self.getToStage)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getToStage(self):
        x = 15
        y = 2.75
        return GoToPoint(x, y, self.robotDrive, speed=0.2)

    def getAproachTagCommand(self):
        setStartPose = ResetXY(x=0, y=0, headingDegrees=0, drivetrain=self.robotDrive)


        from commands.jerky_trajectory import JerkyTrajectory
        trajectory = JerkyTrajectory(
            drivetrain=self.robotDrive,
            endpoint=(7.000, 6.608, 0.000),
            waypoints= [
                (0.779, 6.608, 7.496),
                (2.469, 5.805, 14.470),
                (4.627, 7.124, -37.776)
            ],
            speed=0.2
        )

        from commands.followobject import FollowObject, StopWhen
        fallowtag = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=12.0), speed=0.2)

        from commands.alignwithtag import AlignWithTag
        alignAndPush = AlignWithTag(self.camera, self.robotDrive, 0, speed=0.2, pushForwardSeconds=1.1)

        from commands.swervetopoint import SwerveToSide
        swervleft = SwerveToSide(metersToTheLeft=0.2, metersBackwards=0.01, speed=0.2, drivetrain=self.robotDrive)

        from commands.aimtodirection import AimToDirection
        aimnorth = AimToDirection(0, self.robotDrive, 0.2, 0)

        commands = setStartPose.andThen(fallowtag).andThen(alignAndPush).andThen(swervleft).andThen(aimnorth)
        return commands

    def fallowcoralcommand(self):
        setStartPose = ResetXY(x=0, y=0, headingDegrees=0, drivetrain=self.robotDrive)

        from commands.followobject import FollowObject, StopWhen
        FollowCoral = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=12.0), speed=0.2)

        from commands.alignwithtag import AlignWithTag
        alignAndPush = AlignWithTag(self.camera, self.robotDrive, 0, speed=0.2, pushForwardSeconds=1.1)
        from commands.aimtodirection import AimToDirection

        aimnorth = AimToDirection(0, self.robotDrive, 0.2, 0)

        commands = setStartPose.andThen(FollowCoral).andThen(alignAndPush).andThen(aimnorth)
        print("I created a command to approach coral")
        return commands


    def getcurvedbluerightcommand(self):
        setStartPose = ResetXY(x=0.911, y=6.632, headingDegrees=+60, drivetrain=self.robotDrive)

        from commands.jerky_trajectory import JerkyTrajectory
        gotoFinish = JerkyTrajectory(drivetrain=self.robotDrive,
                                     endpoint=(7.336, 6.273, 86.009),
                                     waypoints=[
                                         (3.860, 7.100, -25.544), (6.066, 5.865, -26.565)
                                     ], swerve=False,
                                     speed=0.8)
        from commands.aimtodirection import AimToDirection
        aimDown = AimToDirection(degrees=-90.000, drivetrain=self.robotDrive)

        command = setStartPose.andThen(gotoFinish).andThen(aimDown)
        return command


    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None

    def makeAlignWithAprilTagCommand(self, desiredHeading):
        from commands.setcamerapipeline import SetCameraPipeline
        from commands.followobject import FollowObject, StopWhen
        from commands.alignwithtag import AlignWithTag
        from commands.swervetopoint import SwerveToSide

        # switch to camera pipeline 3, to start looking for certain kind of AprilTags
        lookForTheseTags = SetCameraPipeline(self.camera, 1)
        approachTheTag = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=10), speed=0.1)  # stop when tag size=4 (4% of the frame pixels)
        alignAndPush = AlignWithTag(self.camera, self.robotDrive, desiredHeading, speed=0.2, pushForwardSeconds=1.5, pushForwardSpeed=0.07)

        # connect them together
        alignToScore = lookForTheseTags.andThen(approachTheTag).andThen(alignAndPush)

        # or you can do this, if you want to score the coral 15 centimeters to the right and two centimeters back from the AprilTag
        # stepToSide = SwerveToSide(drivetrain=self.robotDrive, metersToTheLeft=-0.15, metersBackwards=0.02, speed=0.2)
        # alignToScore = lookForTheseTags.andThen(approachTheTag).andThen(alignAndPush).andThen(stepToSide)

        return alignToScore
