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

        from subsystems.arm import Arm, ArmConstants
        self.arm = Arm(leadMotorCANId=18, followMotorCANId=None)

        # The robot's subsystems
        from subsystems.limelight_camera import LimelightCamera
        self.camera = LimelightCamera("limelight-aiming")  # name of your camera goes in parentheses

        from subsystems.intake import Intake
        self.intake = Intake(leaderCanID=19, followerCanID=None, leaderInverted=True, followerInverted=False)

        # The robots Elevator
        from rev import LimitSwitchConfig
        from subsystems.elevator import Elevator

        self.elevator = Elevator(leadMotorCANId=DriveConstants.kLeadElevationCanId,
                                followMotorCANId=DriveConstants.kFollowElevationCanId,
                                presetSwitchPositions=(2, 15, 28), motorClass=rev.SparkMax,
                                limitSwitchType=LimitSwitchConfig.Type.kNormallyClosed,
                                arm=self.arm)

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

        intakingPosButton = self.scoringController.povLeft()  # position for intaking
        from commands.elevatorcommands import MoveElevatorAndArm
        intakingPosButton.whileTrue(MoveElevatorAndArm(elevator=self.elevator, position=0.0, arm=self.arm, angle=42))

        level0DropButton = self.scoringController.button(XboxController.Button.kA)  # button(XboxController.Button.kRightBumper)
        level0DropButton.whileTrue(MoveElevatorAndArm(elevator=self.elevator, position=0.0, arm=self.arm, angle=70))

        level1DropButton = self.scoringController.button(XboxController.Button.kB)
        level1DropButton.whileTrue(MoveElevatorAndArm(elevator=self.elevator, position= 4.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle))

        level2DropButton = self.scoringController.button(XboxController.Button.kY)
        level2DropButton.whileTrue(MoveElevatorAndArm(elevator=self.elevator, position= 13.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle))

        level3DropButton = self.scoringController.button(XboxController.Button.kX)
        level3DropButton.whileTrue(MoveElevatorAndArm(elevator=self.elevator, position= 30.0, arm=self.arm, angle=135))


        self.robotDrive = DriveSubsystem()

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

        xButton = self.driverController.button(XboxController.Button.kX)
        xButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))
        xButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))  # use the swerve X brake when "X" is pressed

        yButton = self.driverController.button(XboxController.Button.kY)
        yButton.onTrue(ResetSwerveFront(self.robotDrive))

        # this code must be added: see how "A" and "B" button handlers are defined
        from commands.intakecommands import IntakeGamepiece, IntakeFeedGamepieceForward, IntakeEjectGamepieceBackward
        from commands2.instantcommand import InstantCommand

        # while "A" button is pressed, intake the gamepiece until it hits the limit switch (or rangefinder, if connected)
        leftBumper = self.scoringController.button(wpilib.XboxController.Button.kLeftBumper)
        intakeCmd = IntakeGamepiece(self.intake, speed=0.2)
        leftBumper.whileTrue(intakeCmd)

        # while "B" button is pressed, feed that gamepiece forward for a split second
        # (either to ensure it is fully inside, or to eject in that direction if it can eject there)
        leftstick = self.scoringController.button(wpilib.XboxController.Button.kLeftStick)
        intakeFeedFwdCmd = IntakeFeedGamepieceForward(self.intake, speed=0.1).withTimeout(0.3)
        leftstick.whileTrue(intakeFeedFwdCmd)

        # while "Y" button is pressed, eject the gamepiece backward
        rightBumper = self.scoringController.button(wpilib.XboxController.Button.kRightBumper)
        intakeFeedFwdCmd2 = IntakeEjectGamepieceBackward(self.intake, speed=0.3).withTimeout(0.3)
        rightBumper.whileTrue(intakeFeedFwdCmd2)

        # end of the code that must be added

        from commands.trajectory_picker import TrajectoryPicker
        positionPicker = TrajectoryPicker(
            self.robotDrive.field,
            # which subsystems must be locked for exclusive use?
            subsystems=[self.arm, self.elevator],
        )
        leftPOVButton = self.driverController.povDown()
        leftPOVButton.onTrue(InstantCommand(positionPicker.nextTrajectory))

        from commands.elevatorcommands import MoveElevatorAndArm
        MoveElevatorAndArm(self.elevator, 0.0, self.arm, angle=40)


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
        swervleft = SwerveToSide(metersToTheLeft=0.2, speed=0.2, drivetrain=self.robotDrive)
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
