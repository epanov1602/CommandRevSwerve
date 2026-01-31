from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, Servo
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d, Rotation3d

from commands.aimtodirection import AimToDirection
from commands.trajectory import SwerveTrajectory, JerkyTrajectory
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.firing_table import FiringTable
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.limelight_camera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer

from commands.reset_xy import ResetXY
from subsystems.photon_tag_camera import PhotonTagCamera
from subsystems.shooter import Shooter


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # tracks the location of goal posts for shooting, recommends firing angles and speeds
        self.firingTable = FiringTable(
            self.robotDrive,
            shooterLocationOnDrivetrain=Translation2d(x=-0.2, y=0),
            goalIfBlue=Translation2d(x=4.59, y=4.025),
            goalIfRed=Translation2d(x=11.88, y=4.025),
        )

        self.limelightLocalizer = LimelightLocalizer(self.robotDrive)

        #self.lumaFrontCamera = PhotonTagCamera("LumaFront")
        #self.frontCamera = LimelightCamera("limelight-front")

        #self.limelightLocalizer.addCamera(
        #    self.lumaFrontCamera,
        #    cameraPoseOnRobot=Translation3d(x=0.40, y=0.1, z=0.5),
        #    cameraHeadingOnRobot=Rotation2d.fromDegrees(0.0),
        #    cameraPitchAngleDegrees=0.0,
        #)


        # The driver's controller (joystick)
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default command for driving using joystick sticks
        from commands.holonomicdrive import HolonomicDrive
        self.robotDrive.setDefaultCommand(
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

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # example 1: hold the wheels in "swerve X brake" position, when "X" button is pressed
        brakeCommand = RunCommand(self.robotDrive.setX, self.robotDrive)
        xButton = self.driverController.button(XboxController.Button.kX)
        xButton.whileTrue(brakeCommand)  # while "X" button is True (pressed), keep executing the brakeCommand

        # example 2: when "POV-up" button pressed, reset robot field position to "facing North"
        resetFacingNorthCommand = ResetXY(x=1.0, y=4.0, headingDegrees=0, drivetrain=self.robotDrive)
        povUpButton = self.driverController.povUp()
        povUpButton.whileTrue(resetFacingNorthCommand)

        # example 3: when "POV-down" is pressed, reset robot field position to "facing South"
        resetFacingSouthCommand = ResetXY(x=7.0, y=4.0, headingDegrees=180, drivetrain=self.robotDrive)
        povDownButton = self.driverController.povDown()
        povDownButton.whileTrue(resetFacingSouthCommand)

        # example 4: robot drives this trajectory command when "A" button is pressed
        trajectoryCommand1 = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (1.0, 7.0, -54),  # start at left feeding station: x=1.0, y=7.0, heading=-54 degrees
                (1.5, 6.5, 0),  # next waypoint
                (2.0, 4.5, 0),  # next waypoint
            ],
            endpoint=(3.2, 4.0, 0),  # end point at the reef facing North
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.whileTrue(trajectoryCommand1)  # while "A" button is pressed, keep running trajectoryCommand1

        # example 5: and when "B" button is pressed, drive the reversed trajectory
        reversedTrajectoryCommand1 = trajectoryCommand1.reversed()
        bButton = self.driverController.button(XboxController.Button.kB)
        bButton.whileTrue(reversedTrajectoryCommand1)  # while "B" button is pressed, keep running this command


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
        self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

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

    def getAutonomousTrajectoryExample(self) -> commands2.Command:
        command = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (1.0, 4.0, 0.0),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (2.5, 5.0, 0.0),  # next waypoint: x=2.5, y=5.0
                (3.0, 6.5, 0.0),  # next waypoint
                (6.5, 5.0, -90),  # next waypoint
            ],
            endpoint=(6.0, 4.0, -180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )

        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode ("test dance") to exercise all subsystems
        """

        # example commands that test drivetrain's motors and gyro (our only subsystem)
        turnRight = AimToDirection(degrees=-45, drivetrain=self.robotDrive, speed=0.25)
        turnLeft = AimToDirection(degrees=45, drivetrain=self.robotDrive, speed=0.25)
        backToZero = AimToDirection(degrees=0, drivetrain=self.robotDrive, speed=0.0)

        command = turnRight.andThen(turnLeft).andThen(backToZero)
        return command
