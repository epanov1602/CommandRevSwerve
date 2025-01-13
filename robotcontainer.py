from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem

from commands.reset_xy import ResetXY, ResetSwerveFront

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        from subsystems.limelight_camera import LimelightCamera
        self.camera = LimelightCamera("limelight-pickup")  # name of your camera goes in parentheses

        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    True,
                    square=True,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        xButton = JoystickButton(self.driverController, XboxController.Button.kX)
        xButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))
        xButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))  # use the swerve X brake when "X" is pressed

        yButton = JoystickButton(self.driverController, XboxController.Button.kY)
        yButton.onTrue(ResetSwerveFront(self.robotDrive))

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
        # self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        # self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        # self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        self.chosenAuto.setDefaultOption("straight blue right to finish", self.getStraightBlueRightCommand)
        self.chosenAuto.addOption("curved blue right to finish", self.getCurvedBlueRightCommand)
        self.chosenAuto.addOption("approach tag", self.getApproachTagCommand)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getApproachCommand(self):
        setStartPose = ResetXY(x=8.775, y=7.262, headingDegrees=180, drivetrain=self.robotDrive)

        from commands.jerky_trajectory import JerkyTrajectory
        trajectory = JerkyTrajectory
            drivetrain=self.robotDrive
            endpoint=(2.594, 3.996, 0.0),
            waypoint=[
                (8.775, 7.262, 180.0),
                (6.503, 7.262, -178.831),
                (3.578, 7.104, -175.504),
                (2.174, 5.497, -101.094)
            ],
            speed=0.2
        )
        from commands.followobject import FollowObject

        followTag = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=8.0), speed=0.2)

        from commands.arcadedrive import ArcadeDrive
        driveForwardALittle = ArcadeDrive(driveSpeed=0.15, rotationSpeed=0.0, drivetrain=self.robotDrive).withTimeout(0.3)

        dropCoral = None

        command = setStartPose.andThen(trajectory).andThen(followTag).andThen(driveForwardALittle).andThen(dropCoral)
        return command
    def getStraightBlueRightCommand(self):
            setStartPose = ResetXY(x=0.773, y=6.696, headingDegrees=+60.000, drivetrain=self.robotDrive)

            from commands.gotopoint import  GoToPoint
            goToFinish = GoToPoint(x=6.520,y=7.369,drivetrain=self.robotDrive)

            from commands.aimtodirection import AimToDirection
            aimdown = AimToDirection(degrees=90,drivetrain=self.robotDrive)

            command = setStartPose.andThen(goToFinish).andThen(aimdown)
            return command

    def getCurvedBlueRightCommand(self):
            setStartPose = ResetXY(x=0.773, y=6.696, headingDegrees=+60.000, drivetrain=self.robotDrive)

            from commands.jerky_trajectory import  JerkyTrajectory
            goToFinish = JerkyTrajectory(drivetrain=self.robotDrive,
                                         endpoint=(6.520, 7.369,-0.748),
                                         waypoints=[
                                             (0.733, 6.696, 60.000),
                                             (1.673, 7.191, -48.013),
                                             (3.236, 6.508, 0.979),
                                             (4.492, 7.062, 51.911),
                                            # You don't need to add the end point, it's already added above
                                         ],
                                         swerve=True,
                                         speed=0.2)





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
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(0.5, 0.5), Translation2d(1, -0.5)],
            # End 1.5 meters straight ahead of where we started, facing forward
            Pose2d(1.5, 0, Rotation2d(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        driveController = HolonomicDriveController(
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPXController, 0, 0),
            thetaController,
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            driveController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
