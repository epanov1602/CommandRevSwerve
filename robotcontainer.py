from __future__ import annotations
import math

import commands2
import rev
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.followobject import StopWhen
from constants import AutoConstants, DriveConstants, OIConstants
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
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)

        # The robot's subsystems
        from subsystems.limelight_camera import LimelightCamera
        self.camera = LimelightCamera("limelight-aiming")  # name of your camera goes in parentheses

        from subsystems.intake import Intake
        self.intake = Intake()

        self.robotDrive = DriveSubsystem()
        # The robots Elevator
        from subsystems.elevator import Elevator
        from rev import LimitSwitchConfig
        self.elevator = Elevator(leadMotorCANId=DriveConstants.kLeadElevationCanId,
                                 followMotorCANId=DriveConstants.kFollowElevationCanId,
                                 presetSwitchPositions=(10, 20, 70), motorClass=rev.SparkMax,
                                 limitSwitchType=LimitSwitchConfig.Type.kNormallyClosed)

        self.elevator.setDefaultCommand(
           commands2.RunCommand(lambda: self.elevator.drive(self.driverController.getRightY()), self.elevator)
        )

        leftBumper = JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
        leftBumper.onTrue(InstantCommand(self.elevator.switchUp, self.elevator))

        from subsystems.localizer import Localizer
        self.localizer = Localizer(drivetrain=self.robotDrive, fieldLayoutFile="2024-crescendo.json")
        self.localizer.addPhotonCamera("front_camera", directionDegrees=0, positionFromRobotCenter=Translation2d(x=0.3, y=0.0))
        self.localizer.addPhotonCamera("left_camera", directionDegrees=+90, positionFromRobotCenter=Translation2d(x=0.3, y=0.0))


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
        from commands.pickup import PickupGamepiece
        from subsystems.intake import Intake
        pickupCommand = PickupGamepiece(self.intake, self.robotDrive, drivingSpeed=0.3)

        from commands2 import InstantCommand, RunCommand
        # RightStick = JoystickButton(self.driverController, XboxController.Button.kRightStick)
        # RightStick.onTrue(InstantCommand(self.elevator.switchUp, self.elevator))

        # assign this command to run while joystick "right bumper" button is pressed
        rightBumper = JoystickButton(self.driverController, XboxController.Button.kRightBumper)
        # rightBumper.whileTrue(pickupCommand)

        aButton = JoystickButton(self.driverController, XboxController.Button.kA)
        # when "A" button is pressed, start intaking the gamepiece
        aButton.onTrue(InstantCommand(self.intake.intakeGamepiece, self.intake))
        # when "A" button is no longer pressed, stop the intake even if it is not done
        aButton.onFalse(InstantCommand(self.intake.stop, self.intake))
        bButton = JoystickButton(self.driverController, XboxController.Button.kB)
        # when "B" button is pressed, start ejecting the gamepiece
        bButton.onTrue(InstantCommand(self.intake.ejectGamepiece, self.intake))
        # when "B" button is no longer pressed, stop ejecting the gamepiece even if it is still in
        bButton.onFalse(InstantCommand(self.intake.stop, self.intake))

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
        self.chosenAuto.setDefaultOption("Fallow coral", self.fallowcoralcommand)
        self.chosenAuto.addOption("curved blue right", self.getcurvedbluerightcommand)
        self.chosenAuto.addOption("approach tag", self.getAproachTagCommand)
        self.chosenAuto.addOption("left Blue Tag approach", self.leftBlueAprileTag)
        self.chosenAuto.addOption("go to midepoint", self.getToStage)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getToStage(self):
        x = 15
        y = 2.75
        return GoToPoint(x,y, self.robotDrive, speed=0.2)

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
        dropcoral = None

        commands = setStartPose.andThen(fallowtag).andThen(alignAndPush).andThen(swervleft).andThen(aimnorth)
        print("I created a command to approach tag")
        return commands

    def fallowcoralcommand(self):
        setStartPose = ResetXY(x=0, y=0, headingDegrees=0, drivetrain=self.robotDrive)

        from commands.followobject import FollowObject, StopWhen
        FollowCoral = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=12.0), speed=0.2)

        from commands.alignwithtag import AlignWithTag
        alignAndPush = AlignWithTag(self.camera, self.robotDrive, 0, speed=0.2, pushForwardSeconds=1.1)
        from commands.aimtodirection import AimToDirection

        aimnorth = AimToDirection(0, self.robotDrive, 0.2, 0)
        dropCoral = None

        # these two lines go to __init__ function
        from commands.pickup import PickupGamepiece
        pickupCommand = PickupGamepiece(self.intake, self.robotDrive, drivingSpeed=0.3)

        commands = setStartPose.andThen(FollowCoral).andThen(alignAndPush).andThen(aimnorth)
        print("I created a command to approach coral")
        return commands


    def leftBlueAprileTag(self):

        setStartPose = ResetXY(x=0.000, y=0.000, headingDegrees=+0, drivetrain=self.robotDrive)

        from commands.jerky_trajectory import JerkyTrajectory
        trajectory = JerkyTrajectory(
            drivetrain=self.robotDrive,
            endpoint=(3.752, 5.422, -72.300),
            waypoints=[
                (0.000, 0.000, 0.000),
                (3.600, 5,222, -72.300)

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
        dropCoral = None


        commands = setStartPose.andthen(trajectory).andThen(fallowtag).andThen(alignAndPush).andThen(swervleft).andThen(aimnorth)





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
