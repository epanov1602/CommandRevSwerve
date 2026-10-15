from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand, ConditionalCommand, SelectCommand, WaitCommand
from commands2.button import CommandGenericHID
from phoenix6.hardware import TalonFX
from rev import SparkMax, SparkFlex
from wpilib import XboxController, Servo, DriverStation, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d, Rotation3d

from commands.drive_towards_object import SwerveTowardsObject
from commands.intake import PickUp, SuppressIntake, Eject, StowIntake, ShakeIntake, RunIntakeRollersAtIdleSpeed
from commands.shooting import GetReadyAndKeepShooting, GetReadyToShoot, GetInRange, KeepHoodDown, KeepFeederClear, \
    ShootFromFixedPosition, UnjamFeeder
from commands.aimtodirection import AimToDirection
from commands.swervetopoint import SwerveToPoint
from commands.trajectory import SwerveTrajectory, SimpleTrajectory
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems import shooter
from subsystems.firing_table import FiringTable
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.limelight_camera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer
from subsystems.indexer import Indexer

from commands.reset_xy import ResetXY
from subsystems.photon_tag_camera import PhotonTagCamera
from subsystems.shooter import Shooter
from subsystems.intake import IntakeConstants
from subsystems.intake import Intake
from subsystems.hood import Hood
from subsystems.turret import Turret
from subsystems.intake_arm import IntakeArm


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
        self.turret = Turret(
            leadMotorCANId=12,
            canCoderCANId=-1,  # we do not trust cancoder for this, unfortunately (our cancoder forgot its home position)
            drivetrain=self.robotDrive,
            turretLocationOnDrivetrain=Translation2d(x=-4.0*0.0254, y=-4.5*0.0254),
            motorClass=SparkFlex,
            display=True,
        )
        self.firingTable = FiringTable(
            self.robotDrive,
            shooterLocationOnDrivetrain=self.turret.turretLocationOnDrivetrain,
            # where we should shoot when scoring from the alliance area?
            goalIfBlue=Translation2d(x=4.59, y=4.025),
            goalIfRed=Translation2d(x=11.88, y=4.025),
            # where we should shoot, when we are passing from midfield?
            fuelStashesIfBlue=[Translation2d(x=1.6, y=1.6), Translation2d(x=1.6, y=6.4)],
            fuelStashesIfRed=[Translation2d(x=14.8, y=1.6), Translation2d(x=14.8, y=6.4)],
            minimumRangeMeters=0.0,
            maximumRangeMeters=99.0,
        )

        self.indexer = Indexer()
        self.indexer.setDefaultCommand(KeepFeederClear(self.indexer).withTimeout(10))

        #self.hoodServo = Servo(channel=0)
        self.hoodServo = Hood(leadMotorCANId=42, motorClass=TalonFX)
        self.shooter = Shooter(
            inverted= False,
            hoodServos = [self.hoodServo],
            motorClass = TalonFX  # for 714 it's TalonFX, for 8630 it's SparkFlex
        )
        self.shooter.setDefaultCommand(KeepHoodDown(self.shooter))  # shooter keeps hood down when nothing else to do

        self.intake = Intake(
            inverted= False
        )
        self.intake.setDefaultCommand(RunIntakeRollersAtIdleSpeed(self.intake))
        self.intake_arm = IntakeArm(
            leadMotorCANId=2, motorClass=SparkFlex
        )

        self.limelightLocalizer = LimelightLocalizer(self.robotDrive)

        self.limelight_shooter = LimelightCamera("limelight-shooter")
        self.limelightLocalizer.addCamera(
            self.limelight_shooter,
            cameraPoseOnRobot=Translation3d(x=-9.75 * 0.0254, y=9.5 * 0.0254, z=16.5 * 0.0254),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
            cameraPitchAngleDegrees=24
        )

        self.limelight_side = LimelightCamera("limelight-side")
        self.limelightLocalizer.addCamera(
            self.limelight_side,
            cameraPoseOnRobot=Translation3d(x=-6.5 * 0.0254, y=13.5 * 0.0254, z=16.5 * 0.0254),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(90),
            cameraPitchAngleDegrees=15
        )

        self.pickupCamera = LimelightCamera("limelight-intake")



        # The driver's controller (joystick)
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.operatorController = CommandGenericHID(OIConstants.kOperatorControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default command for driving using joystick sticks
        from commands.holonomicdrive import HolonomicDrive

        # if the driver pushes the left bumper, they drive in "FPV mode" (not field-relative)
        fpvButton = self.driverController.button(XboxController.Button.kLeftBumper)

        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                speedFactor=lambda: 0.5 + 0.5 * self.driverController.getRawAxis(XboxController.Axis.kLeftTrigger),
                forwardSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -0.35 * self.driverController.getRawAxis(XboxController.Axis.kRightX),
                fieldRelative=lambda: not fpvButton.getAsBoolean(),
                deadband=OIConstants.kDriveDeadband,
                rateLimit=True,
                square=True,
            )
        )

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)


    def resetXYTwice(self, x, y, headingDegrees, flipIfRed: bool, delay=0.05):
        return ResetXY(x, y, headingDegrees, drivetrain=self.robotDrive, flipIfRed=flipIfRed).andThen(
            WaitCommand(delay)
        ).andThen(
            ResetXY(x, y, headingDegrees, drivetrain=self.robotDrive, flipIfRed=flipIfRed)
        )


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # example 1: hold the wheels in "swerve X brake" position, when "X" button is pressed
        def rehome():
            self.intake_arm.forgetZero()
            self.turret.forgetZero()
            self.hoodServo.forgetZero()

        aButtonD = self.driverController.button(XboxController.Button.kA)
        aButtonD.onTrue(InstantCommand(rehome))
        aButtonO = self.operatorController.button(XboxController.Button.kA)
        aButtonO.onTrue(InstantCommand(rehome))

        yButtonD = self.driverController.button(XboxController.Button.kY)
        yButtonD.onTrue(StowIntake(self.intake_arm))
        yButtonO = self.operatorController.button(XboxController.Button.kY)
        yButtonO.onTrue(StowIntake(self.intake_arm))

        self.operatorController.button(XboxController.Button.kLeftStick).whileTrue(
            ShootFromFixedPosition(self.turret, self.shooter, self.indexer)
        )

        # example 2: when "POV-up" button pressed, reset robot field position to "intake facing the players"
        resetFacingNorthCommand = self.resetXYTwice(x=2.5, y=4.025, headingDegrees=180, flipIfRed=True)
        povUpButton = self.driverController.povUp()
        povUpButton.whileTrue(resetFacingNorthCommand)

        # POV down: unjam the shooter
        povDownButton = self.driverController.povDown()
        povDownButton.whileTrue(UnjamFeeder(self.indexer))
        povDownButtonO = self.operatorController.povDown()
        povDownButtonO.whileTrue(UnjamFeeder(self.indexer))

        from commands.point_towards_location import PointTowardsLocation

        # create a command for keeping the robot nose pointed towards the hub
        keepPointingTowardsHub = PointTowardsLocation(
            drivetrain=self.robotDrive,
            firingTable=self.firingTable,
            location=None,  # Translation2d(x=4.59, y=4.025),
            locationIfRed=None,  # Translation2d(x=11.88, y=4.025),
        )
        whenRightTriggerPressed = self.driverController.axisGreaterThan(
            XboxController.Axis.kRightTrigger, threshold=0.5
        )
        whenRightTriggerPressed.whileTrue(keepPointingTowardsHub)
        # ^^ set up a condition for when to do this: do it when the joystick right trigger is pressed by more than 50%
        whenOperatorRightTriger = self.operatorController.axisGreaterThan(
            XboxController.Axis.kRightTrigger, threshold=0.5
        )
        whenOperatorRightTriger.whileTrue(PickUp(intakeRollers=self.intake, arm=self.intake_arm))

        whenOperatorLeftTriger = self.operatorController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger, threshold=0.4
        )
        whenOperatorLeftTriger.whileTrue(ShakeIntake(intake=self.intake, arm=self.intake_arm))
        # create a command for keeping the robot nose pointed 45 degrees (for traversing the hump on a swerve drive)
        keepNoseAt45Degrees = PointTowardsLocation(
            drivetrain=self.robotDrive,
            firingTable=None,
            locationIfRed=Translation2d(x=999999, y=999999),
            location=Translation2d(x=-999999, y=-999999),
        )
        self.driverController.button(XboxController.Button.kRightBumper).whileTrue(keepNoseAt45Degrees)
        # ^^ set up a condition for when to do this: do it when the joystick right bumper is pressed

        getReadyAndShoot = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        )
        getReadyAndShootLowBattery = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
            rpmFactor=1.05,
        )

        getReadyAndShootAndShake = getReadyAndShoot.deadlineFor(ShakeIntake(self.intake, self.intake_arm))

        #opperator controls
        self.operatorController.button(XboxController.Button.kX).whileTrue(getReadyAndShootAndShake)
        self.operatorController.button(XboxController.Button.kB).whileTrue(getReadyAndShootLowBattery)

        #driver controls
        self.driverController.button(XboxController.Button.kX).whileTrue(getReadyAndShootAndShake)

        # intake commands
        self.driverController.povLeft().whileTrue(PickUp(self.intake, arm=self.intake_arm))
        self.driverController.povRight().whileTrue(ShakeIntake(self.intake, self.intake_arm))

        #self.driverController.button(XboxController.Button.kB).whileTrue(
        #    RunCommand(lambda: self.hoodServo.setPositionGoal(-0.4))
        #)



    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        if command is None:
            return InstantCommand(lambda: SmartDashboard.putString("WhyNotShooting", "chosen auto=None")).andThen(
                WaitCommand(3.0)
            )
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("OverTheHumpRight", self.createOverTheHumpRightAuto)
        self.chosenAuto.addOption("OverTheHumpLeft", self.createOverTheHumpLeftAuto)
        # self.chosenAuto.addOption("4909 Left",self.createAuto4909Left)
        # self.chosenAuto.addOption("4909 Right",self.createAuto4909Right)
        # self.chosenAuto.addOption("1678 right", self.createAuto1678Right)
        # self.chosenAuto.addOption("1678 left", self.createAuto1678Left)
        self.chosenAuto.addOption("Hub to Human", self.createAutoCenterToHuman)
        # self.chosenAuto.addOption("Test2", self.getAutonomousTest2Shooting)
        self.chosenAuto.addOption("Depot",self.getAutonmouseDepotintake)
        self.chosenAuto.addOption("PointNorthLeft",self.createPointNorthLeftAuto)
        self.chosenAuto.addOption("PointNorthRight",self.createPointNorthRightAuto)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def createAuto4909Left(self):
        setStartPose = ConditionalCommand(
            self.resetXYTwice(x=12.96, y=0.652, headingDegrees=+180, flipIfRed=True),
            ResetXY(x=3.580, y=7.49, headingDegrees=+0, drivetrain=self.robotDrive),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed
        )

        speed = 0.9
        driveTrajectory = SimpleTrajectory(
                drivetrain=self.robotDrive,
                speed=speed,
                waypoints=[
                    (2.714, 7.274, 110),
                    (3.872,7.390, 0),
                    (5.625,7.382, -12),
                    (7.763, 6.508,-60),
                    # next waypoint

                ],
                endpoint=(7.557, 4.510, -110),
                flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
                stopAtEnd=True,  # to keep driving onto next command, set =False
                swerve=True,
            )

        driveAndPickUp = driveTrajectory.deadlineFor(
            PickUp(intakeRollers=self.intake, arm=self.intake_arm)
        )
        driveInReverse = driveTrajectory.reversed()

        shootWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,
            indexer=self.indexer,
        ).withTimeout(3.0)

        driveNearHub = SimpleTrajectory(
                drivetrain=self.robotDrive,
                speed=speed,
                waypoints=[
                    (2.709, 7.274, 110),
                    (3.866,7.373, 90),
                    (4.599,7.415, 0),
                    (5.787,7.274,-30),
                    (6.402,6.065,-104)
                    # next waypoint

                ],
                endpoint=(5.787, 4.445, -110),
                flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
                stopAtEnd=True,  # to keep driving onto next command, set =False
                swerve=True,
            )
        driveToHubPickup = driveNearHub.deadlineFor(
            PickUp(intakeRollers=self.intake, arm=self.intake_arm)
        )
        driveToHubReverse = driveNearHub.reversed()
        shootAgain = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,
            indexer=self.indexer,
        ).withTimeout(3.0)

        return setStartPose.andThen(driveAndPickUp).andThen(
            driveInReverse
        ).andThen(
            shootWhenReady
        ).andThen(
            driveToHubPickup
        ).andThen(
            driveToHubReverse
        ).andThen(
            shootAgain
        )

    def createAuto4909Right(self):
        setStartPose = ConditionalCommand(
            ResetXY(x=12.96, y=7.49, headingDegrees=+180, drivetrain=self.robotDrive),
            ResetXY(x=3.580, y=0.580, headingDegrees=+0, drivetrain=self.robotDrive),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed
        )

        speed = 0.9

        driveTrajectory = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=speed,
            waypoints=[
                (2.817, 0.580, -110),
                (3.800,0.580, 0),
                (5.171, 0.580, 0),
                (6.542, 0.850, 12),
                (7.439, 1.454, 45)

                # next waypoint

            ],
            endpoint=(7.665, 3.560, 110),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        )

        driveAndPickUp = driveTrajectory.deadlineFor(
            PickUp(intakeRollers=self.intake, arm=self.intake_arm)
        )
        driveInReverse = driveTrajectory.reversed()

        shootWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,
            indexer=self.indexer,
        ).withTimeout(3.0)

        driveNearHub = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=speed,
            waypoints=[
                (2.709, 0.580, -110),
                (3.573,0.590, -90),
                (4.653, 0.590, -90),
                (6.467,0.839,12),
                (6.737,1.843, 60)


                # next waypoint

            ],
            endpoint=(5.787, 3.440, 100),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        )
        driveToHubPickup = driveNearHub.deadlineFor(
            PickUp(intakeRollers=self.intake, arm=self.intake_arm)
        )
        driveToHubReverse = driveNearHub.reversed()
        shootAgain = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,
            indexer=self.indexer,
        ).withTimeout(3.0)

        return setStartPose.andThen(driveAndPickUp).andThen(
            driveInReverse
        ).andThen(
            shootWhenReady
        ).andThen(
            driveToHubPickup
        ).andThen(
            driveToHubReverse
        ).andThen(
            shootAgain
        )


    def createAuto1678Right(self):
        setStartPose = ResetXY(x=1.626, y=0.587, headingDegrees=+0, drivetrain=self.robotDrive, flipIfRed=True)

        speed = .9

        driveTrajectory = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=speed,
            waypoints=[
                (1.807, 1.868, 220),
                (3.140, 0.690, 0),
                (5.326, 0.690, 0.0),
                (6.718, 0.855, 60.0),
                (7.746, 1.423, 90) # next waypoint
            ],
            endpoint=(7.746, 3.498, 110),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        )

        shootWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).withTimeout(10.0)

        pickUp = WaitCommand(1).andThen(PickUp(intakeRollers=self.intake, arm=self.intake_arm))
        driveAndPickUp = driveTrajectory.deadlineFor(pickUp)   # .alongWith
        driveInReverse = driveTrajectory.reversed()

        return (setStartPose
                .andThen(driveAndPickUp)
                .andThen(driveInReverse).andThen(shootWhenReady))

    def createAuto1678Left(self):
        setStartPose = ResetXY(x=3.490, y=7.49, headingDegrees=+0, drivetrain=self.robotDrive, flipIfRed=True)

        speed = 0.9

        driveTrajectory = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=speed,
            waypoints=[
                (2.532, 5.555, 140),
                (2.907, 7.425, 0),
                (4.641, 7.425, 0),
                (6.426, 7.425, 0),
                (7.992, 6.422,-140), # next waypoint

            ],
            endpoint=(7.759, 4.572, -140),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        )
        scoringWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).withTimeout(10.0)

        pickUp = WaitCommand(1).andThen(PickUp(intakeRollers=self.intake, arm=self.intake_arm))
        driveAndPickUp = driveTrajectory.deadlineFor(pickUp)  # .alongWith
        driveInReverse = driveTrajectory.reversed()

        return (setStartPose
                .andThen(driveAndPickUp)
                .andThen(driveInReverse).andThen(scoringWhenReady))


    def createOverTheHumpRightAuto(self):
        setStartPose = ResetXY(x=3.363, y=2.527, headingDegrees=135, drivetrain=self.robotDrive, flipIfRed=True)

        shootFromThere = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).deadlineFor(ShakeIntake(self.intake, self.intake_arm)).withTimeout(seconds=4.0)

        def inNeutralZone():
            return (self.robotDrive.getPose().x - 8.0) < 4.8

        driveAcrossRamp = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.65,
            waypoints=[
                (3.83, 2.527, 135),
                (5.298, 2.527, 135),
                (5.80, 2.527, 67.5),
            ],
            endpoint=(7.80, 2.527, 0),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
            swerve=True,
        ).deadlineFor(
            PickUp(self.intake, self.intake_arm).onlyWhile(inNeutralZone)
        )

        driveBackAcrossRamp = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.75,
            waypoints=[
                (5.844, 2.527, -45),
                (3.70, 2.527, -45),
            ],
            endpoint=(3.30, 2.527, 225),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        ).deadlineFor(WaitCommand(1.50).andThen(GetReadyToShoot(
            firingTable=self.firingTable, shooter=self.shooter, turret=self.turret, drivetrain=None
        )))

        scoreWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).deadlineFor(ShakeIntake(self.intake, self.intake_arm)).withTimeout(4.0)

        driveWithPinholeTurn = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.70,
            waypoints=[
                (3.83, 2.527, 225),
                (5.298, 2.527, 225),
            ],
            endpoint=(5.70, 2.221, 225),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
            swerve=True,
        )

        driveWithPinholeTurnIntake = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.5,
            waypoints=[
                (6.139, 2.221, -20),
                (7.101, 1.773, 0),
                (7.889, 2.527, 90),
                (7.101, 3.249, 180),
            ],
            endpoint=(6.50, 2.257, 90),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
            swerve=True,
        ).deadlineFor(
            PickUp(intakeRollers=self.intake, arm=self.intake_arm)
        ).deadlineFor(
            WaitCommand(2.0).andThen(GetReadyToShoot(
                firingTable=self.firingTable, shooter=self.shooter, turret=self.turret, drivetrain=None
            ))
        )

        scoreWhenReady2 = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).deadlineFor(ShakeIntake(self.intake, self.intake_arm)).withTimeout(4.0)

        commands = setStartPose.andThen(
            shootFromThere
        ).andThen(
            driveAcrossRamp
        ).andThen(
            driveBackAcrossRamp
        ).andThen(
            scoreWhenReady
        ).andThen(
            driveWithPinholeTurn
        ).andThen(
            driveWithPinholeTurnIntake
        ).andThen(
            scoreWhenReady2
        )

        return commands


    def createOverTheHumpLeftAuto(self):
        setStartPose = ResetXY(x=3.363, y=8.05 - 2.527, headingDegrees=-135, drivetrain=self.robotDrive, flipIfRed=True)

        shootFromThere = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).deadlineFor(ShakeIntake(self.intake, self.intake_arm)).withTimeout(seconds=4.0)

        def inNeutralZone():
            return (self.robotDrive.getPose().x - 8.0) < 4.8

        driveAcrossRamp = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.75,
            waypoints=[
                (3.83, 8.05 - 2.527, -135),
                (5.298, 8.05 - 2.527, -135),
                (5.80, 8.05 - 2.527, -67.5),
            ],
            endpoint=(7.80, 8.05 - 2.527, 0),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
            swerve=True,
        ).deadlineFor(
            PickUp(self.intake, self.intake_arm).onlyWhile(inNeutralZone)
        )

        driveBackAcrossRamp = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.65,
            waypoints=[
                (5.844, 8.05 - 2.527, +45),
                (3.70, 8.05 - 2.527, +45),
            ],
            endpoint=(3.30, 8.05 - 2.527, -225),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        ).deadlineFor(WaitCommand(1.50).andThen(GetReadyToShoot(
            firingTable=self.firingTable, shooter=self.shooter, turret=self.turret, drivetrain=None
        )))

        scoreWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).deadlineFor(ShakeIntake(self.intake, self.intake_arm)).withTimeout(4.0)

        driveWithPinholeTurn = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.70,
            waypoints=[
                (3.83, 8.05 - 2.527, -225),
                (5.298, 8.05 - 2.527, -225),
            ],
            endpoint=(5.70, 8.05 - 2.221, -225),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
            swerve=True,
        )

        driveWithPinholeTurnIntake = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=0.5,
            waypoints=[
                (6.139, 8.05 - 2.221, +20),
                (7.101, 8.05 - 1.773, 0),
                (7.889, 8.05 - 2.527, -90),
                (7.101, 8.05 - 3.249, -180),
            ],
            endpoint=(6.50, 8.05 - 2.257, -90),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
            swerve=True,
        ).deadlineFor(
            PickUp(intakeRollers=self.intake, arm=self.intake_arm)
        ).deadlineFor(
            WaitCommand(2.0).andThen(GetReadyToShoot(
                firingTable=self.firingTable, shooter=self.shooter, turret=self.turret, drivetrain=None
            ))
        )

        scoreWhenReady2 = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).deadlineFor(ShakeIntake(self.intake, self.intake_arm)).withTimeout(4.0)

        commands = setStartPose.andThen(
            shootFromThere
        ).andThen(
            driveAcrossRamp
        ).andThen(
            driveBackAcrossRamp
        ).andThen(
            scoreWhenReady
        ).andThen(
            driveWithPinholeTurn
        ).andThen(
            driveWithPinholeTurnIntake
        ).andThen(
            scoreWhenReady2
        )

        return commands


    def createAutoCenterToHuman(self):
        setStartPose = ResetXY(x=3.527, y=4.025, headingDegrees=270, drivetrain=self.robotDrive, flipIfRed=True)

        speed = 0.33

        shootWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).deadlineFor(ShakeIntake(self.intake, self.intake_arm)).withTimeout(seconds=4.0)

        driveTrajectory = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=speed,
            waypoints=[
                (3.200, 3.70, 270),
            ],
            endpoint=(2.20, 0.652, 0.0),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        )

        shootingTrajectory = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=0.15,
            waypoints=[
                # next waypoint
            ],
            endpoint=(0.2, 0.652, 180),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
            swerve=True,
        ).andThen(WaitCommand(10.0))

        shoot = shootingTrajectory.deadlineFor(
            PickUp(self.intake, self.intake_arm),
            GetReadyAndKeepShooting(
                firingTable=self.firingTable,
                shooter=self.shooter,
                turret=self.turret,
                drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
                indexer=self.indexer,
            )
        )

        commands = setStartPose.andThen(
            shootWhenReady
        ).andThen(
            driveTrajectory
        ).andThen(
            shoot
        )

        return commands


    def getAutonomousTest2Shooting(self):
        setStartPose = ConditionalCommand(
            ResetXY(x=1.542, y=4.025, headingDegrees=+180, drivetrain=self.robotDrive),
            ResetXY(x=AutoConstants.kFieldTags.getFieldLength() - 1.542,
                    y=AutoConstants.kFieldTags.getFieldWidth() - 4.025, headingDegrees=0, drivetrain=self.robotDrive),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        )
        shootWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).withTimeout(seconds=2.0)

        driveToManyGamepieces = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (1.956, 2.217, -65.136),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (3.069, 0.677, -0.567),  # next waypoint: x=2.5, y=5.0
                (5.540, 0.677, 2.570),  # next waypoint
                  # next waypoint
            ],
            endpoint=(7.170, 0.833, +80),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
        )
        centerintake = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+0.5,
            waypoints=[

            ],
            endpoint=(7.972, 2.476, +80),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )

        finalShootingCMD = GetReadyAndKeepShooting(
            firingTable=self.firingTableRighWall,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        ).withTimeout(2.0)
        startIntake = InstantCommand(lambda: self.intake.setVelocityGoal(2000, 1000))
        stopIntake = InstantCommand(lambda: self.intake.setVelocityGoal(0, 0000))
        command = setStartPose.andThen(shootWhenReady.andThen(startIntake).andThen(driveToManyGamepieces)
        .andThen(centerintake).andThen(finalShootingCMD))
        return command

    def createPointNorthLeftAuto(self):
        return self.resetXYTwice(x=3.00, y=8.05 - 0.64, headingDegrees=180, flipIfRed=True)

    def createPointNorthRightAuto(self):
        return self.resetXYTwice(x=3.00, y=0.64, headingDegrees=180, flipIfRed=True)

    def getAutonmouseDepotintake(self):
        setStartPose = self.resetXYTwice(
            x=3.527, y=4.025, headingDegrees=+270, flipIfRed=True
        )

        shootFromThere = SwerveToPoint(
            x=3.250, y=4.025, headingDegrees=+270, drivetrain=self.robotDrive, flipIfRed=True, speed=0.5,
        ).andThen(
            (
                ShootFromFixedPosition(self.turret, self.shooter, self.indexer).deadlineFor(
                    ShakeIntake(self.intake, self.intake_arm)
                )
            ).withTimeout(5.0)
        )

        drivetoball = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (3.00, 4.025, 270),
            ],
            endpoint=(1.758, 5.960, +160),
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
            swerve=True,
        ).andThen(
            # driving with the intake
            SwerveTrajectory(
                drivetrain=self.robotDrive,
                speed=+0.5,
                waypoints=[

                ],
                endpoint=(0.600, 5.960,+180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
                flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
                stopAtEnd=True,  # to keep driving onto next command, set =False
            ).deadlineFor(PickUp(intakeRollers=self.intake, arm=self.intake_arm))
        )

        intakeScoring = SimpleTrajectory(
            drivetrain=self.robotDrive,
            speed=+0.5,
            swerve= True,
            waypoints=[
            ],
            endpoint=(2.461, 5.024, +160),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=True,
            stopAtEnd=True,
        )

        prepareToShoot = GetReadyToShoot(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
        )

        shootafterIntake = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.indexer,
        )

        command = setStartPose.andThen(shootFromThere).andThen(drivetoball).andThen(
            intakeScoring.deadlineFor(prepareToShoot)
        ).andThen(shootafterIntake)
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
        setZero = ResetXY(x=0 , y= 0 ,  headingDegrees =  0 , drivetrain=self.robotDrive)

        turnRight = AimToDirection(degrees=-45, drivetrain=self.robotDrive, speed=0.25)
        turnLeft = AimToDirection(degrees=45, drivetrain=self.robotDrive, speed=0.25)
        backToZero = AimToDirection(degrees=0, drivetrain=self.robotDrive, speed=0.0)
        testGyro = turnRight.andThen(turnLeft).andThen(backToZero)

        squareDance = (
            SwerveToPoint(x= 0.5, y= 0, headingDegrees= 0 , speed= 0.15 , drivetrain=self.robotDrive)
        ).andThen(
            SwerveToPoint( x= 0.5, y= -0.5, headingDegrees= 0 , speed= 0.15 , drivetrain=self.robotDrive)
        ).andThen(
            SwerveToPoint( x= 0, y= -0.5, headingDegrees= 0 , speed= 0.15 , drivetrain=self.robotDrive)
        ).andThen(
            SwerveToPoint( x= 0, y= 0, headingDegrees= 0 , speed= 0.15 , drivetrain=self.robotDrive)
        )

        testArmIntake = PickUp(intakeRollers=self.intake, arm=self.intake_arm).withTimeout(2)

        startWasher = InstantCommand(lambda: self.indexer.setWashingMachineVelocitySetpoint(0.5))
        stopWasher = InstantCommand(lambda: self.indexer.stop())
        testWasher = startWasher.andThen(WaitCommand(2).andThen(stopWasher))

        startFeeder = InstantCommand(lambda: self.indexer.setFeederVelocityGoal(2000))
        stopFeeder = InstantCommand(lambda: self.indexer.stop())
        testFeeder = startFeeder.andThen(WaitCommand(2).andThen(stopFeeder))

        unjamFeeder = UnjamFeeder(self.indexer).withTimeout(3.0)

        liftHood1 = InstantCommand(lambda:self.shooter.setHoodServoGoal(-10.0))
        lowerHood1 = InstantCommand(lambda:self.shooter.setHoodServoGoal(+10.0))
        liftHood2 = InstantCommand(lambda:self.shooter.setHoodServoGoal(-10.0))
        lowerHood2 = InstantCommand(lambda:self.shooter.setHoodServoGoal(+10.0))
        testHood1 = liftHood1.andThen(WaitCommand(1)).andThen(lowerHood1).andThen(WaitCommand(1))
        testHood2 = liftHood2.andThen(WaitCommand(1)).andThen(lowerHood2).andThen(WaitCommand(1))

        turretRight = InstantCommand(lambda:self.turret.setAngleGoal(140))
        turretLeft = InstantCommand(lambda:self.turret.setAngleGoal(220))
        turretZeroPos = InstantCommand(lambda:self.turret.setAngleGoal(180))
        testTurret = turretRight.andThen(
            WaitCommand(2).andThen(turretLeft)
        ).andThen(
            WaitCommand(2).andThen(turretZeroPos)
        )

        startShooter = InstantCommand(lambda: self.shooter.setVelocityGoal(2000, 200))
        stopShooter = InstantCommand(lambda: self.shooter.stop())
        testShooter = startShooter.andThen(WaitCommand(2).andThen(stopShooter))

        command = setZero.andThen(
            testGyro
        ).andThen(
            squareDance
        ).andThen(
            testArmIntake
        ).andThen(
            testWasher
        ).andThen(
            testFeeder
        ).andThen(
            unjamFeeder
        ).andThen(
            testTurret
        ).andThen(
            testHood1.andThen(testHood2)
        ).andThen(
            testShooter
        )

        # disable localizer, then run test dance, then enable it back
        return InstantCommand(lambda: self.limelightLocalizer.setAllowed(False)).andThen(
            command
        ).andThen(
            InstantCommand(lambda: self.limelightLocalizer.setAllowed(True))
        )
