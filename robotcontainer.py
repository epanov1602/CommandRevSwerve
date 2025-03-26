#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import math

import commands2
import rev
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import XboxController, SendableChooser, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import constants
from commands.approach import ApproachTag, ApproachManually
from commands.elevatorcommands import MoveElevatorAndArm

from commands.jerky_trajectory import JerkyTrajectory, SwerveTrajectory, mirror
from commands.setcamerapipeline import SetCameraPipeline
from commands.swervetopoint import SwerveToSide, SwerveMove
from constants import DriveConstants, OIConstants, RobotCameraLocations
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.arm import Arm, ArmConstants, safeArmAngleRange
from subsystems.elevator import ElevatorConstants

from commands.gotopoint import GoToPoint
from commands.reset_xy import ResetXY, ResetSwerveFront

from autofactory import AutoFactory
from subsystems.elevator import ElevatorConstants


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """
    FIELD_LAYOUT_FILE = "2025-reefscape.json"

    def __init__(self, robot) -> None:
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.robot = robot

        # joysticks and trajectory board
        self.driverController = CommandGenericHID(0)
        self.scoringController = CommandGenericHID(1)
        self.trajectoryBoard = CommandGenericHID(2)
        self.trajectorySide = "left"  # some kind of initial value
        self.trajectoryLetter = "A"  # we need some kind of initial value

        # The robot's subsystems
        self.addArmSubsystems()
        self.addRobotDrivetrain(robot)
        self.addCameras(self.FIELD_LAYOUT_FILE)
        self.addCameraBasedLocalizer(self.FIELD_LAYOUT_FILE)
        # self.addIntakeBasedLocalizer()
        # ^^ sometimes gamepiece is ingested already while driving, so intake-based localizer is not a great idea

        # Configure the tag approach styles
        self.configureReefApproachStyles()

        # Configure the button bindings and autos
        self.configureTrajectoryPicker(speed=1.0, TrajectoryCommand=SwerveTrajectory)  # SwerveTrajectory is gentle on wheel modules
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


    def addCameras(self, fieldLayoutFile):
        if not commands2.TimedCommandRobot.isSimulation():
            # real robot, not simulated
            from subsystems.limelight_camera import LimelightCamera
            from subsystems.photon_tag_camera import PhotonTagCamera
            self.frontRightCamera = LimelightCamera("limelight-aiming")
            self.frontLeftCamera = PhotonTagCamera("Arducam_Front")
            self.rearCamera = PhotonTagCamera("Arducam_Rear")

        else:
            # simulated robot, using simulated cameras
            print(f"Loading AprilTag field layout from {fieldLayoutFile} ...")
            fieldLayout = AprilTagFieldLayout(fieldLayoutFile)
            from subsystems.photon_tag_camera import PhotonTagCameraSim
            self.frontRightCamera = PhotonTagCameraSim(
                "Sim_FrontRight", fieldLayout, RobotCameraLocations.kFrontRight, self.robotDrive
            )  # name of your camera goes in parentheses
            self.frontLeftCamera = PhotonTagCameraSim(
                "Sim_FrontLeft", fieldLayout, RobotCameraLocations.kFrontLeft, self.robotDrive
            )
            self.rearCamera = PhotonTagCameraSim(
                "Sim_Rear", fieldLayout, RobotCameraLocations.kRear, self.robotDrive
            )


    def addRobotDrivetrain(self, robot):
        def maxSpeedScaledownFactor():
            if not self.elevator.zeroFound and not commands2.TimedCommandRobot.isSimulation():
                return 0.25  # if elevator does not know its zero, max speed = 25%
            elevatorPosition = self.elevator.getPosition()
            if elevatorPosition > 10.0:
                return 0.2  # if elevator position is above 7 inches, max speed = 10% (maybe needs to be much lower?)
            # otherwise, full 100%
            return 1.0

        self.robotDrive = DriveSubsystem(maxSpeedScaleFactor=maxSpeedScaledownFactor)
        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)


    def addIntakeBasedLocalizer(self):
        def onIntakeSensingGamepiece(sensing):
            if sensing:
                pose: Pose2d = self.robotDrive.getPose()
                heading = pose.rotation().degrees()
                print(f"intake is sensing a new gamepiece, heading={heading}")

                if -10 > heading > -90 and self.robot.isTeleop():
                    distance = constants.LeftFeeder.pose.translation().distance(pose.translation())
                    if distance < 2.0:
                        print(f"resetting odometry to match the left feeder: heading={heading}, distance={distance}")
                        self.robotDrive.resetOdometry(constants.LeftFeeder.pose, resetGyro=False)
                    else:
                        print(f"not resetting odometry to match the left feeder: distance={distance} meters (high!)")

                if 10 < heading < 90 and self.robot.isTeleop():
                    distance = constants.RightFeeder.pose.translation().distance(pose.translation())
                    if distance < 2.0:
                        print(f"resetting odometry to match the right feeder: heading={heading}, distance={distance}")
                        self.robotDrive.resetOdometry(constants.RightFeeder.pose, resetGyro=False)
                    else:
                        print(f"not resetting odometry to match right left feeder: distance={distance} meters (high!)")

        self.intake.setOnSensingGamepiece(onIntakeSensingGamepiece)


    def addCameraBasedLocalizer(self, fieldLayoutFile):
        from subsystems.localizer import Localizer
        from constants import RobotCameraLocations

        # tell the localizer to only allow flipped field, if it's known that the alliance color is red
        def fieldShouldBeFlipped(allianceColor: wpilib.DriverStation.Alliance):
            return allianceColor == wpilib.DriverStation.Alliance.kRed

        self.localizer = Localizer(
            drivetrain=self.robotDrive,
            fieldLayoutFile=fieldLayoutFile,
            flippedFromAllianceColor=fieldShouldBeFlipped
        )
        self.localizer.addPhotonCamera(
            "Arducam_Front",
            directionDegrees=RobotCameraLocations.kFrontLeft.rotation().degrees(),
            positionFromRobotCenter=RobotCameraLocations.kFrontLeft.translation(),
        )
        self.localizer.addPhotonCamera(
            "ELP_RightSide",
            directionDegrees=RobotCameraLocations.kRight.rotation().degrees(),
            positionFromRobotCenter=RobotCameraLocations.kRight.translation(),
            fov=100  # 100 degrees field-of-view
        )
        self.localizer.addPhotonCamera(
              "Arducam_Rear",
              directionDegrees=RobotCameraLocations.kRear.rotation().degrees(),
              positionFromRobotCenter=RobotCameraLocations.kRear.translation(),
        )


    def addArmSubsystems(self):
        from subsystems.intake import Intake
        from playingwithfusion import TimeOfFlight

        # The robots Intake (uses a rangefinder to detect gamepieces)
        self.rangefinder = TimeOfFlight(DriveConstants.kIntakeRangefinderCanId)
        self.rangefinder.setRangingMode(TimeOfFlight.RangingMode.kShort, 24)
        self.rangefinder.setRangeOfInterest(0, 0, 16, 16)

        self.intake = Intake(
            leaderCanID=DriveConstants.kIntakeLeadMotorCanId,
            leaderInverted=True,
            followerCanID=DriveConstants.kIntakeFollowMotorCanId,
            followerInverted=False,
            recoilSpeed=0.15,
            rangeFinder=self.rangefinder,
            rangeToGamepiece=100  # 100 millimeters to gamepiece at most, and if it is further away then it won't count
        )

        # The robots Arm
        self.arm = Arm(leadMotorCANId=DriveConstants.kArmLeadMotorCanId, followMotorCANId=None)

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
            self.arm.setSafeAngleRangeFunction(lambda: safeArmAngleRange(self.elevator.getPosition()))

        self.elevator.setDefaultCommand(
            commands2.RunCommand(lambda: self.elevator.drive(
                self.scoringController.getRawAxis(XboxController.Axis.kRightY)
            ), self.elevator)
        )


    def disabledInit(self):
        AutoFactory.updateDashboard(self)
        if self.trajectoryPicker is not None:
            self.trajectoryPicker.clearDashboard()


    def autonomousInit(self):
        if self.trajectoryPicker is not None:
            self.trajectoryPicker.clearDashboard()
        AutoFactory.updateDashboard(self)
        self.localizer.setAllowed(False)  # localizer not allowed in auto (untested!)


    def teleopInit(self):
        AutoFactory.clearDashboard(self)
        if self.trajectoryPicker is not None:
            self.trajectoryPicker.updateDashboard()
        self.localizer.setAllowed(True)  # localizer allowed in teleop


    def configureAutos(self) -> None:
        AutoFactory.init(self)

    def configurePovTurns(self):
        from commands.aimtodirection import AimToDirection

        def next60degreesClockwise():
            current = 60 * round(self.robotDrive.getHeading().degrees() / 60.0)
            return current - 60

        def next60degreesCounterclockwise():
            current = 60 * round(self.robotDrive.getHeading().degrees() / 60.0)
            return current + 60

        self.driverController.povUp().whileTrue(
            AimToDirection(degrees=0, drivetrain=self.robotDrive)
        )
        self.driverController.povDown().whileTrue(
            AimToDirection(degrees=180, drivetrain=self.robotDrive)
        )
        self.driverController.povRight().whileTrue(
            AimToDirection(degrees=next60degreesClockwise, drivetrain=self.robotDrive)
        )
        self.driverController.povLeft().whileTrue(
            AimToDirection(degrees=next60degreesCounterclockwise, drivetrain=self.robotDrive)
        )


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (CommandGenericHID or XboxController),
        and then passing it to a JoystickButton.
        """

        resetSwerveFrontButton = self.driverController.button(XboxController.Button.kBack)
        resetSwerveFrontButton.onTrue(ResetSwerveFront(drivetrain=self.robotDrive))

        # if "start" pressed, reset X,Y position to the **lower** feeding station (x=1.30, y=6.90, 54 degrees **west**)
        startButton = self.driverController.button(XboxController.Button.kStart)
        # feeder locations:
        #startButton.onTrue(ResetXY(x=1.285, y=1.135, headingDegrees=+54, drivetrain=self.robotDrive))
        #startButton.onTrue(ResetXY(x=1.285, y=6.915, headingDegrees=-54, drivetrain=self.robotDrive))
        # feeder locations:
        #self.driverController.povRight().onTrue(InstantCommand(lambda: self.robotDrive.resetOdometry(constants.RightFeeder.pose, resetGyro=False)))
        #self.driverController.povLeft().onTrue(InstantCommand(lambda: self.robotDrive.resetOdometry(constants.LeftFeeder.pose, resetGyro=False)))

        # startButton.onTrue(InstantCommand(lambda: self.robotDrive.resetOdometry(constants.LeftFeeder.pose)))
        # startButton.onTrue(InstantCommand(lambda: self.robotDrive.resetOdometry(constants.RightFeeder.pose)))

        # ^^ this (Y,Y) is the right feeding station for today's practice

        # if someone pushes left trigger of scoring controller more than 50%, use sticks to drive FPV
        self.configureFpvDriving(self.driverController, speed=0.3)
        if self.scoringController != self.driverController:
            self.configureFpvDriving(self.scoringController, speed=0.3)

        from commands.intakecommands import IntakeGamepiece, IntakeFeedGamepieceForward, IntakeEjectGamepieceBackward
        from commands.elevatorcommands import MoveElevatorAndArm, MoveArm

        # right bumper = intake new gamepiece
        intakingPosButton = self.scoringController.button(XboxController.Button.kRightBumper)
        goToIntakePositionCmd = MoveElevatorAndArm(elevator=self.elevator, position=0.0, arm=self.arm, angle=ArmConstants.kArmIntakeAngle)
        intakeCmd = AutoFactory.intakeGamepiece(self, speed=0.115)  # .onlyIf(goToIntakePositionCmd.succeeded)
        intakingPosButton.whileTrue(goToIntakePositionCmd.andThen(intakeCmd))

        # right bumper for driver joystick = keep wheels locked in X brake
        if self.driverController != self.scoringController:
            xBrakeButton = self.driverController.button(XboxController.Button.kRightBumper)
            keepWheelsLocked = RunCommand(self.robotDrive.setX, self.robotDrive)
            xBrakeButton.whileTrue(keepWheelsLocked)

        # pull the right trigger = eject to score that gamepiece
        ejectButton = self.scoringController.axisGreaterThan(XboxController.Axis.kRightTrigger, 0.5)
        ejectForwardIfElevatorLow = IntakeFeedGamepieceForward(self.intake, speed=0.3).withTimeout(0.3)
        ejectForwardIfElevatorHigh = MoveArm(self.arm, ArmConstants.kArmLevel4ReleaseAngle).andThen(IntakeFeedGamepieceForward(self.intake, speed=0.3).withTimeout(0.3))
        ejectForwardCmd = cmd.ConditionalCommand(ejectForwardIfElevatorHigh, ejectForwardIfElevatorLow, lambda: self.elevator.getPosition() > 20)
        ejectButton.whileTrue(ejectForwardCmd)

        # driver can eject algae by pressing right trigger
        if self.scoringController != self.driverController:
            ejectButtonDriver = self.driverController.axisGreaterThan(XboxController.Axis.kRightTrigger, 0.5)
            ejectButtonDriver.whileTrue(IntakeFeedGamepieceForward(self.intake, speed=0.3).withTimeout(0.3))

        # pull the left trigger = spin the intake in reverse direction
        ejectBackwardsButton = self.scoringController.axisGreaterThan(XboxController.Axis.kLeftTrigger, 0.5)
        ejectBackwards = IntakeEjectGamepieceBackward(self.intake, speed=0.3).withTimeout(0.3)
        ejectBackwardsButton.whileTrue(ejectBackwards)

        # elevator buttons
        self.configureElevatorButtons()

        # X and B buttons of driver controller allow to approach reef AprilTags for scoring
        # ("POV up" button too, but only if trajectory picker trajectory was set)
        if self.scoringController != self.driverController:

            self.driverController.button(XboxController.Button.kX).whileTrue(
               self.approachReef(
                   self.frontRightCamera,
                   pushForwardSeconds=1.17 * constants.ApproachReefTeleop.timeSeconds,
                   cameraPoseOnRobot=RobotCameraLocations.kFrontLeft
               )
            )
            self.driverController.button(XboxController.Button.kB).whileTrue(
                self.approachReef(
                    self.frontLeftCamera,
                    pushForwardSeconds=constants.ApproachReefTeleop.timeSeconds,
                    cameraPoseOnRobot=RobotCameraLocations.kFrontLeft
                )
            )
            # right and left trigger = approach right or left feeder, manually
            self.driverController.axisGreaterThan(XboxController.Axis.kRightTrigger, 0.1).whileTrue(
                self.approachFeeder(
                    headingDegrees = +54,  # right feeder
                    speed = lambda: 0.7 * self.driverController.getRawAxis(XboxController.Axis.kRightTrigger)
                )
            )
            self.driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger, 0.1).whileTrue(
                self.approachFeeder(
                    headingDegrees = -54,  # left feeder
                    speed = lambda: 0.7 * self.driverController.getRawAxis(XboxController.Axis.kLeftTrigger)
                )
            )

        self.configurePovTurns()

    def configureElevatorButtons(self):
        from commands.intakecommands import IntakeEjectGamepieceBackward

        # elevator buttons for different levels
        #  - 0
        level0PosButton = self.scoringController.button(XboxController.Button.kA)
        level0PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position=ElevatorConstants.heightOfLevel1,
                                               arm=self.arm, angle=ArmConstants.kArmSafeTravelAngle)
        level0PosButton.onTrue(level0PositionCmd)

        # (in game manual there are levels 2, 3 and 4)
        #  - 2
        level2PosButton = self.scoringController.button(XboxController.Button.kB)
        level2PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position=ElevatorConstants.heightOfLevel2,
                                               arm=self.arm, angle=ArmConstants.kArmSafeTravelAngle)
        level2PosButton.onTrue(level2PositionCmd)
        self.trajectoryBoard.button(2).onTrue(level2PositionCmd)
        #  - 3
        level3PosButton = self.scoringController.button(XboxController.Button.kY)
        level3PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position=ElevatorConstants.heightOfLevel3,
                                               arm=self.arm, angle=ArmConstants.kArmSafeTravelAngle)
        level3PosButton.onTrue(level3PositionCmd)
        self.trajectoryBoard.button(3).onTrue(level3PositionCmd)
        #  - 4
        level4PosButton = self.scoringController.button(XboxController.Button.kX)
        level4PositionCmd = MoveElevatorAndArm(elevator=self.elevator, position=ElevatorConstants.heightOfLevel4,
                                               arm=self.arm, angle=ArmConstants.kArmSafeTravelAngle)
        level4PosButton.onTrue(level4PositionCmd)
        self.trajectoryBoard.button(4).onTrue(level4PositionCmd)
        #  - algae 1 (driver controller "A" button)
        levelA1PosButton = self.driverController.button(XboxController.Button.kA)
        levelA1PositionCmd = MoveElevatorAndArm(elevator=self.elevator,
                                                position=ArmConstants.kArmAlgaeElevatorPosition1, arm=self.arm,
                                                angle=ArmConstants.kArmAlgaeIntakeAngle)
        levelA1PosButton.whileTrue(levelA1PositionCmd.andThen(IntakeEjectGamepieceBackward(self.intake, 0.2)))
        #  - algae 2 (driver controller "Y" button)
        levelA2PosButton = self.driverController.button(XboxController.Button.kY)
        levelA2PositionCmd = MoveElevatorAndArm(elevator=self.elevator,
                                                position=ArmConstants.kArmAlgaeElevatorPosition2, arm=self.arm,
                                                angle=ArmConstants.kArmAlgaeIntakeAngle)
        levelA2PosButton.whileTrue(levelA2PositionCmd.andThen(IntakeEjectGamepieceBackward(self.intake, 0.2)))


    def configureFpvDriving(self, joystick, speed):
        """
        FPV = not field-relative
        """
        from commands.holonomicdrive import HolonomicDrive

        # if someone pushes this left bumper
        leftBumper = joystick.button(XboxController.Button.kLeftBumper)

        # ... then the sticks of this joystick start driving the robot FPV-style (not field-relative)
        leftBumper.whileTrue(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -speed * joystick.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -speed * joystick.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -speed * joystick.getRawAxis(XboxController.Axis.kRightX),
                deadband=0,
                fieldRelative=False,  # driving FPV (first person view), not field-relative (install an FPV camera on robot?)
                rateLimit=False,
                square=True,
            )
        )


    def configureTrajectoryPicker(self, swerve=True, speed=0.2, TrajectoryCommand=JerkyTrajectory):
        from commands.trajectory_picker import TrajectoryPicker, ReversedTrajectoryPicker

        # trajectory picker will only run when these subsystems are not busy with other commands
        # (intake not in the list, because sometimes intake needs to finish working while already driving)
        requirements = [self.robotDrive]

        # POV up: run the trajectory while button pushed
        self.trajectoryPicker = TrajectoryPicker(self.robotDrive.field, subsystems=requirements)
        #self.driverController.povUp().whileTrue(self.trajectoryPicker)

        # POV left+right: pick trajectory
        #self.driverController.povLeft().onTrue(InstantCommand(self.trajectoryPicker.previousTrajectory))
        #self.driverController.povRight().onTrue(InstantCommand(self.trajectoryPicker.nextTrajectory))

        self.reversedTrajectoryPicker = ReversedTrajectoryPicker(self.trajectoryPicker, subsystems=[self.robotDrive])
        backUp = SwerveMove(metersToTheLeft=0, metersBackwards=0.15, drivetrain=self.robotDrive, speed=0.5, slowDownAtFinish=False)

        #armDown = MoveElevatorAndArm(self.elevator, position=0.0, arm=self.arm, angle=ArmConstants.kArmIntakeAngle)
        #reverseTrajectory = backUp.andThen(self.reversedTrajectoryPicker.alongWith(armDown))
        reverseTrajectory = backUp.andThen(self.reversedTrajectoryPicker)

        # (when button is pushed, first back up safely and then drive the reverse trajectory)

        #self.driverController.povDown().whileTrue(reverseTrajectory)
        self.trajectoryBoard.button(1).whileTrue(reverseTrajectory)

        # a function to choose trajectory by combining the letter and side (for example, "C-left")
        def chooseTrajectory(letter=None, side=None):
            if letter:
                print(f"choosing trajectory letter {letter}")
                self.trajectoryLetter = letter
            if side:
                print(f"choosing trajectory side {side}")
                self.trajectorySide = side
            self.trajectoryPicker.pickTrajectory(self.trajectoryLetter + "-" + self.trajectorySide)

        # trajectory board using this function (are the button numbers correct?)
        self.trajectoryBoard.button(5).onTrue(InstantCommand(lambda: chooseTrajectory(letter="A")))
        self.trajectoryBoard.button(6).onTrue(InstantCommand(lambda: chooseTrajectory(letter="B")))
        self.trajectoryBoard.button(7).onTrue(InstantCommand(lambda: chooseTrajectory(letter="C")))
        self.trajectoryBoard.button(8).onTrue(InstantCommand(lambda: chooseTrajectory(letter="D")))
        self.trajectoryBoard.button(9).onTrue(InstantCommand(lambda: chooseTrajectory(letter="E")))
        self.trajectoryBoard.button(10).onTrue(InstantCommand(lambda: chooseTrajectory(letter="F")))
        self.trajectoryBoard.button(11).onTrue(InstantCommand(lambda: chooseTrajectory(side="left")))
        self.trajectoryBoard.button(12).onTrue(InstantCommand(lambda: chooseTrajectory(side="right")))

        # while the "left" and "right" buttons of trajectory board are pressed, keep driving the chosen trajectory
        self.trajectoryBoard.button(11).whileTrue(self.trajectoryPicker)
        self.trajectoryBoard.button(12).whileTrue(self.trajectoryPicker)

        # when moving on a reversed trajectory, robot can prepare to back into left feeder or right feeder
        def prepareToBackIntoLeftFeeder():
            self.rearCamera.setOnlyTagIds([1, 13])

        def prepareToBackIntoRightFeeder():
            self.rearCamera.setOnlyTagIds([2, 12])


        # now add the trajectories (please replace these with the real ones):

        #  - go to left branch of reef side B
        goSideELeftBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=(5.394, 5.83, -120),
            waypoints=[
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
                (4.806, 6.243, -90),
            ],
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "E-left",  # needs more work!
            SetCameraPipeline(self.frontRightCamera, 5, onlyTagIds=(11, 20)),
            goSideELeftBranch,
            self.approachReef(self.frontRightCamera, desiredHeading=+240)
        )

        goSideERightBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=(4.96, 5.81, -120),
            waypoints=[
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
                (4.306, 6.243, -75),
            ],
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "E-right",
            SetCameraPipeline(self.frontRightCamera, 0, onlyTagIds=(11, 20)),
            goSideERightBranch,
            self.approachReef(self.frontLeftCamera, desiredHeading=+240)
        )

        #  - go to right branch of reef side B
        goSideCLeftBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=mirror((4.96, 5.81, -120)),
            waypoints=mirror([
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
                (4.306, 6.243, -75),
            ]),
            speed=speed,
            setup=prepareToBackIntoRightFeeder,
        )
        self.trajectoryPicker.addCommands(
            "C-left",
            SetCameraPipeline(self.frontRightCamera, 9, onlyTagIds=(9, 22)),
            goSideCLeftBranch,
            self.approachReef(self.frontRightCamera, desiredHeading=+120)
        )

        goSideCRightBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=mirror((5.394, 5.83, -120)),
            waypoints=mirror([
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
                (4.806, 6.243, -90),
            ]),
            speed=speed,
            setup=prepareToBackIntoRightFeeder,
        )
        self.trajectoryPicker.addCommands(
            "C-right",
            SetCameraPipeline(self.frontRightCamera, 0, onlyTagIds=(9, 22)),
            goSideCRightBranch,
            self.approachReef(self.frontLeftCamera, desiredHeading=+120)
        )

        goSideALeftBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=(2.07, 4.250, 0),
            waypoints=[
                (1.435, 6.765, -54),
                (1.641, 5.622, -54),
            ],
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "A-left",
            SetCameraPipeline(self.frontRightCamera, 7, onlyTagIds=(7, 18)),
            goSideALeftBranch,
           self.approachReef(self.frontRightCamera, desiredHeading=0)
        )

        goSideARightBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=(2.072, 3.621, 0),
            waypoints=[
                (1.435, 6.765, -54),
                (1.641, 5.622, -54),
            ],
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "A-right",
            SetCameraPipeline(self.frontRightCamera, 0, onlyTagIds=(7, 18)),
            goSideARightBranch,
            self.approachReef(self.frontLeftCamera, desiredHeading=0)
        )

        goSideBLeftBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=mirror((3.270, 5.446, -60.0)),
            waypoints=mirror([
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
            ]),
            speed=speed,
            setup=prepareToBackIntoRightFeeder,
        )
        self.trajectoryPicker.addCommands(
            "B-left",
            SetCameraPipeline(self.frontRightCamera, 8, onlyTagIds=(8, 17)),
            goSideBLeftBranch,
            self.approachReef(self.frontRightCamera, desiredHeading=+60)
        )

        goSideBRightBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=mirror((3.450, 5.496, -60.0)),
            waypoints=mirror([
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
            ]),
            speed=speed,
            setup=prepareToBackIntoRightFeeder,
        )
        self.trajectoryPicker.addCommands(
            "B-right",
            SetCameraPipeline(self.frontRightCamera, 0, onlyTagIds=(8, 17)),
            goSideBRightBranch,
            self.approachReef(self.frontLeftCamera, desiredHeading=+60)
        )

        goSideDRightBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=mirror((6.70, 3.85, 180)),
            waypoints=[
                (1.835, 6.265, -54),
            ] + mirror([
                (2.201, 1.986, 54.0),
                (5.355, 1.716, 90),
                (6.742, 2.494, 135),
            ]),
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "D-right",
            SetCameraPipeline(self.frontRightCamera, 4, onlyTagIds=(10, 21)),
            goSideDRightBranch,
            self.approachReef(self.frontLeftCamera, desiredHeading=+180)
        )

        goSideDLeftBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=mirror((6.70, 4.20, 180)),
            waypoints=[
                (1.835, 6.265, -54),
            ] + mirror([
                (2.201, 1.986, 54.0),
                (5.355, 1.716, 90),
                (6.742, 2.494, 135),
            ]),
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "D-left",
            SetCameraPipeline(self.frontRightCamera, 0, onlyTagIds=(10, 21)),
            goSideDLeftBranch,
            self.approachReef(self.frontRightCamera, desiredHeading=+180)
        )

        goSideFLeftBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=(3.450, 5.496, -60.0),
            waypoints=[
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
            ],
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "F-left",  # was too far to the left :(
            SetCameraPipeline(self.frontRightCamera, 6, onlyTagIds=(6, 19)),
            goSideFLeftBranch,
            self.approachReef(self.frontRightCamera, desiredHeading=+300)
        )

        goSideFRightBranch = TrajectoryCommand(
            drivetrain=self.robotDrive,
            swerve=swerve,
            endpoint=(3.270, 5.446, -60.0),
            waypoints=[
                (1.835, 6.265, -54),
                (2.641, 5.922, -40),
            ],
            speed=speed,
            setup=prepareToBackIntoLeftFeeder,
        )
        self.trajectoryPicker.addCommands(
            "F-right",
            SetCameraPipeline(self.frontRightCamera, 0, onlyTagIds=(6, 19)),
            goSideFRightBranch,
            self.approachReef(self.frontLeftCamera, desiredHeading=+300)
        )

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        if self.trajectoryPicker is not None:
            self.trajectoryPicker.clearDashboard()

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        return AutoFactory.makeAutoCommand(self)


    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        from commands.intakecommands import IntakeGamepiece, IntakeFeedGamepieceForward
        from commands.elevatorcommands import MoveElevatorAndArm, MoveArm
        from commands.aimtodirection import AimToDirection

        resetXY = ResetXY(x=0, y=0, headingDegrees=0, drivetrain=self.robotDrive)

        # 1. square dance to test the drivetrain (did it drive crooked or made a good square? rezero the wheels!)
        from commands.swervetopoint import SwerveMove
        forward = SwerveMove(metersToTheLeft=0, metersBackwards=-0.5, speed=0.2, drivetrain=self.robotDrive)
        left = SwerveMove(metersToTheLeft=0.5, metersBackwards=0, speed=0.2, drivetrain=self.robotDrive)
        back = SwerveMove(metersToTheLeft=0, metersBackwards=0.5, speed=0.2, drivetrain=self.robotDrive)
        right = SwerveMove(metersToTheLeft=-0.5, metersBackwards=0, speed=0.2, drivetrain=self.robotDrive)
        squareDance = forward.andThen(left).andThen(back).andThen(right)

        # 2. intake the gamepiece and eject it in position 2, to test arm+elevator+intake
        intake = MoveElevatorAndArm(position=0, angle=ArmConstants.kArmIntakeAngle, elevator=self.elevator, arm=self.arm).andThen(
            IntakeGamepiece(intake=self.intake, speed=0.115).withTimeout(10.0)
        )
        score = MoveElevatorAndArm(position=ElevatorConstants.heightOfLevel3, elevator=self.elevator, arm=self.arm).andThen(
            IntakeFeedGamepieceForward(intake=self.intake, speed=0.3).withTimeout(1.0)
        )
        armDown = MoveElevatorAndArm(position=0, angle=ArmConstants.kArmIntakeAngle, elevator=self.elevator, arm=self.arm)

        # 3. rotate 60 degrees left and back, to test the gyro (did it come back? or continued to spin left?)
        rotation1 = AimToDirection(degrees=60, speed=0.3, drivetrain=self.robotDrive)
        rotation2 = AimToDirection(degrees=0.0, speed=0.3, drivetrain=self.robotDrive)
        rotations = rotation1.andThen(rotation2)

        # 4. the combination
        return resetXY.andThen(squareDance).andThen(intake).andThen(score).andThen(armDown).andThen(rotations)


    def configureReefApproachStyles(self):
        self.reefApproachStyle = SendableChooser()
        self.reefApproachStyle.setDefaultOption("1811 style", 1811)
        self.reefApproachStyle.addOption("5895 style", 5895)
        self.reefApproachStyle.addOption("any tag in sight", 0)
        SmartDashboard.putData("reefAprch", self.reefApproachStyle)


    def approachReef(self, camera, desiredHeading=None, cameraPoseOnRobot=None, pushForwardSeconds=None, finalApproachObjSize=10):
        pushForwardMinDistance = constants.ApproachReefTeleop.minDistance

        def roundToMultipleOf60():
            # angles like 110 will be rounded to nearest multiple of 60, in this case 120
            angle = self.robotDrive.getHeading().degrees()
            rounded = 60 * round(angle / 60)
            print(f"robot angle {angle} rounded to {rounded} degrees")
            return rounded

        # if we already know the robot heading, not need to do any guess work
        if desiredHeading is not None:
            return ApproachTag(
                camera,
                self.robotDrive,
                specificHeadingDegrees=desiredHeading,
                pushForwardSeconds=pushForwardSeconds,
                pushForwardMinDistance=pushForwardMinDistance,
                finalApproachObjSize=finalApproachObjSize
            )

        # otherwise, we need to guess the tag and the heading: there are two ways to do it
        from commands.pick_tag import AimLike5895, AimLike1811

        # option 1
        aimLike5895 = AimLike5895(self.FIELD_LAYOUT_FILE, camera, self.robotDrive, cameraPoseOnRobot)
        approachLike5895 = aimLike5895.andThen(ApproachTag(
            camera,
            self.robotDrive,
            specificHeadingDegrees=aimLike5895.getChosenHeadingDegrees,
            pushForwardSeconds=pushForwardSeconds,
            pushForwardMinDistance=pushForwardMinDistance,
            finalApproachObjSize=finalApproachObjSize
        ))

        # option 2
        aimLike1811 = AimLike1811(self.FIELD_LAYOUT_FILE, camera, self.robotDrive, cameraPoseOnRobot, forceBlue=True)
        approachLike1811 = aimLike1811.andThen(ApproachTag(
            camera,
            self.robotDrive,
            specificHeadingDegrees=aimLike1811.getChosenHeadingDegrees,
            pushForwardSeconds=pushForwardSeconds,
            pushForwardMinDistance=pushForwardMinDistance,
            finalApproachObjSize=finalApproachObjSize
        ))

        # option 3
        pickAnyTag = SetCameraPipeline(camera, 0, ())
        approachAnyTag = pickAnyTag.andThen(ApproachTag(camera,
            self.robotDrive,
            specificHeadingDegrees=roundToMultipleOf60,
            pushForwardSeconds=pushForwardSeconds,
            pushForwardMinDistance=pushForwardMinDistance,
            finalApproachObjSize=finalApproachObjSize
        ))

        command = commands2.SelectCommand(
            {
                0: approachAnyTag,
                1811: approachLike1811,
                5895: approachLike5895,
            },
            self.reefApproachStyle.getSelected)

        return command



    def approachFeeder(self, headingDegrees, speed):
        pipeline = SetCameraPipeline(self.rearCamera, 0, onlyTagIds=(1, 2, 12, 13))

        command = ApproachManually(
            self.rearCamera,
            self.robotDrive,
            speed=speed,
            specificHeadingDegrees=headingDegrees,
            reverse=True,
            settings={"GainTran": constants.ApproachFeederTeleop.speedGain},
        )

        return pipeline.andThen(command)
