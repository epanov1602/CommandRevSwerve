#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.units import degreesToRadians
from commands2 import TimedCommandRobot, WaitCommand

from commands.aimtodirection import AimToDirection
from commands.jerky_trajectory import JerkyTrajectory, SwerveTrajectory
from commands.swervetopoint import SwerveToSide
from commands.reset_xy import ResetXY


class AutoFactory(object):

    @staticmethod
    def makeAutoCommand(self):
        startX, startY, startHeading = self.startPos.getSelected()
        startPosCmd = ResetXY(startX, startY, startHeading, drivetrain=self.robotDrive)

        goal1traj = self.goal1traj.getSelected()
        goal1branch = self.goal1branch.getSelected()
        goal1level = self.goal1level.getSelected()

        # commands for approaching and retreating from goal 1 scoring location
        headingDegrees, approachCmd, retreatCmd = goal1traj(self, branch=goal1branch)

        # command do we use for aligning the robot to AprilTag after approaching goal 1
        alignWithTagCmd = AutoFactory.alignToTag(self, headingDegrees=headingDegrees, branch=goal1branch)

        # commands for raising the arm and firing that gamepiece for goal 1
        raiseArmCmd = AutoFactory.moveArm(self, level=goal1level)
        shootCmd = AutoFactory.ejectGamepiece(self)
        backupCmd = SwerveToSide(metersToTheLeft=0, metersBackwards=0.4, drivetrain=self.robotDrive)
        dropArmCmd = AutoFactory.moveArm(self, level="intake")

        # not yet done: add goal 2

        # connect them all!
        result = startPosCmd.andThen(
            approachCmd
        ).andThen(
            alignWithTagCmd.alongWith(raiseArmCmd)
        ).andThen(
            shootCmd
        ).andThen(
            backupCmd
        ).andThen(
            retreatCmd.alongWith(dropArmCmd)
        )

        return result

    @staticmethod
    def init(self):
        # 0. starting position for all autos
        self.startPos = SendableChooser()
        self.startPos.setDefaultOption("L", (8.763, 7.256, 180))  # (x, y, headingDegrees)
        self.startPos.addOption("M", (8.763, 6.165, 180))  # (x, y, headingDegrees)
        self.startPos.addOption("R", (8.763, 5.074, 180))  # (x, y, headingDegrees)
        SmartDashboard.putData("autoStartCage", self.startPos)

        # goal 1
        #  - which reef to choose for goal 1
        self.goal1traj = SendableChooser()
        self.goal1traj.setDefaultOption("C", AutoFactory.trajectoriesToSideC)
        self.goal1traj.addOption("D", AutoFactory.trajectoriesToSideD)
        self.goal1traj.addOption("E", AutoFactory.trajectoriesToSideE)
        self.goal1traj.addOption("F", AutoFactory.trajectoryToSideF)
        SmartDashboard.putData("autoTgtReef1", self.goal1traj)
        # - which branch to choose for goal 1
        self.goal1branch = SendableChooser()
        self.goal1branch.setDefaultOption("left", "left")
        self.goal1branch.addOption("right", "right")
        SmartDashboard.putData("autoBranch1", self.goal1branch)
        # - which scoring level to choose for goal 1
        self.goal1level = SendableChooser()
        self.goal1level.setDefaultOption("base", "base")
        self.goal1level.addOption("1", "1")
        self.goal1level.addOption("2", "2")
        self.goal1level.addOption("3", "3")
        SmartDashboard.putData("autoLevel1", self.goal1level)

        # goal 2
        # - which branch to choose for goal 2
        self.goal2branch = SendableChooser()
        self.goal2branch.setDefaultOption("left", "left")
        self.goal2branch.addOption("right", "right")
        SmartDashboard.putData("autoBranch2", self.goal2branch)
        # - which scoring level to choose for goal 2
        self.goal2level = SendableChooser()
        self.goal2level.setDefaultOption("left", "left")
        self.goal2level.addOption("right", "right")
        SmartDashboard.putData("autoLevel2", self.goal2level)


    @staticmethod
    def trajectoriesToSideD(self, branch="right", speed=0.2, swerve="last-point"):
        assert branch in ("right", "left")

        heading = 180
        endpoint = (6.70, 4.20, heading) if branch == "right" else (6.70, 3.80, heading)

        approach = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                (8.751, 5.086, 180),
                (7.313, 4.650, -140),
            ],
            endpoint=endpoint,
        )

        retreat = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                endpoint,
                (6.653, 1.538, +54),
            ],
            endpoint=(1.285, 1.135, +54.0),
        )

        return heading, approach, retreat


    @staticmethod
    def trajectoriesToSideC(self, branch="right", speed=0.2, swerve="last-point"):
        assert branch in ("right", "left")

        heading = 120
        endpoint = (5.838, 2.329, heading) if branch == "right" else (5.335, 2.053, heading)

        approach = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                (8.751, 5.086, 180),
                (7.241, 4.630, -140),
                (6.581, 2.628, -175),
            ],
            endpoint=endpoint,
        )

        retreat = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                endpoint,
                (4.843, 1.478, +54),
            ],
            endpoint=(1.285, 1.135, +54.0),
        )

        return heading, approach, retreat


    @staticmethod
    def trajectoriesToSideE(self, branch="right", speed=0.2, swerve="last-point"):
        assert branch in ("right", "left")

        heading = -120
        endpoint = (5.095, 5.829, heading) if branch == "right" else (5.706, 5.709, heading)

        approach = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                # no waypoints needed for side E
            ],
            endpoint=endpoint,
        )

        retreat = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                endpoint,
                (4.891, 5.632, -54.0),
            ],
            endpoint=(1.285, 6.915, -54.0),
        )

        return heading, approach, retreat


    @staticmethod
    def trajectoryToSideF(self, branch="right", speed=0.2, swerve="last-point"):
        assert False, "not implemented yet"


    @staticmethod
    def alignToTag(self, headingDegrees, branch="right", speed=0.15, pushFwdSpeed=0.07, pushFwdSeconds=1.5):
        assert branch in ("right", "left")

        # which camera do we use? depends whether we aim for "right" or "left" branch
        camera = self.frontLeftCamera if branch == "right" else self.frontLeftCamera

        if TimedCommandRobot.isSimulation():
            return AutoFactory.alignToTagSim(self, headingDegrees, branch, speed, pushFwdSpeed, pushFwdSeconds)
        from commands.setcamerapipeline import SetCameraPipeline
        from commands.followobject import FollowObject, StopWhen
        from commands.alignwithtag import AlignWithTag

        # switch to camera pipeline 3, to start looking for certain kind of AprilTags
        lookForTheseTags = SetCameraPipeline(camera, 1)
        approachTheTag = FollowObject(camera, self.robotDrive, stopWhen=StopWhen(maxSize=10), speed=speed)  # stop when tag size=4 (4% of the frame pixels)
        alignAndPush = AlignWithTag(camera, self.robotDrive, headingDegrees, speed=speed, pushForwardSeconds=pushFwdSeconds, pushForwardSpeed=pushFwdSpeed)

        # connect them together
        alignToScore = lookForTheseTags.andThen(approachTheTag).andThen(alignAndPush)
        return alignToScore


    @staticmethod
    def moveArm(self, level):
        if TimedCommandRobot.isSimulation():
            return WaitCommand(seconds=1)  # play pretend arm move in simulation

        from commands.elevatorcommands import MoveElevatorAndArm
        from subsystems.arm import ArmConstants

        if level == "intake" or level == "base":
            return MoveElevatorAndArm(self.elevator, 0.0, arm=self.arm, angle=42)
        if level == "1":
            return MoveElevatorAndArm(self.elevator, 4.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle)
        if level == "2":
            return MoveElevatorAndArm(self.elevator, 13.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle)
        if level == "3":
            return MoveElevatorAndArm(self.elevator, 30.0, arm=self.arm, angle=135)

        assert False, f"level={level} is not supported"


    @staticmethod
    def ejectGamepiece(self, speed=0.3, timeoutSeconds=0.3):
        from commands.intakecommands import IntakeFeedGamepieceForward
        return IntakeFeedGamepieceForward(self.intake, speed=speed).withTimeout(timeoutSeconds)


    @staticmethod
    def alignToTagSim(self, headingDegrees, branch, speed, pushFwdSpeed, pushFwdSeconds):
        # no camera use in simulation
        fwd = SwerveToSide(metersToTheLeft=0, metersBackwards=-0.4, speed=speed, drivetrain=self.robotDrive)
        align = AimToDirection(headingDegrees, drivetrain=self.robotDrive)
        push = SwerveToSide(metersToTheLeft=0, metersBackwards=-99, drivetrain=self.robotDrive, speed=pushFwdSpeed)
        return fwd.andThen(align).andThen(push.withTimeout(pushFwdSeconds))
