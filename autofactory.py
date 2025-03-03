#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Translation2d, Rotation2d, Pose2d, Transform2d
from wpimath.units import degreesToRadians
from commands2 import TimedCommandRobot, WaitCommand, InstantCommand, Command

from commands.aimtodirection import AimToDirection
from commands.jerky_trajectory import JerkyTrajectory, SwerveTrajectory
from commands.swervetopoint import SwerveMove
from commands.reset_xy import ResetXY


class AutoFactory(object):

    @staticmethod
    def makeAutoCommand(self):
        startPos = self.startPos.getSelected()
        startX, startY, startHeading = startPos
        startPosCmd = ResetXY(startX, startY, startHeading, drivetrain=self.robotDrive)

        goal1traj = self.goal1traj.getSelected()
        goal1branch = self.goal1branch.getSelected()
        goal1level = self.goal1level.getSelected()

        # commands for approaching and retreating from goal 1 scoring location
        headingDegrees, approachCmd, retreatCmd = goal1traj(self, startPos, branch=goal1branch)

        # command do we use for aligning the robot to AprilTag after approaching goal 1
        alignWithTagCmd = AutoFactory.alignToTag(self, headingDegrees=headingDegrees, branch=goal1branch)

        # commands for raising the arm and firing that gamepiece for goal 1
        raiseArmCmd = AutoFactory.moveArm(self, level=goal1level)
        shootCmd = AutoFactory.ejectGamepiece(self)
        backupCmd = SwerveMove(metersToTheLeft=0, metersBackwards=0.4, drivetrain=self.robotDrive)
        dropArmCmd = AutoFactory.moveArm(self, level="intake")

        # not yet done: add goal 2

        # connect them all (and report status in "autoStatus" widget at dashboard)
        result = startPosCmd.andThen(
            runCmd("approach...", approachCmd)
        ).andThen(
            runCmd("align+raise...", alignWithTagCmd.alongWith(raiseArmCmd))
        ).andThen(
            runCmd("shoot...", shootCmd)
        ).andThen(
            runCmd("back up...", backupCmd)
        ).andThen(
            runCmd("retreat...", retreatCmd.alongWith(dropArmCmd))
        ).andThen(
            autoStatus("done")
        )

        return result

    @staticmethod
    def init(self):
        SmartDashboard.putString("autoStatus", "initialized")

        # 0. starting position for all autos
        self.startPos = SendableChooser()
        self.startPos.addOption("1: L+", (7.189, 7.75, 180))  # (x, y, headingDegrees)
        self.startPos.addOption("2: L", (7.189, 6.177, 180))  # (x, y, headingDegrees)
        self.startPos.setDefaultOption("3: ML", (7.189, 4.40, 180))  # (x, y, headingDegrees)
        self.startPos.addOption("4: MID", (7.189, 4.025, 180))  # (x, y, headingDegrees)
        self.startPos.addOption("5: MR", (7.189, 3.65, 180))  # (x, y, headingDegrees)
        self.startPos.addOption("6: R", (7.189, 1.897, 180))  # (x, y, headingDegrees)
        self.startPos.addOption("7: R+", (7.189, 0.4, 180))  # (x, y, headingDegrees)

        # goal 1
        #  - which reef to choose for goal 1
        self.goal1traj = SendableChooser()
        self.goal1traj.setDefaultOption("C", AutoFactory.trajectoriesToSideC)
        self.goal1traj.addOption("D", AutoFactory.trajectoriesToSideD)
        self.goal1traj.addOption("E", AutoFactory.trajectoriesToSideE)
        self.goal1traj.addOption("F", AutoFactory.trajectoriesToSideF)

        # - which branch to choose for goal 1
        self.goal1branch = SendableChooser()
        self.goal1branch.setDefaultOption("left", "left")
        self.goal1branch.addOption("right", "right")

        # - which scoring level to choose for goal 1
        self.goal1level = SendableChooser()
        self.goal1level.setDefaultOption("base", "base")
        self.goal1level.addOption("2", "2")
        self.goal1level.addOption("3", "3")
        self.goal1level.addOption("4", "4")

        SmartDashboard.putData("autoLevel1", self.goal1level)
        SmartDashboard.putData("autoStartPos", self.startPos)
        SmartDashboard.putData("autoTgtReef1", self.goal1traj)
        SmartDashboard.putData("autoBranch1", self.goal1branch)

        self.startPos.onChange(lambda _: AutoFactory.updateDashboard(self))
        self.goal1traj.onChange(lambda _: AutoFactory.updateDashboard(self))
        self.goal1branch.onChange(lambda _: AutoFactory.updateDashboard(self))

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
    def trajectoriesToSideD(self, start, branch="right", speed=0.2, swerve="last-point"):
        assert branch in ("right", "left")

        heading = 180
        endpoint = (6.59, 4.20, heading) if branch == "right" else (6.59, 3.80, heading)

        approach = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                start,
            ],
            endpoint=endpoint,
        )

        retreat = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=True,
            speed=-speed,
            waypoints=[
                endpoint,
                (6.253, 2.138, 150),
            ],
            endpoint=(1.285, 1.135, +54.0),
        )

        return heading, approach, retreat


    @staticmethod
    def trajectoriesToSideC(self, start, branch="right", speed=0.2, swerve="last-point"):
        assert branch in ("right", "left")

        heading = 120
        endpoint = (5.838, 2.329, heading) if branch == "right" else (5.335, 2.053, heading)

        approach = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                start,
                (6.152, 2.411, 120),
            ],
            endpoint=endpoint,
        )

        retreat = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=True,
            speed=-speed,
            waypoints=[
                endpoint,
                (4.843, 1.478, +54),
            ],
            endpoint=(1.285, 1.135, +54.0),
        )

        return heading, approach, retreat


    @staticmethod
    def trajectoriesToSideE(self, start, branch="right", speed=0.2, swerve="last-point"):
        assert branch in ("right", "left")

        heading = -120
        endpoint = (5.095, 5.829, heading) if branch == "right" else (5.706, 5.709, heading)

        approach = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=swerve,
            speed=speed,
            waypoints=[
                start,
                # no waypoints are needed for side E
            ],
            endpoint=endpoint,
        )

        retreat = JerkyTrajectory(
            drivetrain=self.robotDrive,
            swerve=True,
            speed=-speed,
            waypoints=[
                endpoint,
                (4.691, 6.332, -54.0),
            ],
            endpoint=(1.285, 6.915, -54.0),
        )

        return heading, approach, retreat


    @staticmethod
    def trajectoriesToSideF(self, start, branch="right", speed=0.2, swerve="last-point"):
        assert False, "not implemented"


    @staticmethod
    def alignToTag(self, headingDegrees, branch="right", pipeline=1, tags=(), speed=0.15, pushFwdSpeed=0.07, pushFwdSeconds=1.5):
        assert branch in ("right", "left")

        # which camera do we use? depends whether we aim for "right" or "left" branch
        camera = self.frontLeftCamera if branch == "right" else self.frontLeftCamera

        if TimedCommandRobot.isSimulation():
            return AutoFactory.alignToTagSim(self, headingDegrees, branch, speed, pushFwdSpeed, pushFwdSeconds)
        from commands.setcamerapipeline import SetCameraPipeline
        from commands.followobject import FollowObject, StopWhen
        from commands.alignwithtag import AlignWithTag

        # switch to camera pipeline 3, to start looking for certain kind of AprilTags
        lookForWhichTags = SetCameraPipeline(camera, pipelineIndex=pipeline, tags=tags)

        # if tag is not seen, wiggle right and left until it is maybe seen
        wiggle = AimToDirection(headingDegrees + 30, self.robotDrive).andThen(
            WaitCommand(seconds=0.1)
        ).andThen(
            AimToDirection(headingDegrees - 30, self.robotDrive)
        )
        findTheTag = wiggle.until(camera.hasDetection)

        approachTheTag = FollowObject(camera, self.robotDrive, stopWhen=StopWhen(maxSize=10), speed=speed)  # stop when tag size=4 (4% of the frame pixels)
        alignAndPush = AlignWithTag(camera, self.robotDrive, headingDegrees, speed=speed, pushForwardSeconds=pushFwdSeconds, pushForwardSpeed=pushFwdSpeed)

        # connect them together
        alignToScore = (
            runCmd("align: setpipe...", lookForWhichTags)
        ).andThen(
            runCmd("align: find...", findTheTag)
        ).andThen(
            runCmd("align: approach...", approachTheTag)
        ).andThen(
            runCmd("align: algn+push...", alignAndPush)
        )
        return alignToScore


    @staticmethod
    def moveArm(self, level):
        if TimedCommandRobot.isSimulation():
            return WaitCommand(seconds=1)  # play pretend arm move in simulation

        from commands.elevatorcommands import MoveElevatorAndArm
        from subsystems.arm import ArmConstants

        if level == "intake" or level == "base":
            return MoveElevatorAndArm(self.elevator, 0.0, arm=self.arm, angle=42)
        if level == "2":
            return MoveElevatorAndArm(self.elevator, 4.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle)
        if level == "3":
            return MoveElevatorAndArm(self.elevator, 13.0, arm=self.arm, angle=ArmConstants.kArmSafeStartingAngle)
        if level == "4":
            return MoveElevatorAndArm(self.elevator, 30.0, arm=self.arm, angle=135)

        assert False, f"level={level} is not supported"


    @staticmethod
    def ejectGamepiece(self, speed=0.3, timeoutSeconds=0.3):
        from commands.intakecommands import IntakeFeedGamepieceForward
        return IntakeFeedGamepieceForward(self.intake, speed=speed).withTimeout(timeoutSeconds)


    @staticmethod
    def alignToTagSim(self, headingDegrees, branch, speed, pushFwdSpeed, pushFwdSeconds):
        # no camera use in simulation
        fwd = SwerveMove(metersToTheLeft=0, metersBackwards=-0.4, speed=speed, drivetrain=self.robotDrive)
        align = AimToDirection(headingDegrees, drivetrain=self.robotDrive)
        push = SwerveMove(metersToTheLeft=0, metersBackwards=-99, drivetrain=self.robotDrive, speed=pushFwdSpeed)
        return fwd.andThen(align).andThen(push.withTimeout(pushFwdSeconds))


    @staticmethod
    def clearDashboard(self):
        fieldDashboard = self.robotDrive.field
        if fieldDashboard is not None:
            fieldDashboard.getObject("start").setPoses([])
            fieldDashboard.getObject("approaching").setPoses([])
            fieldDashboard.getObject("score").setPoses([])
            fieldDashboard.getObject("retreating").setPoses([])
            fieldDashboard.getObject("retreated").setPoses([])


    @staticmethod
    def updateDashboard(self):
        fieldDashboard = self.robotDrive.field
        if fieldDashboard is not None:
            start = self.startPos.getSelected()
            sX, sY, sDeg = start
            goal1traj = self.goal1traj.getSelected()
            goal1branch = self.goal1branch.getSelected()

            heading, approach, retreat = goal1traj(self, start=start, branch=goal1branch)
            display = lambda t: t.trajectoryToDisplay() if hasattr(t, "trajectoryToDisplay") else []
            approach, retreat = display(approach), display(retreat)

            fieldDashboard.getObject("start").setPoses([Pose2d(Translation2d(sX, sY), Rotation2d.fromDegrees(sDeg))])
            fieldDashboard.getObject("approaching").setPoses(interpolate(approach))
            fieldDashboard.getObject("score").setPoses(scorePoint(approach, heading))
            fieldDashboard.getObject("retreating").setPoses(interpolate(retreat))
            fieldDashboard.getObject("retreated").setPoses(retreat[-1:])


def interpolate(poses, chunks=10):
    result = []
    prev: Pose2d = None
    for pose in poses:
        if prev is None:
            result.append(pose)
        else:
            vector = pose.translation() - prev.translation()
            points = [prev.translation() + (vector * float((1 + chunk) / chunks)) for chunk in range(chunks)]
            result.extend([Pose2d(i, Rotation2d()) for i in points])
        prev = pose
    return result


def scorePoint(approachPoses, headingDegrees, distance=0.8):
    if not approachPoses:
        return []
    startPose = approachPoses[-1]
    heading = Rotation2d.fromDegrees(headingDegrees)
    location = startPose.translation() + Translation2d(distance, 0).rotateBy(heading)
    return [Pose2d(location, heading)]


def autoStatus(text) -> Command:
    return InstantCommand(lambda: SmartDashboard.putString("autoStatus", text))

def runCmd(text, command):
    return autoStatus(text).andThen(command)
