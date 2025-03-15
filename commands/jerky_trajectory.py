#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import typing

import commands2
from commands2 import InstantCommand

from subsystems.drivesubsystem import DriveSubsystem
from commands.aimtodirection import AimToDirection
from commands.swervetopoint import SwerveToPoint
from commands.gotopoint import GoToPoint

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import SmartDashboard


class JerkyTrajectory(commands2.Command):
    def __init__(
        self,
        drivetrain: DriveSubsystem,
        endpoint: Pose2d | Translation2d | tuple | list | None,
        waypoints: typing.List[Pose2d | Translation2d | tuple | list] = (),
        swerve: bool | str = False,
        speed=1.0,
        setup: typing.Optional[typing.Callable[[], None]] = None,
        stopAtEnd=True,
    ):
        """
        A simple trajectory command that automatically skips all the waypoints that are already behind
        (this is why it is better to have start point should be listed in waypoints)
        :param swerve: do we want to use the swerve functionality (you can also say swerve="last-point")
        :param drivetrain: drive train, swerve or tank/arcade
        :param endpoint: end point of the trajectory, can be (X,Y) tuple, (X,Y,heading) tuple, Translation2d or Pose2d
        :param waypoints: a list of waypoints (if any), can be (X,Y) tuples, (X,Y,heading), Translation2d or Pose2d
        :param speed: maximum allowed driving speed (can be smaller than max speed of the drivetrain)
        """
        super().__init__()
        assert swerve in (
            False, True, "last-point"
        ), f"swerve={swerve} is not allowed (allowed: False, True, 'last-point')"

        if endpoint is None:
            assert len(waypoints) > 0, "if endpoint is None, waypoints cannot be empty"

        self.drivetrain = drivetrain
        self.speed = speed
        self.swerve = swerve
        self.stopAtEnd = stopAtEnd
        self.setup = setup
        self.waypoints = [self._makeWaypoint(w) for w in waypoints]
        if endpoint is not None:
            self.waypoints += [self._makeWaypoint(endpoint)]
        assert len(self.waypoints) > 0
        self.command = None

        self.addRequirements(self.drivetrain)

    def reversed(self):
        waypoints = self.waypoints[1:]
        waypoints.reverse()
        endpoint = self.waypoints[0]
        return JerkyTrajectory(self.drivetrain, endpoint, waypoints, self.swerve, -self.speed, self.setup)

    def trajectoryToDisplay(self):
        result = []
        for translation, rotation in self.waypoints:
            result.append(Pose2d(translation, rotation if rotation is not None else Rotation2d()))
        return result

    def initialize(self):
        if self.setup is not None:
            self.setup()

        # skip the waypoints that are already behind
        waypoints = self.getRemainingWaypointsAheadOfUs()
        assert len(waypoints) > 0
        last = len(waypoints) - 1
        direction = None

        # make the commands connecting the waypoints which remain after skipping
        commands = []
        for index, (point, heading) in enumerate(waypoints):
            command = self._makeWaypointCommand(point, heading, index == last)
            direction = heading
            commands.append(command)

        # if the last waypoint (endpoint) has a specific direction, aim in that direction at the end
        if direction is not None:
            degrees = direction.degrees()
            log = lambda: SmartDashboard.putString("command/c" + self.__class__.__name__, f"aim: {degrees}")
            aim = AimToDirection(degrees, speed=self.speed, drivetrain=self.drivetrain).beforeStarting(log)
            commands.append(aim)

        # connect all these commands together and start
        self.command = commands2.SequentialCommandGroup(*commands)
        self.command.initialize()

    def getRemainingWaypointsAheadOfUs(self):
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")
        # find the waypoint nearest to the current location: we want to skip all the waypoints before it
        nearest, distance = None, None
        location = self.drivetrain.getPose()
        for point, heading in self.waypoints:
            d = location.translation().distance(point)
            if nearest is None or d < distance:
                nearest, distance = point, d
        # only use the waypoints that we must not skip (those past the nearest waypoint, i.e. not already behind)
        waypoints = []
        if nearest is not None:
            skip = True
            for point, heading in self.waypoints:
                if not skip:
                    waypoints.append((point, heading))
                elif point == nearest:
                    skip = False
        if len(waypoints) == 0:
            waypoints = [self.waypoints[-1]]
        return waypoints

    def execute(self):
        if self.command is not None:
            self.command.execute()

    def isFinished(self) -> bool:
        if self.command is not None:
            return self.command.isFinished()

    def end(self, interrupted: bool):
        if self.command is not None:
            self.command.end(interrupted)
            self.command = None
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")
        else:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "finished")

    def _makeWaypoint(self, waypoint):
        point, heading = None, None

        # did the user give us (X, Y) but not heading?
        if isinstance(waypoint, (tuple, list)) and len(waypoint) == 2:
            point, heading = waypoint[0], waypoint[1]
            if not isinstance(point, Translation2d):
                point = Translation2d(waypoint[0], waypoint[1])
                heading = None

        # did the user give us (X, Y) and heading?
        elif isinstance(waypoint, (tuple, list)) and len(waypoint) == 3:
            heading = waypoint[2]
            if isinstance(heading, (float, int)):
                heading = Rotation2d.fromDegrees(heading)
            point = Translation2d(waypoint[0], waypoint[1])

        # did the user just give us a Translation2d with (X, Y) inside?
        elif isinstance(waypoint, Translation2d):
            point = waypoint

        # did the user give us a Rotation2d with (X, Y, heading) inside?
        elif isinstance(waypoint, Pose2d):
            point = waypoint.translation()
            heading = waypoint.rotation()

        # here type(point) is Translation2d and type(heading) is Rotation2d or None
        return point, heading

    def _makeWaypointCommand(self, point, heading, last):
        slowdown = last and self.stopAtEnd
        if not self.swerve or (not last and self.swerve == "last-point"):
            # use arcade drive, if we aren't allowed to use swerve (or not allowed to use swerve for non-end points)
            command = GoToPoint(
                point.x, point.y, drivetrain=self.drivetrain, speed=self.speed, slowDownAtFinish=slowdown
            )
        else:
            command = SwerveToPoint(
                point.x, point.y, heading, drivetrain=self.drivetrain, speed=self.speed, slowDownAtFinish=slowdown, rateLimit=not last
            )

        log = lambda: SmartDashboard.putString("command/c" + self.__class__.__name__, f"next: {point.x}, {point.y}")
        return InstantCommand(log).andThen(command)


class SwerveTrajectory(JerkyTrajectory):

    def reversed(self):
        waypoints = self.waypoints[1:]
        waypoints.reverse()
        endpoint = self.waypoints[0]
        return SwerveTrajectory(self.drivetrain, endpoint, waypoints, self.swerve, -self.speed, self.setup)

    def initialize(self):
        if self.setup is not None:
            self.setup()

        # skip the waypoints that are already behind
        waypoints = self.getRemainingWaypointsAheadOfUs()
        assert len(waypoints) > 0
        endPt, endHeading = waypoints[-1]

        import math
        from constants import DriveConstants, AutoConstants
        from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
        from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController

        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond * abs(self.speed),
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the current point
            self.drivetrain.getPose(),
            # Pass through these interior waypoints
            [w[0] for w in waypoints[0:-1]],
            # End 1.5 meters straight ahead of where we started, facing forward
            Pose2d(endPt, endHeading),
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
            trajectory,
            self.drivetrain.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            driveController,
            self.drivetrain.setModuleStates,
            (self.drivetrain,),
        )

        # If the robot fell behind the trajectory (trajectory speed too high in optimizer), brute-force catch up @ end
        catchup = SwerveToPoint(
            endPt.x, endPt.y, endHeading, drivetrain=self.drivetrain, speed=self.speed, slowDownAtFinish=True
        ).beforeStarting(
            lambda: SmartDashboard.putString("command/c" + self.__class__.__name__, f"catchup: {endPt.x}, {endPt.y}")
        )

        # Run path following command, then stop at the end (possibly turn in correct direction)
        if endHeading is not None:
            degrees = endHeading.degrees()
            log = lambda: SmartDashboard.putString("command/c" + self.__class__.__name__, f"aim: {degrees}")
            stop = AimToDirection(degrees, speed=self.speed, drivetrain=self.drivetrain).beforeStarting(log)
        else:
            stop = InstantCommand(self.drivetrain.stop, self.drivetrain)

        self.command = swerveControllerCommand.andThen(catchup).andThen(stop)
        self.command.initialize()


def mirror(waypoints, fieldWidth=8.052):
    """
    Converts right-side trajectory into left-side trajectory
    :param waypoints: original trajectory, list of tuples of (x, y, heading) or (x, y)
    :param fieldWidth: width of the field
    :return: a mirror image of trajectory waypoints
    """
    # a tuple is treated as a single waypoint
    if isinstance(waypoints, tuple):
        return mirror([waypoints])[0]

    def reflect(heading):
        if heading is not None:
            return heading * -1.0

    result = []
    for point in waypoints:
        if len(point) == 2 and isinstance(point[0], Translation2d):
            location, heading = point
            result.append((Translation2d(location.x, fieldWidth - location.y), reflect(heading)))
        elif len(point) == 2:
            x, y = point
            result.append((x, fieldWidth - y))
        elif len(point) == 3:
            x, y, heading = point
            result.append((x, fieldWidth - y, reflect(heading)))
        else:
            assert False, f"unknown waypoint format: {point}"

    return result