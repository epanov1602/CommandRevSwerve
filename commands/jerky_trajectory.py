#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import typing

import commands2

from subsystems.drivesubsystem import DriveSubsystem
from commands.aimtodirection import AimToDirection
from commands.swervetopoint import SwerveToPoint
from commands.gotopoint import GoToPoint

from wpimath.geometry import Pose2d, Rotation2d, Translation2d


class JerkyTrajectory(commands2.Command):
    def __init__(
        self,
        drivetrain: DriveSubsystem,
        endpoint: Pose2d | Translation2d | tuple | list,
        waypoints: typing.List[Pose2d | Translation2d | tuple | list] = (),
        swerve: bool = False,
        speed=1.0,
    ):
        """
        A simple trajectory command that automatically skips all the waypoints that are already behind
        (this is why it is better to have start point should be listed in waypoints)
        :param swerve: do we want to use the swerve functionality (otherwise it will use tank/arcade drive)
        :param drivetrain: drive train, swerve or tank/arcade
        :param endpoint: end point of the trajectory, can be (X,Y) tuple, (X,Y,heading) tuple, Translation2d or Pose2d
        :param waypoints: a list of waypoints (if any), can be (X,Y) tuples, (X,Y,heading), Translation2d or Pose2d
        :param speed: maximum allowed driving speed (can be smaller than max speed of the drivetrain)
        """
        super().__init__()
        assert endpoint is not None

        self.drivetrain = drivetrain
        self.speed = speed
        self.swerve = swerve
        self.waypoints = [self._makeWaypoint(w) for w in waypoints] + [self._makeWaypoint(endpoint)]
        assert len(self.waypoints) > 0
        self.command = None

        self.addRequirements(self.drivetrain)

    def initialize(self):
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
            commands.append(AimToDirection(direction.degrees(), speed=self.speed, drivetrain=self.drivetrain))

        # connect all these commands together and start
        self.command = commands2.SequentialCommandGroup(*commands)
        self.command.initialize()

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
                heading = Rotation2d(heading)
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
        if self.swerve:
            return SwerveToPoint(
                point.x, point.y, heading, drivetrain=self.drivetrain, speed=self.speed, slowDownAtFinish=last
            )
        else:
            return GoToPoint(
                point.x, point.y, drivetrain=self.drivetrain, speed=self.speed, slowDownAtFinish=last
            )
