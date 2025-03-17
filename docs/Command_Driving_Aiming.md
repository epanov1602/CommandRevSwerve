# Autonomous Driving and Aiming Command Examples
(but you can also bind them to teleop buttons too: in `robotcontainer.py`, inside `configureButtonBindings` function)

## 1. Pre-requisites
Does your robot already have a drivetrain with odometry? (tank or swerve)

If not, why not copy (or fork) of one of our templates (they are public, with WPI license):

* Swerve drivetrain example: https://www.youtube.com/watch?v=44iMiQXQH5U
* Tank/Arcade drivetrain example: https://www.youtube.com/watch?v=DhYMjLz0ync

## 2. Three must-have commands, for all else to work (even on swerve bots)
**Must have!** These commands have important constants that may need to be adjusted to your robot.

Click on triangle signs below, in order to see the command code:

<details>
<summary>A command to set the (X, Y) position of robot on the field at the start of the game</summary>
Please try to put this code in file `commands/reset_xy.py` :

```python
from __future__ import annotations
import commands2

from wpimath.geometry import Rotation2d, Pose2d, Translation2d


class ResetXY(commands2.Command):
    def __init__(self, x, y, headingDegrees, drivetrain):
        """
        Reset the starting (X, Y) and heading (in degrees) of the robot to where they should be.
        :param x: X
        :param y: X
        :param headingDegrees: heading (for example: 0 = "North" of the field, 180 = "South" of the field)
        :param drivetrain: drivetrain on which the (X, Y, heading) should be set
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.position = Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(headingDegrees))
        self.addRequirements(drivetrain)

    def initialize(self):
        self.drivetrain.resetOdometry(self.position)

    def isFinished(self) -> bool:
        return True  # this is an instant command, it finishes right after it initialized

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """


class ResetSwerveFront(commands2.Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        pose = self.drivetrain.getPose()
        self.drivetrain.resetOdometry(pose)

    def isFinished(self) -> bool:
        return True  # this is an instant command, it finishes right after it initialized

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """
```
</details>

<details>
<summary>Aiming in certain direction (for example, 0 degrees = North, 180 degrees = South, etc)</summary>

Please try to put this code in file `commands/aimtodirection.py`:
```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import math
import typing
import commands2

from wpimath.geometry import Rotation2d
from constants import AutoConstants


class AimToDirectionConstants:
    kP = 0.002  # 0.002 is the default, but you must calibrate this to your robot
    kUseSqrtControl = AutoConstants.kUseSqrtControl

    kMinTurnSpeed = 0.03  # turning slower than this is unproductive for the motor (might not even spin)
    kAngleToleranceDegrees = 2.0  # plus minus 2 degrees is "close enough"
    kAngleVelocityToleranceDegreesPerSec = 50  # velocity under 100 degrees/second is considered "stopped"


class AimToDirection(commands2.Command):
    def __init__(self, degrees: float | typing.Callable[[], float], drivetrain, speed=1.0, fwd_speed=0.0):
        super().__init__()

        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.speed = min((1.0, abs(speed)))
        self.targetDirection = None
        self.fwdSpeed = fwd_speed

        # setting the target angle in a way that works for all cases
        self.targetDegrees = degrees
        if degrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(degrees):
            self.targetDegrees = lambda: degrees

    def initialize(self):
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())

    def execute(self):
        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()

        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed and self.fwdSpeed == 0:
            turnSpeed = AimToDirectionConstants.kMinTurnSpeed  # but not too small

        # 3. act on it! if target angle is on the right, turn right
        if degreesRemaining > 0:
            self.drivetrain.arcadeDrive(self.fwdSpeed, +turnSpeed)
        else:
            self.drivetrain.arcadeDrive(self.fwdSpeed, -turnSpeed)  # otherwise, turn left

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        if self.fwdSpeed != 0:
            return False   # if someone wants us to drive forward while aiming, then we are never finished

        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # if we are pretty close to the direction we wanted, consider the command finished
        if abs(degreesRemaining) < AimToDirectionConstants.kAngleToleranceDegrees:
            turnVelocity = self.drivetrain.getTurnRateDegreesPerSec()
            if abs(turnVelocity) < AimToDirectionConstants.kAngleVelocityToleranceDegreesPerSec:
                return True

```
</details>


<details>
<summary>Driving to a certain (X, Y) point (use negative `speed` to drive in reverse)</summary>

Please try to put this code in file `commands/gotopoint.py`:
```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2
import math

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d, Translation2d

from commands.aimtodirection import AimToDirectionConstants
from constants import AutoConstants

class GoToPointConstants:
    kPTranslate = 0.25  # make it 0.2 to be conservative?  # you will need to calibrate this one to your robot
    kUseSqrtControl = AutoConstants.kUseSqrtControl

    kMinTranslateSpeed = 0.035  # moving forward slower than this is unproductive
    kApproachRadius = 0.2  # within this radius from target location, try to point in desired direction
    kOversteerAdjustment = 0.5


class GoToPoint(commands2.Command):
    def __init__(self, x, y, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True, finishDirection=None) -> None:
        """
        Go to a point with (X, Y) coordinates. Whether this is the end of your trajectory or not.
        :param x:
        :param y:
        :param drivetrain:
        :param speed: between -1.0 and +1.0 (you can use negative speed to drive backwards)
        :param finishDirection: Rotation2d for robot direction at the finish point, example: Rotation2d.fromDegrees(-70)
        :param slowDownAtFinish:
        """
        super().__init__()
        self.targetPosition = Translation2d(x, y)
        self.initialPosition = None
        self.speed = speed
        self.stop = slowDownAtFinish
        self.desiredEndDirection = None
        self.initialDistance = None
        self.pointingInGoodDirection = False
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.finishDirection = finishDirection
        if self.speed < 0 and self.finishDirection is not None:
            self.finishDirection = self.finishDirection.rotateBy(GoToPoint.REVERSE_DIRECTION)

    def initialize(self):
        self.initialPosition = self.drivetrain.getPose().translation()
        if self.finishDirection is not None:
            self.desiredEndDirection = self.finishDirection
        else:
            initialDirection = self.targetPosition - self.initialPosition
            self.desiredEndDirection = Rotation2d(initialDirection.x, initialDirection.y)
        if self.speed < 0:
            self.desiredEndDirection = self.desiredEndDirection.rotateBy(GoToPoint.REVERSE_DIRECTION)
        self.initialDistance = self.initialPosition.distance(self.targetPosition)
        self.pointingInGoodDirection = False

    def execute(self):
        # 1. to which direction we should be pointing?
        currentPose = self.drivetrain.getPose()
        currentDirection = currentPose.rotation()
        currentPoint = currentPose.translation()
        targetDirectionVector = self.targetPosition - currentPoint
        targetDirection = Rotation2d(targetDirectionVector.x, targetDirectionVector.y)
        if self.speed < 0:
            targetDirection = targetDirection.rotateBy(GoToPoint.REVERSE_DIRECTION)
        degreesRemaining = _optimize((targetDirection - currentDirection).degrees())
        rotateSpeed = min([abs(self.speed), AimToDirectionConstants.kP * abs(degreesRemaining)])

        # 2. if we are pointing in a very wrong direction (more than 45 degrees away), rotate away without moving
        if degreesRemaining > 45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, rotateSpeed)
            return
        elif degreesRemaining < -45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, -rotateSpeed)
            return

        self.pointingInGoodDirection = True

        # 3. otherwise, drive forward but with an oversteer adjustment (better way is to use RAMSETE unicycle)
        distanceRemaining = self.targetPosition.distance(currentPoint)
        if distanceRemaining < GoToPointConstants.kApproachRadius:
            targetDirection = self.desiredEndDirection  # avoid wiggling the direction when almost there
            degreesRemaining = _optimize((targetDirection - currentDirection).degrees())

        elif GoToPointConstants.kOversteerAdjustment != 0:
            deviationFromInitial = _optimize((targetDirection - self.desiredEndDirection).degrees())
            adjustment = GoToPointConstants.kOversteerAdjustment * deviationFromInitial
            if adjustment > 30: adjustment = 30  # avoid oscillations by capping the adjustment at 30 degrees
            if adjustment < -30: adjustment = -30  # avoid oscillations by capping the adjustment at 30 degrees
            targetDirection = targetDirection.rotateBy(Rotation2d.fromDegrees(adjustment))
            degreesRemaining = _optimize((targetDirection - currentDirection).degrees())
            # SmartDashboard.putNumber("z-heading-target", targetDirection.degrees())

        # 4. now when we know the desired direction, we can compute the turn speed
        rotateSpeed = abs(self.speed)
        proportionalRotateSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalRotateSpeed = math.sqrt(0.5 * proportionalRotateSpeed)  # will match the non-sqrt value when 50% max speed
        if rotateSpeed > proportionalRotateSpeed:
            rotateSpeed = proportionalRotateSpeed

        # 5. but if not too different, then we can drive while turning
        proportionalTransSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        if GoToPointConstants.kUseSqrtControl:
            proportionalTransSpeed = math.sqrt(0.5 * proportionalTransSpeed)

        translateSpeed = abs(self.speed)  # if we don't plan to stop at the end, go at max speed
        if translateSpeed > proportionalTransSpeed and self.stop:
            translateSpeed = proportionalTransSpeed  # if we plan to stop at the end, slow down when close
        if translateSpeed < GoToPointConstants.kMinTranslateSpeed:
            translateSpeed = GoToPointConstants.kMinTranslateSpeed
        if self.speed < 0:
            translateSpeed = -translateSpeed  # negative translation speed if supposed to go in reverse

        # 6. if we need to be turning *right* while driving, use negative rotation speed
        if degreesRemaining < 0:
            self.drivetrain.arcadeDrive(translateSpeed, -rotateSpeed)
        else:  # otherwise, use positive
            self.drivetrain.arcadeDrive(translateSpeed, +rotateSpeed)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        # 1. did we reach the point where we must move very slow?
        currentPose = self.drivetrain.getPose()
        currentPosition = currentPose.translation()
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)

        if not self.stop and distanceFromInitialPosition > self.initialDistance - GoToPointConstants.kApproachRadius:
            return True  # close enough

        distanceRemaining = self.targetPosition.distance(currentPosition)
        translateSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        if GoToPointConstants.kUseSqrtControl:
            translateSpeed = math.sqrt(0.5 * translateSpeed)

        # 1. have we reached the point where we are moving very slowly?
        tooSlowNow = translateSpeed < 0.125 * GoToPointConstants.kMinTranslateSpeed and self.stop

        # 2. did we overshoot?
        if distanceFromInitialPosition >= self.initialDistance or tooSlowNow:
            return True  # we overshot or driving too slow

    REVERSE_DIRECTION = Rotation2d.fromDegrees(180)


def _optimize(degrees):
    while degrees > 180:  # for example, if we have 350 degrees to turn left, we probably want -10 degrees right
        degrees -= 360

    while degrees < -180:  # for example, if we have -350 degrees to turn right, we probably want +10 degrees left
        degrees += 360

    return degrees
```
</details>

## 3. Another must-have command, if you have swerve drive
<details>
<summary>"Swerving" to a point (X, Y), and you can optionally add the "heading" direction (0 degrees = North, etc.)</summary>
Please try to put this code in file `commands/swervetopoint.py`:

```python

#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2
import math

from subsystems.drivesubsystem import DriveSubsystem
from commands.aimtodirection import AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpilib import SmartDashboard


class SwerveToPoint(commands2.Command):
    def __init__(self, x, y, headingDegrees, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True, rateLimit=False) -> None:
        super().__init__()
        self.targetPose = None
        self.targetPoint = Translation2d(x, y)
        if isinstance(headingDegrees, Rotation2d):
            self.targetHeading = headingDegrees
        elif headingDegrees is not None:
            self.targetHeading = Rotation2d.fromDegrees(headingDegrees)
        else:
            self.targetHeading = None

        self.speed = speed
        self.stop = slowDownAtFinish
        self.rateLimit = rateLimit
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.initialPosition = None
        self.initialDistance = None
        self.overshot = False

    def initialize(self):
        initialPose = self.drivetrain.getPose()
        self.initialPosition = initialPose.translation()

        targetHeading = initialPose.rotation() if self.targetHeading is None else self.targetHeading
        self.targetPose = Pose2d(self.targetPoint, targetHeading)

        self.initialDistance = self.initialPosition.distance(self.targetPose.translation())
        self.overshot = False

        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")

    def execute(self):
        currentXY = self.drivetrain.getPose()
        xDistance, yDistance = self.targetPose.x - currentXY.x, self.targetPose.y - currentXY.y
        totalDistance = self.targetPose.translation().distance(currentXY.translation())

        totalSpeed = abs(self.speed)
        if self.stop:  # proportional control: start slowing down if close to finish
            totalSpeed = GoToPointConstants.kPTranslate * totalDistance
            if GoToPointConstants.kUseSqrtControl:
                totalSpeed = math.sqrt(0.5 * totalSpeed)

        if totalSpeed > abs(self.speed):
            totalSpeed = abs(self.speed)
        if totalSpeed < GoToPointConstants.kMinTranslateSpeed:
            totalSpeed = GoToPointConstants.kMinTranslateSpeed

        # distribute the total speed between x speed and y speed
        xSpeed, ySpeed = 0, 0
        if totalDistance > 0:
            xSpeed = totalSpeed * xDistance / totalDistance
            ySpeed = totalSpeed * yDistance / totalDistance

        degreesLeftToTurn = self.getDegreesLeftToTurn()
        turningSpeed = abs(degreesLeftToTurn) * AimToDirectionConstants.kP
        if AimToDirectionConstants.kUseSqrtControl:
            turningSpeed = math.sqrt(0.5 * turningSpeed)  # will match the non-sqrt value when 50% max speed
        if turningSpeed > abs(self.speed):
            turningSpeed = abs(self.speed)
        if degreesLeftToTurn < 0:
            turningSpeed = -turningSpeed

        # now rotate xSpeed and ySpeed into robot coordinates
        speed = Translation2d(x=xSpeed, y=ySpeed).rotateBy(-self.drivetrain.getHeading())

        self.drivetrain.drive(speed.x, speed.y, turningSpeed, fieldRelative=False, rateLimit=self.rateLimit)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")

    def isFinished(self) -> bool:
        currentPose = self.drivetrain.getPose()
        currentPosition = currentPose.translation()

        # did we overshoot?
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)
        if not self.stop and distanceFromInitialPosition > self.initialDistance - GoToPointConstants.kApproachRadius:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "acceptable")
            return True  # close enough

        if distanceFromInitialPosition > self.initialDistance:
            if not self.overshot:
                SmartDashboard.putString("command/c" + self.__class__.__name__, "overshooting")
            self.overshot = True

        if self.overshot:
            distanceFromTargetDirectionDegrees = self.getDegreesLeftToTurn()
            if abs(distanceFromTargetDirectionDegrees) < 3 * AimToDirectionConstants.kAngleToleranceDegrees:
                SmartDashboard.putString("command/c" + self.__class__.__name__, "completed")
                return True  # case 2: overshot in distance and target direction is correct

    def getDegreesLeftToTurn(self):
        # can we get rid of this function by using Rotation2d? probably we can

        currentHeading = self.drivetrain.getPose().rotation()
        degreesLeftToTurn = (self.targetPose.rotation() - currentHeading).degrees()

        # if we have +350 degrees left to turn, this really means we have -10 degrees left to turn
        while degreesLeftToTurn > 180:
          degreesLeftToTurn -= 360

        # if we have -350 degrees left to turn, this really means we have +10 degrees left to turn
        while degreesLeftToTurn < -180:
          degreesLeftToTurn += 360

        return degreesLeftToTurn


class SwerveMove(commands2.Command):
    def __init__(
        self,
        metersToTheLeft: float,
        metersBackwards: float,
        drivetrain: DriveSubsystem,
        speed=1.0,
        heading=None
    ) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.speed = speed
        self.metersToTheLeft = metersToTheLeft
        self.metersBackwards = metersBackwards
        self.desiredHeading = heading
        self.subcommand = None

    def initialize(self):
        position = self.drivetrain.getPose()
        heading = self.desiredHeading if self.desiredHeading is not None else position.rotation()
        tgt = position.translation() + Translation2d(x=-self.metersBackwards, y=self.metersToTheLeft).rotateBy(heading)
        self.subcommand = SwerveToPoint(
            x=tgt.x, y=tgt.y, headingDegrees=heading.degrees(), drivetrain=self.drivetrain, speed=self.speed
        )
        self.subcommand.initialize()

    def isFinished(self) -> bool:
        return self.subcommand.isFinished()

    def execute(self):
        return self.subcommand.execute()

    def end(self, interrupted: bool):
        self.subcommand.end(interrupted)


SwerveToSide = SwerveMove

```
</details>

## 4. Curved trajectory to some point (simplistic)

<details>
<summary>Trajectory to some endpoint via "waypoints" (skips waypoints that are already behind)</summary>
    
Please try to put this code in file `commands/jerky_trajectory.py` :

```python

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


class JerkyTrajectory(commands2.Command):
    def __init__(
        self,
        drivetrain: DriveSubsystem,
        endpoint: Pose2d | Translation2d | tuple | list,
        waypoints: typing.List[Pose2d | Translation2d | tuple | list] = (),
        swerve: bool | str = False,
        speed=1.0,
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
        assert endpoint is not None
        assert swerve in (
            False, True, "last-point"
        ), f"swerve={swerve} is not allowed (allowed: False, True, 'last-point')"

        self.drivetrain = drivetrain
        self.speed = speed
        self.swerve = swerve
        self.waypoints = [self._makeWaypoint(w) for w in waypoints] + [self._makeWaypoint(endpoint)]
        assert len(self.waypoints) > 0
        self.command = None

        self.addRequirements(self.drivetrain)

    def reversed(self) -> JerkyTrajectory:
        waypoints = self.waypoints[1:]
        waypoints.reverse()
        endpoint = self.waypoints[0]
        return JerkyTrajectory(self.drivetrain, endpoint, waypoints, self.swerve, -self.speed)

    def trajectoryToDisplay(self):
        result = []
        for translation, rotation in self.waypoints:
            result.append(Pose2d(translation, rotation if rotation is not None else Rotation2d()))
        return result

    def initialize(self):
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
            commands.append(AimToDirection(direction.degrees(), speed=self.speed, drivetrain=self.drivetrain))

        # connect all these commands together and start
        self.command = commands2.SequentialCommandGroup(*commands)
        self.command.initialize()

    def getRemainingWaypointsAheadOfUs(self):
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
        if not self.swerve or (not last and self.swerve == "last-point"):
            # use arcade drive, if we aren't allowed to use swerve (or not allowed to use swerve for non-end points)
            return GoToPoint(
                point.x, point.y, drivetrain=self.drivetrain, speed=self.speed, slowDownAtFinish=last
            )
        else:
            return SwerveToPoint(
                point.x, point.y, heading, drivetrain=self.drivetrain, speed=self.speed, slowDownAtFinish=last, rateLimit=not last
            )


class SwerveTrajectory(JerkyTrajectory):

    def reversed(self) -> SwerveTrajectory:
        waypoints = self.waypoints[1:]
        waypoints.reverse()
        endpoint = self.waypoints[0]
        return SwerveTrajectory(self.drivetrain, endpoint, waypoints, self.swerve, -self.speed)

    def initialize(self):
        # skip the waypoints that are already behind
        waypoints = self.getRemainingWaypointsAheadOfUs()
        assert len(waypoints) > 0
        endPoint, endRotation = waypoints[-1]

        last = len(waypoints) - 1
        direction = None

        import math
        from constants import DriveConstants, AutoConstants
        from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
        from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController

        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
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
            Pose2d(endPoint, endRotation),
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

        # Run path following command, then stop at the end
        stop = InstantCommand(
            lambda: self.drivetrain.drive(0, 0, 0, False, False),
            self.drivetrain,
        )
        self.command = swerveControllerCommand.andThen(stop)
        self.command.initialize()

```
</details>


## 5. Using vision (cameras) to navigate
<details>
    <summary>Setting the camera pipeline (picking which objects to detect on camera)</summary>

This code works with Limelight or PhotonVision cameras from [here](Adding_Camera.md) (step-by-step video https://www.youtube.com/watch?v=8b9DZQ8CyII).

The code below should go to `commands/setcamerapipeline.py` .

```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2

class SetCameraPipeline(commands2.Command):

    def __init__(self, camera, pipelineIndex=0, onlyTagIds=()):
        super().__init__()
        self.pipelineIndex = pipelineIndex
        self.onlyTagIds = onlyTagIds
        self.camera = camera
        self.addRequirements(camera)

    def initialize(self):
        # if camera allows to set filter to look for specific tag IDs, filter for them
        if hasattr(self.camera, "setOnlyTagIds"):
            self.camera.setOnlyTagIds(self.onlyTagIds)
        # if camera has "setPipeline", set it
        if hasattr(self.camera, "setPipeline"):
            self.camera.setPipeline(self.pipelineIndex)

    def isFinished(self) -> bool:
        # if camera has no "setPipeline", we have nothing to wait for
        if not hasattr(self.camera, "setPipeline"):
            return True
        # we are finished when the camera has responded that pipeline index is now set
        if self.camera.getPipeline() == self.pipelineIndex:
            return True
        # we are in sim, and camera doesn't respond
        if commands2.TimedCommandRobot.isSimulation():
            return True
        # otherwise, print that we aren't finished
        print("SetCameraPipeline: not yet finished, because camera pipeline = {} and we want {}".format(
            self.camera.getPipeline(), self.pipelineIndex)
        )
```
</details>

<details>
    <summary>Setting the camera pipeline to the AprilTag in front of the robot</summary>

First you need to add `"apriltag"` into `robotpy_extras` in `pyproject.toml` file and run `sync` to get the AprilTag layouts installed.

Then, the code below should go to `commands/setcameragoal.py` .

```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpimath.geometry
from wpilib import Field2d
from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from os.path import isfile, join


# if you are using Limelight, probably need to set up the pipelines to match this
# (or you can provide your own `pipeline2TagGroup` that matches your Limelight, when constructing the command)
DEFAULT_PIPELINE_TO_TAGS = {
    0: (),  # all tags
    6: (6, 19),
    7: (7, 18),
    8: (8, 17),
    9: (9, 22),
    4: (10, 21),
    5: (11, 20),
}


class SetCameraToTagInFront(commands2.Command):
    MAX_DISTANCE_TO_TAG = 3  # meters (10 feet)

    def __init__(self, fieldLayoutFile, camera, drivetrain, robotLengthMeters, reverse=False, pipeline2TagGroup=None):
        """
        :param fieldLayoutFile: something like "2025-reefscape.json" or "2024-crescendo.json", you can download them at
         https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/
        """
        super().__init__()
        assert camera is not None

        self.camera = camera
        self.addRequirements(camera)

        self.drivetrain = drivetrain
        self.field2d: Field2d = getattr(self.drivetrain, "field", None)

        self.reverse = reverse
        if not reverse:
            self.toRobotFront = Transform2d(0.5 * robotLengthMeters, 0.0, Rotation2d.fromDegrees(0))
        else:
            self.toRobotFront = Transform2d(-0.5 * robotLengthMeters, 0.0, Rotation2d.fromDegrees(180))

        from robotpy_apriltag import AprilTagFieldLayout

        self.fieldLayout = None
        for prefix in ['/home/lvuser/py/', '/home/lvuser/py_new/', '']:
            candidate = join(prefix, fieldLayoutFile)
            print(f"Localizer: trying field layout from {candidate}")
            if isfile(candidate):
                self.fieldLayout = AprilTagFieldLayout(candidate)
                self.fieldLength = self.fieldLayout.getFieldLength()
                self.fieldWidth = self.fieldLayout.getFieldWidth()
                break
        assert self.fieldLayout is not None, f"file with field layout {fieldLayoutFile} does not exist"
        print(f"SetCameraToTagAhead: loaded field layout with {len(self.fieldLayout.getTags())} tags, from {fieldLayoutFile}")

        # which tag is in which pipeline?
        if pipeline2TagGroup is None:
            pipeline2TagGroup = DEFAULT_PIPELINE_TO_TAGS

        self.tagToPipeline = {}
        self.pipelineToTags = pipeline2TagGroup
        for pipeline, tags in pipeline2TagGroup.items():
            for tag in tags:
                pose = self.fieldLayout.getTagPose(tag)
                if pose is not None:
                    self.tagToPipeline[tag] = pipeline
        self.tagPoses = [(tagId, self.fieldLayout.getTagPose(tagId)) for tagId in self.tagToPipeline.keys()]
        self.tagPoses = [(tagId, pose.toPose2d()) for tagId, pose in self.tagPoses if pose is not None]

        # state
        self.chosenTagId = None
        self.chosenTagPose: Pose2d = None
        self.chosenPipeline = 0
        self.chosenTagIds = None


    def getChosenHeadingDegrees(self):
        if self.chosenTagPose is None:
            return None
        direction = self.chosenTagPose.rotation()
        if not self.reverse:  # robot direction should be opposite to tag's (we are supposed to face it)
            direction = direction.rotateBy(Rotation2d.fromDegrees(180))
        return direction.degrees()


    def recomputeChosenHeadingDegrees(self):
        self.pickChosenTagId()
        return self.getChosenHeadingDegrees()


    def initialize(self):
        self.pickChosenTagId()

        if hasattr(self.camera, "setOnlyTagIds"):
            self.camera.setOnlyTagIds(self.chosenTagIds)

        # if camera has "setPipeline", set it
        if hasattr(self.camera, "setPipeline"):
            self.camera.setPipeline(self.chosenPipeline)


    def isFinished(self) -> bool:
        # if camera has no "setPipeline", we have nothing to wait for
        if not hasattr(self.camera, "setPipeline"):
            return True
        # we are finished when the camera has responded that pipeline index is now set
        if self.camera.getPipeline() == self.chosenPipeline:
            return True
        # we are in sim, and camera doesn't respond
        if commands2.TimedCommandRobot.isSimulation():
            return True
        # otherwise, print that we aren't finished
        print("SetCameraPipeline: not yet finished, because camera pipeline is {} and we want {}".format(
            self.camera.getPipeline(), self.chosenPipeline)
        )


    def execute(self):
        pass


    def end(self, interrupted: bool):
        pass


    def reset(self):
        self.chosenTagId = None
        self.chosenTagPose = None
        self.chosenPipeline = 0
        self.chosenTagIds = None


    def pickChosenTagId(self):
        self.reset()

        # 0. where is the front of the robot
        front: Pose2d = self.drivetrain.getPose().transformBy(self.toRobotFront)
        self.drawArrow("front", front, flip=False)

        # 1. which of the tags is in front of us?
        self.chosenTagId = (
            self.findNearestVisibleTagInFront(front, fieldOfViewDegrees=50) or
            self.findNearestVisibleTagInFront(front, fieldOfViewDegrees=75)  # if all fails
        )

        # 2. decide on chosen pipeline and tags
        if self.chosenTagId is not None:
            self.chosenPipeline = self.tagToPipeline.get(self.chosenTagId)
            self.chosenTagIds = self.pipelineToTags.get(self.chosenPipeline)
            chosenTagPose = self.fieldLayout.getTagPose(self.chosenTagId)
            self.chosenTagPose = chosenTagPose.toPose2d() if chosenTagPose is not None else None

        if self.chosenPipeline is None:
            self.chosenPipeline = 0  # this is the default pipeline, if nothing is chosen (the camera better have it)
            print(f"SetCameraToTagAhead: no pipeline chosen => using 0, chosenTagIds={self.chosenTagIds}")

        # 3. display the result
        if self.field2d is not None:
            self.drawArrow("tag", self.chosenTagPose, flip=True)
        print(f"SetCameraToTagAhead: picked tag {self.chosenTagId}, facing {self.getChosenHeadingDegrees()} degrees")


    def findNearestVisibleTagInFront(self, front, fieldOfViewDegrees):
        bestTagId, bestDistance = None, None
        for tagId, tagPose in self.tagPoses:
            visible, distance = self.isTagVisibleFromHere(front, tagPose, fieldOfViewDegrees)
            if visible and (bestDistance is None or distance < bestDistance):
                bestDistance = distance
                bestTagId = tagId
        return bestTagId


    def isTagVisibleFromHere(self, front, tagPose, fieldOfViewDegrees):
        directionToTag = Transform2d(front, tagPose)
        vectorToTag = directionToTag.translation()

        distance = vectorToTag.norm()
        if distance > self.MAX_DISTANCE_TO_TAG:
            return False, None  # too far

        direction = vectorToTag.angle().degrees()
        if abs(direction) > fieldOfViewDegrees / 2:
            return False, None  # won't fit into our imaginary 50 degree field of view

        orientation = directionToTag.rotation().rotateBy(Rotation2d.fromDegrees(180) - vectorToTag.angle())
        if abs(orientation.degrees()) > 45:
            return False, None  # tag tilted too far to the side, not oriented towards us

        return True, distance


    def drawArrow(self, name: str, pose: Pose2d, flip):
        if self.field2d is None:
            return
        if pose is None:
            poses = []
        else:
            poses = [
                pose,
                pose.transformBy(Transform2d(0.03, 0, 0)),
                pose.transformBy(Transform2d(0.06, 0, 0)),
                pose.transformBy(Transform2d(0.09, 0, 0)),
                pose.transformBy(Transform2d(0.12, 0, 0)),
                pose.transformBy(Transform2d(0.15, 0, 0)),
                pose.transformBy(Transform2d(0.18, 0, 0)),
                pose.transformBy(Transform2d(0.21, 0, 0)),
                pose.transformBy(Transform2d(0.24, 0, 0)),
                pose.transformBy(Transform2d(0.27, 0, 0)),
                pose.transformBy(Transform2d(0.30, 0, 0)),
            ]
            if flip:
                poses += [
                    pose,
                    pose.transformBy(Transform2d(0.1, 0.1, 0)),
                    pose,
                    pose.transformBy(Transform2d(0.1, -0.1, 0)),
                ]
            else:
                poses += [
                    pose.transformBy(Transform2d(0.3, 0, 0)),
                    pose.transformBy(Transform2d(0.2, 0.1, 0)),
                    pose.transformBy(Transform2d(0.3, 0, 0)),
                    pose.transformBy(Transform2d(0.2, -0.1, 0)),
                ]
        self.field2d.getObject(name).setPoses(poses)

```
</details>

<details>
    <summary>Approaching (or pointing to) a visible gamepiece/tag/etc using camera</summary>

This code works with Limelight or PhotonVision cameras from [here](Adding_Camera.md) (step-by-step video https://www.youtube.com/watch?v=8b9DZQ8CyII).


The code below should go to `commands/followobject.py` .

```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math

import commands2

from commands.aimtodirection import AimToDirection, AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d
from wpilib import Timer


class FollowObject(commands2.Command):
    ANGLE_TOLERANCE = 50  # if pointing further away than this, do not move forward (rotate towards the object first)
    MIN_STEP_SECONDS = 0.038  # the robot is making 50 decisions per second, so we don't realistically want shorter step
    MIN_SPEED = GoToPointConstants.kMinTranslateSpeed

    def __init__(self,
                 camera,
                 drivetrain: DriveSubsystem,
                 stepSeconds=0.33,
                 stopWhen=None,
                 smoothness=1.0,
                 speed=1.0):

        super().__init__()

        self.camera0 = camera
        self.stopWhen = stopWhen

        self.initialStepSeconds = stepSeconds
        self.speed = atLeast(speed, FollowObject.MIN_SPEED)
        self.smoothness = smoothness
        self.drivetrain = drivetrain

        self.addRequirements(drivetrain)
        self.addRequirements(camera)

        self.finished = False
        self.drivingAllowed = True
        self.stepSeconds = self.initialStepSeconds

        self.halfStepTime = 0.0
        self.minDetectionIndex = None
        self.notSeenSinceWhen = None
        self.subcommand = None

    def initialize(self):
        self.finished = False
        self.stepSeconds = self.initialStepSeconds
        self.drivingAllowed = (self.initialStepSeconds > 0)  # driving allowed if we are allowed to make steps forward
        self.minDetectionIndex = None

    def execute(self):
        # 1. if there is subcommand to go in some direction, just work on executing it
        if self.subcommand is not None:
            if self.subcommand.isFinished():
                self.subcommand.end(False)  # if subcommand is finished, we must end() it
                self.subcommand = None  # and we don't have it anymore
            else:
                self.subcommand.execute()  # otherwise, the subcommand must run

        # 2. otherwise, look at the camera to find target direction, and make a subcommand to go in that direction
        if self.subcommand is None or Timer.getFPGATimestamp() > self.halfStepTime:
            directionInfo = self.findDirectionFromCamera()
            if directionInfo is not None:
                subcommand, stepSeconds = self.makeSubcommand(directionInfo)
                print("new subcommand {} seconds after halfstep, old {}".format(Timer.getFPGATimestamp() - self.halfStepTime, self.subcommand is not None))
                self.setSubcommand(subcommand, stepSeconds)

    def makeSubcommand(self, directionInfo):
        direction, x, y, size = directionInfo
        degreesFromTarget = (self.drivetrain.getPose().rotation() - direction).degrees()
        stepSeconds = self.initialStepSeconds
        speed = self.speed

        if abs(degreesFromTarget) < FollowObject.ANGLE_TOLERANCE and self.drivingAllowed:
            # 1. if robot is mostly aiming in correct direction already, make a step in that direction
            if self.stopWhen is not None:  # if pretty close to the object, drive slower and make finer steps
                slowdownFactor = self.stopWhen.reduceSpeedIfClose(x, y, max(size), smoothness=self.smoothness)
                stepSeconds = atLeast(stepSeconds * slowdownFactor, FollowObject.MIN_STEP_SECONDS)
                speed = atLeast(speed * slowdownFactor, FollowObject.MIN_SPEED)

            drive = AimToDirection(direction.degrees(), self.drivetrain, fwd_speed=speed)
            newSubcommand = drive.withTimeout(stepSeconds)  # correct timeout for the step
        else:
            # 2. otherwise, rotate if robot is pointing too far from the object (or if we aren't supposed to make steps)
            turn = AimToDirection(direction.degrees(), self.drivetrain)

            newSubcommand = turn

        return newSubcommand, stepSeconds

    def setSubcommand(self, subcommand, stepSeconds):
        self.minDetectionIndex = None  # invalidate the current detection, because the robot is about to move
        if self.subcommand is not None:
            self.subcommand.end(True)  # if previous subcommand is not finished yet, force-finish it
        self.subcommand = subcommand
        self.subcommand.initialize()
        self.halfStepTime = Timer.getFPGATimestamp() + stepSeconds / 2  # half a step later look again

    def end(self, interrupted: bool):
        if self.subcommand is not None:
            self.subcommand.end(interrupted)
            self.subcommand = None
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        return self.finished

    def findDirectionFromCamera(self):
        if self.finished:
            return

        index, x, y, size = self.camera0.getHB(), self.camera0.getX(), self.camera0.getY(), [self.camera0.getA()]
        if x == 0.0 and y == 0.0:
            x, y = None, None  # Limelight gives us 0 instead of None, when it didn't detect anything

        # 1. do we have a freshly detected object from the camera
        if self.minDetectionIndex is None:
            self.notSeenSinceWhen = Timer.getFPGATimestamp()
            self.minDetectionIndex = index + 1
            return  # we don't know if we are looking at an old video frame or fresh one => try again at the next frame
        if index < self.minDetectionIndex:
            return  # not a new detection, so we are still looking at an old frame
        if x is None and self.stopWhen is not None:
            now = Timer.getFPGATimestamp()
            if now > self.notSeenSinceWhen + self.stopWhen.secondsNotSeen:
                print(f"FollowObject: finished, cannot detect object for {now - self.notSeenSinceWhen} seconds")
                self.finished = True  # no hope: object not detected after waiting for long time
            return

        # 2. if we see the object well now, is it close enough to finish the approach?
        if self.stopWhen is not None:
            if not self.drivingAllowed and abs(x) < self.stopWhen.aimingToleranceDegrees:
                turnVelocity = self.drivetrain.getTurnRateDegreesPerSec()
                if abs(turnVelocity) < AimToDirectionConstants.kAngleVelocityToleranceDegreesPerSec:
                    print(f"FollowObject: finished, the object is pretty close (x={x}, turnVelocity={turnVelocity})")
                    self.finished = True  # already aiming at it pretty well and not allowed to move to it
                    return
            if self.drivingAllowed and self.stopWhen.isThisCloseToStopping(x, y, max(size)) >= 1:
                print(f"FollowObject: stop driving, the object is pretty close (x={x}, y={y}, size={size})")
                self.drivingAllowed = False  # looks like we should not be driving any further, maybe only turning to it

        # 3. otherwise we are not done: pick a target direction for the robot to turn towards (and go)
        directionToObjectCenter = Rotation2d.fromDegrees(-x)
        direction = self.drivetrain.getPose().rotation().rotateBy(directionToObjectCenter)
        return direction, x, y, size


class StopWhen:
    """
    How close is "close enough", for a given object-following command?
    """
    def __init__(self, maxY=999, minY=-999, maxSize=9999, aimingToleranceDegrees=3.0, secondsNotSeen=2.0):
        """
        When to stop object following
        :param maxY: if the "Y" (pitch) of the object is above this, finish
        :param minY: if the "Y" (pitch) of the object is below this, finish
        :param maxSize: if the angular size of the object is greater than this, finish
        :param aimingToleranceDegrees: if we aren't approaching but simply aiming (fwd_step=0), how close is enough?
        """
        self.maxY = maxY
        assert maxY > 0, f"only positive values allowed for maxY (not StopWhen(maxY={maxY}))"
        self.minY = minY
        assert minY < 0, f"only negative values allowed for minY (not StopWhen(minY={minY}))"
        self.maxSize = maxSize
        assert maxSize > 0, f"only positive values allowed for maxSize (not StopWhen(maxSize={maxSize}))"
        self.aimingToleranceDegrees = aimingToleranceDegrees
        assert aimingToleranceDegrees >= AimToDirectionConstants.kAngleToleranceDegrees, (
            "angleToleranceDegrees={} is not achievable since it is under {}"
        ).format(aimingToleranceDegrees, AimToDirectionConstants.kAngleToleranceDegrees)
        self.secondsNotSeen = secondsNotSeen
        assert self.secondsNotSeen > 0, f"invalid secondsNotSeen in StopWhen(secondsNotSeen={secondsNotSeen}), must>0"

    EPSILON_DEGREES = 10  # appropriate when field of view is 40 degrees

    def isThisCloseToStopping(self, x, y, size):
        """
        returns a value between 0.0 (not any close to stopping) and 1.0 (stop now!)
        """
        result = 0.0
        result = max([result, (y + StopWhen.EPSILON_DEGREES) / (self.maxY + StopWhen.EPSILON_DEGREES)])
        result = max([result, (-y + StopWhen.EPSILON_DEGREES) / (-self.minY + StopWhen.EPSILON_DEGREES)])
        result = max([result, size / self.maxSize])
        return min([1.0, result])

    def reduceSpeedIfClose(self, x, y, size, smoothness=1.0):
        if smoothness <= 0:
            return 1.0  # if we are not supposed to drive smoothly, don't reduce speed

        closeToStopping = self.isThisCloseToStopping(x, y, size)
        # ^^ if this number is close to 1.0, we should be reducing the speed

        percentage = (1.0 - closeToStopping) / smoothness
        # ^^ if this number is close to 0.0, we should be reducing the speed

        return min([percentage, 1.0])
        # ^^ don't return multipliers greater than 1.0, because we are not increasing speed here!


def atLeast(signedValue, lowestMagnitude):
    if abs(signedValue) > abs(lowestMagnitude):
        return signedValue
    # otherwise, return at least lowestMagnitude
    return math.copysign(abs(lowestMagnitude), signedValue)

```
</details>

<details>
    <summary>Rotating until you find an object using camera</summary>

This code works with Limelight or PhotonVision cameras from [here](Adding_Camera.md) (step-by-step video https://www.youtube.com/watch?v=8b9DZQ8CyII).

The code below should go to `commands/findobject.py` .

```python
import commands2
from commands.aimtodirection import AimToDirection


class FindObject(commands2.Command):

    def __init__(self, camera, drivetrain, turnDegrees=-45, turnSpeed=1.0, waitSeconds=0.1):
        super().__init__()
        self.camera0 = camera
        self.drivetrain = drivetrain
        self.addRequirements(camera)
        self.addRequirements(drivetrain)

        # to find object, we will be running (wait + turn) + (wait + turn) ... in a long cycle, until object is detected
        wait = commands2.WaitCommand(waitSeconds)
        turn = AimToDirection(
            degrees=lambda: self.drivetrain.getHeading().degrees() + turnDegrees,
            drivetrain=self.drivetrain,
            speed=turnSpeed
        )

        self.subcommand = wait.andThen(turn)

    def initialize(self):
        self.subcommand.initialize()

    def isFinished(self) -> bool:
        return self.camera0.hasDetection()

    def execute(self):
        # 1. just execute the wait+turn subcommand
        self.subcommand.execute()

        # 2. and if that command is finished, start it again (we are doing cycles, right?)
        if self.subcommand.isFinished():
            self.subcommand.end(False)
            self.subcommand.initialize()

    def end(self, interrupted: bool):
        self.subcommand.end(interrupted)

```

</details>

## 6. REEFSCAPE special command!
<details>
    <summary>Aligning a swerve robot to an AprilTag in front of it, and then firmly pushing against the tag to score that coral onto reef</summary>

This should go as a function in `robotcontainer.py`, and you can either bind it to a button or make an auto out of it:
```python3
   def makeAlignWithAprilTagCommand(self):
        from commands.approach import ApproachTag
        from commands.swervetopoint import SwerveToSide

        approach = ApproachTag(self.camera, self.robotDrive, None, speed=1.0, pushForwardSeconds=None)  # tuning this at speed=0.5, should be comfortable setting speed=1.0 instead

        # or you can do this, if you want to score the coral 15 centimeters to the right and two centimeters back from the AprilTag
        # stepToSide = SwerveToSide(drivetrain=self.robotDrive, metersToTheLeft=-0.15, metersBackwards=0.02, speed=0.2)
        # alignToScore = approach.andThen(stepToSide)

        return approach

```

This code works with Limelight or PhotonVision cameras from [here](Adding_Camera.md) (step-by-step video https://www.youtube.com/watch?v=8b9DZQ8CyII).

The command code can go to `commands/approach.py` :

```python
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import math
import commands2

from commands.aimtodirection import AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d, Translation2d
from wpilib import Timer, SmartDashboard, SendableChooser


class Tunable:
    _choosers = {}

    def __init__(self, settings, prefix, name, default, minMaxRange):
        if settings is not None:
            if name in settings:
                self.value = settings[name]
                self.chooser = None
                return
            if (prefix + name) in settings:
                self.value = settings[prefix + name]
                self.chooser = None
                return
        self.value = None
        self.chooser = Tunable._choosers.get(prefix + name)
        if self.chooser is not None:
            return
        # if that chooser was not created yet, create it now
        Tunable._choosers[prefix + name] = self.chooser = SendableChooser()
        for index, factor in enumerate([0, 0.1, 0.17, 0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0, 2.8, 4.0]):
            label, value = f"{factor * default}", factor * default
            if minMaxRange[0] <= value <= minMaxRange[1]:
                if factor == 1.0:
                    self.chooser.setDefaultOption(label, value)
                else:
                    self.chooser.addOption(label, value)
        SmartDashboard.putData(name, self.chooser)

    def fetch(self):
        if self.chooser is not None:
            self.value = self.chooser.getSelected()

    def __call__(self, *args, **kwargs):
        self.fetch()
        return self.value


class ApproachTag(commands2.Command):

    def __init__(
        self,
        camera,
        drivetrain,
        specificHeadingDegrees=None,
        speed=1.0,
        reverse=False,
        settings: dict | None=None,
        pushForwardSeconds=0.0,  # length of final approach
        finalApproachObjSize=10.0,
        detectionTimeoutSeconds=2.0,
        cameraMinimumFps=4.0,
        dashboardName="apch"
    ):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specificHeadingDegrees: do you want the robot to face in a very specific direction? then specify it
        :param speed: positive speed, even if camera is on the back of your robot (for the latter case set reverse=True)
        :param pushForwardSeconds: if you want the robot to do some kind of final approach at the end of alignment
        :param reverse: set it =True if the camera is on the back of the robot (not front)
        :param detectionTimeoutSeconds: if no detection within this many seconds, assume the tag is lost
        :param cameraMinimumFps: what is the minimal number of **detected** frames per second expected from this camera
        """
        super().__init__()
        assert hasattr(camera, "getX"), "camera must have `getX()` to give us the object coordinate (in degrees)"
        assert hasattr(camera, "getA"), "camera must have `getA()` to give us object size (in % of screen)"
        assert hasattr(camera, "getSecondsSinceLastHeartbeat"), "camera must have a `getSecondsSinceLastHeartbeat()`"
        assert hasattr(drivetrain, "drive"), "drivetrain must have a `drive()` function, because we need a swerve drive"

        self.drivetrain = drivetrain
        self.camera = camera
        self.addRequirements(drivetrain)
        self.addRequirements(camera)

        self.reverse = reverse
        self.approachSpeed = min((1.0, abs(speed)))  # ensure that the speed is between 0.0 and 1.0
        self.finalApproachObjSize = finalApproachObjSize
        self.pushForwardSeconds = pushForwardSeconds
        if self.pushForwardSeconds is None:
            self.pushForwardSeconds = Tunable(settings, dashboardName, "BrakeDst", 1.0, (0.0, 10.0))
        elif not callable(self.pushForwardSeconds):
            self.pushForwardSeconds = lambda : pushForwardSeconds

        self.finalApproachSpeed = None
        self.tagToFinalApproachPt = None  # will be assigned in initialize()

        assert detectionTimeoutSeconds > 0, f"non-positive detectionTimeoutSeconds={detectionTimeoutSeconds}"
        self.detectionTimeoutSeconds = detectionTimeoutSeconds

        assert cameraMinimumFps > 0, f"non-positive cameraMinimumFps={cameraMinimumFps}"
        self.frameTimeoutSeconds = 1.0 / cameraMinimumFps

        # setting the target heading in a way that works for all cases
        self.targetDegrees = specificHeadingDegrees
        if specificHeadingDegrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(specificHeadingDegrees):
            self.targetDegrees = lambda: specificHeadingDegrees

        # state
        self.targetDirection = None
        self.lastSeenObjectTime = None
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenDistanceToTag = None
        self.everSawObject = False
        self.tReachedGlidePath = 0.0  # time when aligned to the tag and desired direction for the first time
        self.tReachedFinalApproach = 0.0  # time when reached the final approach
        self.lostTag = ""
        self.finished = ""

        # debugging
        self.tStart = 0
        self.lastState = self.getState()
        self.lastWarnings = None

        self.initTunables(settings, dashboardName)


    def isReady(self, minRequiredObjectSize=0.3):
        return self.camera.hasDetection() and self.camera.getA() > minRequiredObjectSize


    def initTunables(self, settings, prefix):
        self.KPMULT_TRANSLATION = Tunable(settings, prefix, "GainTran", 0.7, (0.1, 8.0))  # gain for how quickly to move
        self.KPMULT_ROTATION = Tunable(settings, prefix, "GainRot", 0.5, (0.1, 8.0))  # gail for how quickly to rotate

        # acceptable width of glide path, in inches
        self.GLIDE_PATH_WIDTH_INCHES = Tunable(settings, prefix, "Tolernce", 2.0, (0.5, 4.0))

        # if plus minus 30 degrees from desired heading, forward motion is allowed before final approach
        self.DESIRED_HEADING_RADIUS = Tunable(settings, prefix, "Headng+-", 30, (5, 45))

        # shape pre final approach: 2 = use parabola for before-final-approach trajectory, 3.0 = use cubic curve, etc.
        self.APPROACH_SHAPE = Tunable(settings, prefix, "TrjShape", 3.0, (2.0, 8.0))

        self.tunables = [
            self.GLIDE_PATH_WIDTH_INCHES,
            self.DESIRED_HEADING_RADIUS,
            self.KPMULT_TRANSLATION,
            self.KPMULT_ROTATION,
            self.APPROACH_SHAPE,
        ]
        if isinstance(self.pushForwardSeconds, Tunable):
            self.tunables.append(self.pushForwardSeconds)


    def initialize(self):
        for t in self.tunables:
            t.fetch()

        kpMultTran = self.KPMULT_TRANSLATION.value
        print(f"ApproachTag: translation gain value {kpMultTran}, power={self.APPROACH_SHAPE.value}")

        targetDegrees = self.targetDegrees()
        if targetDegrees is None:
            targetDegrees = self.drivetrain.getHeading().degrees()

        self.targetDirection = Rotation2d.fromDegrees(targetDegrees)
        self.tReachedGlidePath = 0.0  # time when reached the glide path
        self.tReachedFinalApproach = 0.0  # time when reached the final approach
        self.lostTag = False
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenDistanceToTag = 999
        self.lastSeenObjectTime = Timer.getFPGATimestamp()
        self.everSawObject = False
        self.finished = ""

        # final approach parameters
        self.tagToFinalApproachPt = self.computeTagDistanceFromTagSizeOnFrame(self.finalApproachObjSize)
        self.finalApproachSpeed = 0
        self.finalApproachSeconds = max([0, self.pushForwardSeconds()])
        if self.finalApproachSeconds > 0:
            self.finalApproachSpeed = self.computeProportionalSpeed(self.tagToFinalApproachPt)

        # debugging info
        self.tStart = Timer.getFPGATimestamp()
        self.lastState = -1
        self.lastWarnings = None
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")


    def isFinished(self) -> bool:
        if self.finished:
            return True

        now = Timer.getFPGATimestamp()

        # bad ways to finish
        if self.lostTag:
            self.finished = self.lostTag
        elif now > self.lastSeenObjectTime + self.detectionTimeoutSeconds + self.finalApproachSeconds:
            delay = now - self.lastSeenObjectTime
            self.finished = f"not seen {int(1000 * delay)}ms"

        # good ways to finish
        elif self.tReachedFinalApproach != 0 and now > self.tReachedFinalApproach + self.finalApproachSeconds:
            self.finished = "approached" if self.finalApproachSeconds > 0 else "reached approach point"

        if not self.finished:
            return False

        return True


    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")
        else:
            elapsed = Timer.getFPGATimestamp() - self.tStart
            SmartDashboard.putString("command/c" + self.__class__.__name__, f"{int(1000 * elapsed)}ms: {self.finished}")


    def execute(self):
        now = Timer.getFPGATimestamp()

        # 0. look at the camera
        self.updateVision(now)
        visionOld = (now - self.lastSeenObjectTime) / (0.5 * self.frameTimeoutSeconds)
        if self.lostTag:
            self.drivetrain.stop()
            return

        # 1. how many degrees are left to turn? (and recommended rotation speed)
        rotationSpeed, degreesLeftToRotate = self.getGyroBasedRotationSpeed()

        # 2. how far from the glide path? (and recommended translation speed)
        fwdSpeed, leftSpeed, distanceToGlidePath = self.getVisionBasedSwerveSpeed(now)

        # 3. have we reached the glide path?
        if self.hasReachedGlidePath(degreesLeftToRotate, distanceToGlidePath):
            if self.tReachedGlidePath == 0:
                self.tReachedGlidePath = now

        # 4. be careful with forward speed
        warnings = None
        if self.tReachedFinalApproach:
            # - if we are on final approach, completely ignore the fwdSpeed from the visual estimation
            fwdSpeed = 0
            if self.finalApproachSeconds > 0:
                completedPercentage = (now - self.tReachedFinalApproach) / self.finalApproachSeconds
                fwdSpeed = self.finalApproachSpeed * max((0.0, 1.0 - completedPercentage))
            leftSpeed *= max(0.0, 1 - visionOld * visionOld)  # final approach: dial down the left speed if no object
        else:
            # - otherwise slow down if the visual estimate is old or if heading is not right yet
            farFromDesiredHeading = abs(degreesLeftToRotate) / self.DESIRED_HEADING_RADIUS.value
            if farFromDesiredHeading >= 1:
                warnings = "large heading error"
            if visionOld >= 1:
                warnings = "temporarily out of sight"
            # any other reason to slow down? put it above

            problems = max((visionOld, farFromDesiredHeading))
            fwdSpeed *= max((0.0, 1.0 - problems * problems))

        # 5. drive!
        if self.reverse:
            self.drivetrain.drive(-fwdSpeed, -leftSpeed, rotationSpeed, fieldRelative=False, rateLimit=False)
        else:
            self.drivetrain.drive(fwdSpeed, leftSpeed, rotationSpeed, fieldRelative=False, rateLimit=False)

        # 6. debug
        state = self.getState()
        if state != self.lastState or warnings != self.lastWarnings:
            SmartDashboard.putString("command/c" + self.__class__.__name__, warnings or self.STATE_NAMES[state])
        self.lastState = state
        self.lastWarnings = warnings


    def getGyroBasedRotationSpeed(self):
        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # (optimize: do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportionalSpeed = self.KPMULT_ROTATION.value * AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed

        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        turnSpeed = min([proportionalSpeed, 1.0])
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed

        return turnSpeed, degreesRemaining


    def getVisionBasedSwerveSpeed(self, now):
        direction = self.getVisionBasedSwerveDirection(now)
        if direction is None:
            return 0.0, 0.0, None

        # use proportional control to compute the velocity
        distance = direction.norm()
        velocity = self.computeProportionalSpeed(distance)

        # adjust the direction to account for APPROACH_SHAPE_POWER (= how strongly we prioritize getting to glide path)
        direction = Translation2d(direction.x, direction.y * self.APPROACH_SHAPE.value)
        norm = direction.norm()

        # distribute velocity between X and Y velocities in a way that gives us correct trajectory shape
        yVelocity = velocity * (direction.y / norm)
        xVelocity = velocity * (direction.x / norm)

        # do we need to rescale these velocities to meet constraints?
        norm = Translation2d(xVelocity, yVelocity).norm()
        if norm < GoToPointConstants.kMinTranslateSpeed:
            factor = GoToPointConstants.kMinTranslateSpeed / norm
            xVelocity *= factor
            yVelocity *= factor

        # done
        return xVelocity, yVelocity, abs(direction.y)


    def getVisionBasedSwerveDirection(self, now):
        # can we trust the last seen object?
        if not (self.lastSeenObjectSize > 0):
            return None  # the object is not yet there, hoping that this is temporary

        # where are we?
        robotX, robotY, tagX = self.localize()

        # have we reached the final approach point now? (must already be on glide path, otherwise it doesn't count)
        if self.tReachedGlidePath != 0 and self.tReachedFinalApproach == 0 and robotX > 0:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "reached final approach")
            self.tReachedFinalApproach = now
            print("final approach starting")

        # if we already reached the glide path, and we want nonzero final approach (after reaching desired size)
        # ... then go directly towards the tag (x, y = tagX, 0) instead of going towards 0, 0
        if self.tReachedGlidePath != 0 and self.finalApproachSeconds > 0:
            direction = Translation2d(x=tagX - robotX, y=0.0 - robotY)
        else:
            direction = Translation2d(x=0.0 - robotX, y=0.0 - robotY)  # otherwise go towards 0, 0
        if not (direction.x != 0 or direction.y != 0):
            SmartDashboard.putString("command/c" + self.__class__.__name__, "warning: distance not positive")
            return None

        return direction


    def localize(self):
        """
        localize the robot camera in the frame of the final approach point
        (i.e. final approach point is assumed to be at (x, y) = (0, 0), tag is assumed to be at (x, y) = (d, 0))
        :return: (x, y, d), where `x,y` are coordinates of the robot and `d` is distance between that point and tag
        """
        distanceToTag = self.lastSeenDistanceToTag

        # trigonometry: how many meters on the left is our tag? (if negative, then it's on the right)
        angle = Rotation2d.fromDegrees(self.lastSeenObjectX)
        y = -distanceToTag * angle.sin()

        distanceToFinalApproach = distanceToTag - self.tagToFinalApproachPt
        return -distanceToFinalApproach, -y, self.tagToFinalApproachPt


    def computeProportionalSpeed(self, distance) -> float:
        kpMultTran = self.KPMULT_TRANSLATION.value
        velocity = distance * GoToPointConstants.kPTranslate * kpMultTran
        if GoToPointConstants.kUseSqrtControl:
            velocity = math.sqrt(0.5 * velocity * kpMultTran)
        if velocity > self.approachSpeed:
            velocity = self.approachSpeed
        if velocity < GoToPointConstants.kMinTranslateSpeed:
            velocity = GoToPointConstants.kMinTranslateSpeed
        return velocity


    def computeTagDistanceFromTagSizeOnFrame(self, objectSizePercent):
        """
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        """
        return math.sqrt(0.2 * 0.2 / (1.70 * 0.01 * objectSizePercent))
        # note: Arducam w OV9281 (and Limelight 3 / 4) is 0.57 sq radians (not 1.33)


    def hasReachedGlidePath(self, degreesLeftToRotate: float, distanceToGlidePath: float) -> bool:
        reachedNow = (
            distanceToGlidePath is not None and
            abs(distanceToGlidePath) < self.GLIDE_PATH_WIDTH_INCHES.value * 0.0254 * 0.5 and
            abs(degreesLeftToRotate) < 4 * AimToDirectionConstants.kAngleToleranceDegrees
        )
        if self.tReachedGlidePath and not reachedNow:
            print(f"WARNING: not on glide path anymore (distance={distanceToGlidePath}, degrees={degreesLeftToRotate}")
        return reachedNow


    def updateVision(self, now):
        # non-sim logic:
        if self.camera.hasDetection():
            x = self.camera.getX()
            a = self.camera.getA()
            if x != 0 and a > 0:
                self.lastSeenDistanceToTag = self.computeTagDistanceFromTagSizeOnFrame(a)
                self.lastSeenObjectTime = now
                self.lastSeenObjectSize = a
                self.lastSeenObjectX = x
                if not self.everSawObject:
                    self.everSawObject = True

        timeSinceLastHeartbeat = self.camera.getSecondsSinceLastHeartbeat()
        if timeSinceLastHeartbeat > self.frameTimeoutSeconds:
            self.lostTag = f"no camera heartbeat > {int(1000 * timeSinceLastHeartbeat)}ms"

        if self.lastSeenObjectTime != 0:
            timeSinceLastDetection = now - self.lastSeenObjectTime
            if timeSinceLastDetection > self.detectionTimeoutSeconds:
                if self.tReachedFinalApproach == 0:
                    self.lostTag = f"object lost for {int(1000 * timeSinceLastDetection)}ms before final approach"
                elif timeSinceLastDetection > self.detectionTimeoutSeconds + self.finalApproachSeconds:
                    self.lostTag = f"object lost for {int(1000 * timeSinceLastDetection)}ms on final approach"


    STATE_NAMES = [
        "starting",
        "catching glide path",
        "on glide path",
        "final approach",
        "finished",
        "lost tag",
    ]

    def getState(self):
        if self.lostTag:
            return 5
        if self.finished:
            return 4
        if self.tReachedFinalApproach != 0:
            return 3
        if self.tReachedGlidePath != 0:
            return 2
        if self.lastSeenObjectX != 1:
            return 1
        # otherwise, just starting
        return 0

```

Previous code for slower command AlignWithTag was `commands/alignwithtag.py` :

```python

#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import math
import commands2

from commands.aimtodirection import AimToDirectionConstants, AimToDirection
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d
from wpilib import Timer, SmartDashboard

from constants import DriveConstants


class AlignWithTag(commands2.Command):
    TOLERANCE_METERS = 0.025  # one inch tolerance for alignment
    KP_MULT = 0.3

    def __init__(self,
                 camera,
                 drivetrain,
                 specificHeadingDegrees=None,
                 speed=0.2,
                 reverse=False,
                 pushForwardSeconds=0.0,
                 pushForwardSpeed=0.1,
                 detectionTimeoutSeconds=2.0,
                 cameraMinimumFps=4.0):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specificHeadingDegrees: do you want the robot to face in a very specific direction
        :param speed: if the camera is on the back of the robot, please use negative speed
        :param pushForwardSeconds: if you want the robot to push forward at the end of alignment
        :param reverse: set it =True if the camera is on the back of the robot (not front)
        :param detectionTimeoutSeconds: if no detection within this many seconds, assume the tag is lost
        :param cameraMinimumFps: what is the minimal number of **detected** frames per second expected from this camera
        """
        super().__init__()
        assert hasattr(camera, "getX"), "camera must have `getX()` to give us the object coordinate (in degrees)"
        assert hasattr(camera, "getA"), "camera must have `getA()` to give us object size (in % of screen)"
        assert hasattr(camera, "getSecondsSinceLastHeartbeat"), "camera must have a `getSecondsSinceLastHeartbeat()`"
        assert hasattr(drivetrain, "drive"), "drivetrain must have a `drive()` function, because we need a swerve drive"
        assert pushForwardSeconds >= 0, f"pushForwardSeconds={pushForwardSeconds}, but must be >=0"
        if pushForwardSeconds > 0:
            assert pushForwardSpeed > 0, f"if pushForwardSeconds={pushForwardSeconds} is not zero, pushForwardSpeed must be positive (not {pushForwardSpeed})"

        self.drivetrain = drivetrain
        self.camera = camera

        self.addRequirements(drivetrain)
        self.addRequirements(camera)

        self.reverse = reverse
        self.speed = min((1.0, abs(speed)))  # ensure that the speed is between 0.0 and 1.0
        self.pushForwardSeconds = pushForwardSeconds
        self.pushForwardSpeed = pushForwardSpeed

        assert detectionTimeoutSeconds > 0, f"non-positive detectionTimeoutSeconds={detectionTimeoutSeconds}"
        self.detectionTimeoutSeconds = detectionTimeoutSeconds

        assert cameraMinimumFps > 0, f"non-positive cameraMinimumFps={cameraMinimumFps}"
        self.frameTimeoutSeconds = 1.0 / cameraMinimumFps

        # setting the target angle in a way that works for all cases
        self.targetDegrees = specificHeadingDegrees
        if specificHeadingDegrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(specificHeadingDegrees):
            self.targetDegrees = lambda: specificHeadingDegrees

        # state
        self.targetDirection = None
        self.lastSeenObjectTime = None
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.everSawObject = False
        self.tAlignedToTag = 0.0  # time when aligned to the tag and desired direction for the first time
        self.lostTag = ""
        self.finished = ""


    def initialize(self):
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())
        self.tAlignedToTag = 0.0  # time when aligned to the tag and desired direction
        self.lostTag = False
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenObjectTime = Timer.getFPGATimestamp()
        self.everSawObject = False
        self.finished = ""
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")


    def isFinished(self) -> bool:
        if self.finished:
            return True

        now = Timer.getFPGATimestamp()

        # bad ways to finish
        if self.lostTag:
            self.finished = self.lostTag
        elif now > self.lastSeenObjectTime + self.detectionTimeoutSeconds + self.pushForwardSeconds:
            delay = now - self.lastSeenObjectTime
            self.finished = f"not seen {int(1000 * delay)}ms"

        # good ways to finish
        elif self.tAlignedToTag != 0 and now > self.tAlignedToTag + self.pushForwardSeconds:
            self.finished = "algnd+ppushed" if self.pushForwardSeconds > 0 else "aligned"

        if not self.finished:
            return False

        SmartDashboard.putString("command/c" + self.__class__.__name__, self.finished)
        return True


    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")


    def execute(self):
        now = Timer.getFPGATimestamp()
        if self.camera.hasDetection():
            x = self.camera.getX()
            a = self.camera.getA()
            if x != 0 and a > 0:
                self.lastSeenObjectTime = now
                self.lastSeenObjectSize = a
                self.lastSeenObjectX = x
                self.everSawObject = True

        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 2. use that to calculate the turn speed
        turnSpeed = self.getTurnSpeed(degreesRemaining)

        # 3. if the robot heading is almost aligned, start swerving right or left (for centering on that tag precisely)
        swerveSpeed = self.getSwerveSpeed(degreesRemaining)

        # 4. if we just aligned and no forward move is needed
        if self.tAlignedToTag != 0 and self.pushForwardSeconds == 0:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "no need to push")
            self.drivetrain.stop()
            return

        # 6. if aligned, push forward
        fwdSpeed = 0.0
        if self.tAlignedToTag != 0:
            fwdSpeed = (now - self.tAlignedToTag) * DriveConstants.kMagnitudeSlewRate
            if abs(fwdSpeed) > self.pushForwardSpeed:
                fwdSpeed = self.pushForwardSpeed
            if self.reverse:
                fwdSpeed = -fwdSpeed

        self.drivetrain.drive(fwdSpeed, swerveSpeed, turnSpeed, fieldRelative=False, rateLimit=False)


    def getTurnSpeed(self, degreesRemaining):
        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AlignWithTag.KP_MULT * AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed
        return turnSpeed


    def getSwerveSpeed(self, degreesRemaining):
        now = Timer.getFPGATimestamp()
        objectXDegrees = self.lastSeenObjectX
        objectSizePercent = self.lastSeenObjectSize

        timeSinceLastHeartbeat = self.camera.getSecondsSinceLastHeartbeat()
        if timeSinceLastHeartbeat > self.frameTimeoutSeconds:
            self.lostTag = f"no camera heartbeat > {int(1000 * timeSinceLastHeartbeat)}ms"
            return 0.0

        timeSinceLastDetection = now - self.lastSeenObjectTime
        if timeSinceLastDetection > self.detectionTimeoutSeconds:
            if self.tAlignedToTag == 0:
                self.lostTag = f"not aligned and no detection for {int(1000 * timeSinceLastDetection)}ms"
                return 0.0
            if timeSinceLastDetection > self.detectionTimeoutSeconds + self.pushForwardSeconds:
                self.lostTag = f"aligned but no detection for {int(1000 * timeSinceLastDetection)}ms"
                return 0.0

        if self.tAlignedToTag != 0 and timeSinceLastDetection > 0.5 * self.frameTimeoutSeconds:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "not fresh")
            return 0.0  # the last detection we know is not fresh, no need to use it

        if not (objectSizePercent > 0):
            SmartDashboard.putString("command/c" + self.__class__.__name__, "not yet acquired")
            return 0.0  # the object is not there, perhaps temporarily?

        swerveSpeed, objectXMeters = self.calculateSwerveLeftSpeed(objectSizePercent, objectXDegrees)
        if self.tAlignedToTag == 0 and abs(swerveSpeed) <= GoToPointConstants.kMinTranslateSpeed:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "90% aligned")
            print(f"AlignWithTag: almost done, since swerve speed {swerveSpeed} is already small")
            if abs(objectXMeters) <= AlignWithTag.TOLERANCE_METERS:
                SmartDashboard.putString("command/c" + self.__class__.__name__, "99% aligned")
                print(f"AlignWithTag: objectXMeters={objectXMeters} is small enough too")
                if abs(degreesRemaining) <= AimToDirectionConstants.kAngleToleranceDegrees:
                    SmartDashboard.putString("command/c" + self.__class__.__name__, "aligned")
                    print(f"AlignWithTag: degreesRemaining={degreesRemaining} is small, we are done aligning")
                    self.tAlignedToTag = Timer.getFPGATimestamp()

        return swerveSpeed


    def calculateSwerveLeftSpeed(self, objectSizePercent, objectXDegrees):
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        distanceMeters = math.sqrt(0.2 * 0.2 / (1.33 * 0.01 * objectSizePercent))

        # trigonometry: how many meters on the left is our object? (if negative, then it's on the right)
        objectXMeters = -distanceMeters * Rotation2d.fromDegrees(objectXDegrees).sin()
        # if the camera is on the back of the robot, then right and left are swapped
        if self.reverse:
            objectXMeters = -objectXMeters

        # how fast should we swerve to the right or left? use proportional control!
        swerveSpeed = self.speed
        proportionalSpeed = AlignWithTag.KP_MULT * GoToPointConstants.kPTranslate * abs(objectXMeters)
        if GoToPointConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)
        if proportionalSpeed < swerveSpeed:
            swerveSpeed = proportionalSpeed
        if swerveSpeed < GoToPointConstants.kMinTranslateSpeed:
            swerveSpeed = GoToPointConstants.kMinTranslateSpeed

        # if the object is on the right, swerve to the right (negative swerve speed)
        if objectXMeters < 0:
            swerveSpeed = -swerveSpeed

        return swerveSpeed, objectXMeters

```
</details>
