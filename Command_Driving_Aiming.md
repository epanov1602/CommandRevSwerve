# Autonomous Driving and Aiming Command Examples
(but you can also bind them to teleop buttons too: in `robotcontainer.py`, inside `configureButtonBindings` function)

## 1. Pre-requisites
Does your robot already have a drivetrain with odometry? (tank or swerve)
* Swerve drivetrain example: https://www.youtube.com/watch?v=44iMiQXQH5U
* Tank/Arcade drivetrain example: https://www.youtube.com/watch?v=DhYMjLz0ync

## 2. Three must-have commands, for all else to work (even on swerve bots)
**Must have!** These commands have important constants that may need to be adjusted to your robot.

Click on triangle signs below, in order to see the command code:

<details>
<summary>A command to set the (X, Y) position of robot on the field at the start of the game</summary>
Please try to put this code in file `commands/reset_xy.py`:

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
import commands2
import typing

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d


class AimToDirectionConstants:
    kP = 0.0058  # 0.002 is the default
    kMinTurnSpeed = 0.05  # turning slower than this is unproductive for the motor (might not even spin)
    kAngleToleranceDegrees = 2.0  # plus minus 2 degrees is "close enough"
    kAngleVelocityToleranceDegreesPerSec = 50  # velocity under 100 degrees/second is considered "stopped"


class AimToDirection(commands2.Command):
    def __init__(self, degrees: float | typing.Callable[[], float], drivetrain: DriveSubsystem, speed=1.0, fwd_speed=0.0):
        super().__init__()
        self.targetDegrees = degrees
        self.speed = min((1.0, abs(speed)))
        self.targetDirection = None
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.fwdSpeed = fwd_speed

    def initialize(self):
        if callable(self.targetDegrees):
            self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())
        else:
            self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees)

    def execute(self):
        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed:
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

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d, Translation2d

from commands.aimtodirection import AimToDirectionConstants


class GoToPointConstants:
    kPTranslate = 2.0
    kMinTranslateSpeed = 0.07  # moving forward slower than this is unproductive
    kApproachRadius = 0.1  # within this radius from target location, try to point in desired direction
    kOversteerAdjustment = 0.5


class GoToPoint(commands2.Command):
    def __init__(self, x, y, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True, finishDirection=None) -> None:
        """
        Go to a point with (X, Y) coordinates. Whether this is the end of your trajectory or not.
        :param x:
        :param y:
        :param drivetrain:
        :param speed: between -1.0 and +1.0 (you can use negative speed to drive backwards)
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

        # 2. if we are pointing in a very wrong direction (more than 45 degrees away), rotate away without moving
        if degreesRemaining > 45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, abs(self.speed))
            return
        elif degreesRemaining < -45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, -abs(self.speed))
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
        if rotateSpeed > proportionalRotateSpeed:
            rotateSpeed = proportionalRotateSpeed

        # 5. but if not too different, then we can drive while turning
        proportionalTransSpeed = GoToPointConstants.kPTranslate * distanceRemaining
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

from subsystems.drivesubsystem import DriveSubsystem
from commands.aimtodirection import AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d, Translation2d, Pose2d


class SwerveToPoint(commands2.Command):
    def __init__(self, x, y, headingDegrees, drivetrain: DriveSubsystem, speed=1.0, slowDownAtFinish=True) -> None:
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

    def execute(self):
        currentXY = self.drivetrain.getPose()
        xDistance, yDistance = self.targetPose.x - currentXY.x, self.targetPose.y - currentXY.y
        totalDistance = self.targetPose.translation().distance(currentXY.translation())

        totalSpeed = GoToPointConstants.kPTranslate * totalDistance
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
        if turningSpeed > abs(self.speed):
            turningSpeed = abs(self.speed)
        if turningSpeed < AimToDirectionConstants.kMinTurnSpeed:
            turningSpeed = AimToDirectionConstants.kMinTurnSpeed
        if degreesLeftToTurn < 0:
            turningSpeed = -turningSpeed

        self.drivetrain.drive(xSpeed, ySpeed, turningSpeed, fieldRelative=True, rateLimit=False)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        currentPose = self.drivetrain.getPose()
        currentPosition = currentPose.translation()

        # did we overshoot?
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)
        if not self.stop and distanceFromInitialPosition > self.initialDistance - 3 * GoToPointConstants.kApproachRadius:
            return True  # close enough

        if distanceFromInitialPosition > self.initialDistance:
            self.overshot = True

        if self.overshot:
            distanceFromTargetDirectionDegrees = self.getDegreesLeftToTurn()
            if abs(distanceFromTargetDirectionDegrees) < 3 * AimToDirectionConstants.kAngleToleranceDegrees:
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

```
</details>

## 4. Curved trajectory to some point (simplistic)

<details>
<summary>Trajectory to some endpoint via "waypoints" (skips waypoints that are already behind)</summary>
Please try to put this code in file `commands/jerky_trajectory.py`:

```python
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
        :param swerve: do we want to use the swerve functionality (otherwise it will arcade drive)
        :param drivetrain: drive train
        :param endpoint: end point of the trajectory
        :param waypoints: a list of waypoints, if any (can be (X,Y) tuples, Translation2d or Rotation2d)
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

    def _makeWaypoint(self, waypoint):
        point, heading = None, None
        if isinstance(waypoint, (tuple, list)) and len(waypoint) == 2:
            point, heading = waypoint[0], waypoint[1]
            if not isinstance(point, Translation2d):
                point = Translation2d(waypoint[0], waypoint[1])
                heading = None
        elif isinstance(waypoint, (tuple, list)) and len(waypoint) == 3:
            heading = waypoint[2]
            if isinstance(heading, (float, int)):
                heading = Rotation2d(heading)
            point = Translation2d(waypoint[0], waypoint[1])
        elif isinstance(waypoint, Translation2d):
            point = waypoint
        elif isinstance(waypoint, Pose2d):
            point = waypoint.translation()
            heading = waypoint.rotation()
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

    def initialize(self):
        # find the waypoint nearest to the current location: we want to skip all the waypoints before it
        nearest, distance = None, None
        location = self.drivetrain.getPose()
        for point, heading in self.waypoints:
            d = location.translation().distance(point)
            if nearest is None or d < distance:
                nearest, distance = point, d

        # only use the waypoints that we must not skip (past the nearest waypoint)
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
        last_direction = None

        # make the commands connecting the waypoints which remain after skipping
        commands = []
        for index, (point, heading) in enumerate(waypoints):
            command = self._makeWaypointCommand(point, heading, index == last)
            last_direction = heading
            commands.append(command)

        # if the last waypoint (endpoint) has a specific direction, aim in that direction at the end
        commands.append(AimToDirection(last_direction.degrees(), speed=self.speed, drivetrain=self.drivetrain))

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

```
</details>
