# Autonomous Driving and Aiming Command Examples
(but you can also bind them to teleop buttons too: in `robotcontainer.py`, inside `configureButtonBindings` function)

## 1. Pre-requisites
Does your robot already have a drivetrain with odometry? (tank or swerve)

You can look at these templates below to add it... or simply fork your copy of one of these templates, right here on GitHub (whey are public, with WPI license).

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
        turningSpeed = 0.5 * abs(degreesLeftToTurn) * AimToDirectionConstants.kP
        if AimToDirectionConstants.kUseSqrtControl:
            turningSpeed = math.sqrt(0.5 * turningSpeed)  # will match the non-sqrt value when 50% max speed
        if turningSpeed > abs(self.speed):
            turningSpeed = abs(self.speed)
        if degreesLeftToTurn < 0:
            turningSpeed = -turningSpeed

        # now rotate xSpeed and ySpeed into robot coordinates
        speed = Translation2d(x=xSpeed, y=ySpeed).rotateBy(-self.drivetrain.getHeading())

        self.drivetrain.drive(speed.x, speed.y, turningSpeed, fieldRelative=False, rateLimit=False)

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


class SwerveToSide(commands2.Command):
    def __init__(self, metersToTheLeft: float, drivetrain: DriveSubsystem, speed=1.0) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.speed = speed
        self.metersToTheLeft = metersToTheLeft
        self.subcommand = None

    def initialize(self):
        position = self.drivetrain.getPose()
        heading = position.rotation()
        target = position.translation() + Translation2d(x=0, y=self.metersToTheLeft).rotateBy(heading)
        self.subcommand = SwerveToPoint(
            x=target.x, y=target.y, headingDegrees=heading.degrees(), drivetrain=self.drivetrain, speed=self.speed
        )
        self.subcommand.initialize()

    def isFinished(self) -> bool:
        return self.subcommand.isFinished()

    def execute(self):
        return self.subcommand.execute()

    def end(self, interrupted: bool):
        self.subcommand.end(interrupted)

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

from commands.aimtodirection import AimToDirection, AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d
from wpilib import Timer


class SetCameraPipeline(commands2.Command):

    def __init__(self, camera, pipelineIndex: int):
        super().__init__()
        self.pipelineIndex = pipelineIndex
        self.camera = camera
        self.addRequirements(camera)

    def initialize(self):
        self.camera.setPipeline(self.pipelineIndex)

    def isFinished(self) -> bool:
        # we are finished when the camera has responded that pipeline index is now set
        if self.camera.getPipeline() == self.pipelineIndex:
            return True
        # otherwise, print that we aren't finished
        print("SetCameraPipeline: not yet finished, because camera pipeline = {} and we want {}".format(
            self.camera.getPipeline(), self.pipelineIndex)
        )

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
    <summary>Turning until you find an object using camera</summary>

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

```python
        from commands.setcamerapipeline import SetCameraPipeline
        from commands.followobject import FollowObject, StopWhen
        from commands.alignwithtag import AlignWithTag
        from commands.swervetopoint import SwerveToSide

        # switch to camera pipeline 3, to start looking for certain kind of AprilTags
        lookForTheseTags = SetCameraPipeline(self.camera, 3)
        approachTheTag = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=4), speed=0.5)
        alignAndPush = AlignWithTag(self.camera, self.robotDrive, 0, speed=0.2, pushForwardSeconds=1.0)
        dropCoralOnLevel2 = ... # your robot has a command for this, right?

        # connect them together
        scoreCoral = lookForTheseTags.andThen(approachTheTag).andThen(alignAndPush).andThen(dropCoralOnLevel2)

        # or you can do this, if you want to score the coral 15 centimeters to the right of above the AprilTag
        # stepToSide = SwerveToSide(drivetrain=self.robotDrive, metersToTheLeft=-0.15, speed=0.2)
        # scoreCoral = lookForTheseTags.andThen(approachTheTag).andThen(alignAndPush).andThen(stepToSide).andThen(dropCoralOnLevel2)

```

This code works with Limelight or PhotonVision cameras from [here](Adding_Camera.md) (step-by-step video https://www.youtube.com/watch?v=8b9DZQ8CyII).
The code for the command AlignWithTag should go to `commands/alignwithtag.py` .
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
from wpilib import Timer

from subsystems.limelight_camera import LimelightCamera


class AlignWithTag(commands2.Command):
    FINISH_ALIGNMENT_EXTRA_SECONDS = 0.5  # at least 0.5 seconds to finish the alignment
    TOLERANCE_METERS = 0.025  # one inch tolerance for alignment

    def __init__(self,
                 camera,
                 drivetrain,
                 specificHeadingDegrees=None,
                 speed=0.2,
                 reverse=False,
                 pushForwardSeconds=0.0,
                 pushForwardSpeed=0.1):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specificHeadingDegrees: do you want the robot to face in a very specific direction
        :param speed: if the camera is on the back of the robot, please use negative speed
        :param pushForwardSeconds: if you want the robot to push forward at the end of alignment
        :param reverse: set it =True if the camera is on the back of the robot (not front)
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

        # setting the target angle in a way that works for all cases
        self.targetDegrees = specificHeadingDegrees
        if specificHeadingDegrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(specificHeadingDegrees):
            self.targetDegrees = lambda: specificHeadingDegrees

        # state
        self.targetDirection = None
        self.alignedToTag = False
        self.pushForwardCommand = None
        self.haveNotSeenObjectSinceTime = None
        self.everSawObject = False


    def initialize(self):
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())
        self.alignedToTag = False
        self.pushForwardCommand = None
        self.haveNotSeenObjectSinceTime = Timer.getFPGATimestamp()
        self.everSawObject = False


    def isFinished(self) -> bool:
        now = Timer.getFPGATimestamp()
        if now > self.haveNotSeenObjectSinceTime + 3.0:
            print("AlignSwerveWithTag: finished because have not seen the object for >3 seconds")
            return True
        if self.alignedToTag and self.pushForwardCommand is None:
            print("AlignSwerveWithTag: finished because aligned to the tag and don't need to push forward")
            return True
        if self.alignedToTag and self.pushForwardCommand is not None and self.pushForwardCommand.isFinished():
            print("AlignSwerveWithTag: finished because aligned to the tag and the forward push is finished")
            return True


    def end(self, interrupted: bool):
        if self.pushForwardCommand is not None:
            self.pushForwardCommand.end(interrupted)
        self.drivetrain.arcadeDrive(0, 0)


    def execute(self):
        if self.camera.hasDetection():
            self.haveNotSeenObjectSinceTime = Timer.getFPGATimestamp()
            self.everSawObject = True

        # 0. are we pushing forward already?
        if self.alignedToTag and self.pushForwardCommand is not None:
            self.pushForwardCommand.execute()
            return

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
        #swerveSpeed = 0
        #if abs(degreesRemaining) < 4 * AimToDirectionConstants.kAngleToleranceDegrees or abs(turnSpeed) < AimToDirectionConstants.kMinTurnSpeed:
        swerveSpeed = self.getSwerveLeftSpeed(degreesRemaining)

        # 4. if we just aligned the heading and the swerve axis and should be pushing forward, make that push
        if not self.alignedToTag:
            self.drivetrain.drive(0, swerveSpeed, turnSpeed, fieldRelative=False, rateLimit=False)
        elif self.pushForwardCommand is None and self.pushForwardSeconds > 0:
            self.pushForwardCommand = self.getPushForwardCommand()
            self.pushForwardCommand.initialize()


    def getTurnSpeed(self, degreesRemaining):
        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed
        return turnSpeed


    def getPushForwardCommand(self):
        speed = self.pushForwardSpeed if not self.reverse else -self.pushForwardSpeed
        command = AimToDirection(degrees=None, drivetrain=self.drivetrain, fwd_speed=speed)
        return command.withTimeout(self.pushForwardSeconds)


    def getSwerveLeftSpeed(self, degreesRemaining):
        swerveSpeed = 0.0
        secondsSinceHeartbeat = self.camera.getSecondsSinceLastHeartbeat()
        objectSizePercent = self.camera.getA()
        objectXDegrees = self.camera.getX()
        if secondsSinceHeartbeat > 0.25:
            print(f"AlignSwerveWithTag: camera not usable (dead or too few frames per second), we see {secondsSinceHeartbeat} seconds since last hearbeat")
        elif objectXDegrees == 0 or objectSizePercent <= 0:
            print(f"AlignSwerveWithTag: invalid camera detection (objectX, objectSize) = ({objectXDegrees}, {objectSizePercent})")
            if not self.alignedToTag and self.everSawObject:
                now = Timer.getFPGATimestamp()
                if now > self.haveNotSeenObjectSinceTime + 1.5:
                    print(f"AlignSwerveWithTag: blind {now - self.haveNotSeenObjectSinceTime} seconds, too close??")
                    self.alignedToTag = True
        else:
            swerveSpeed, objectXMeters = self.calculateSwerveLeftSpeed(objectSizePercent, objectXDegrees)
            if not self.alignedToTag and abs(swerveSpeed) <= GoToPointConstants.kMinTranslateSpeed:
                print(f"AlignSwerveWithTag: almost done, since swerve speed {swerveSpeed} is already small")
                if abs(objectXMeters) <= AlignWithTag.TOLERANCE_METERS:
                    print(f"AlignSwerveWithTag: objectXMeters={objectXMeters} is small enough too")
                    if abs(degreesRemaining) <= AimToDirectionConstants.kAngleToleranceDegrees:
                        print(f"AlignSwerveWithTag: degreesRemaining={degreesRemaining} is small, we are done aligning")
                        self.alignedToTag = True
        return swerveSpeed


    def calculateSwerveLeftSpeed(self, objectSizePercent, objectXDegrees):
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        
        # further calibration revealed that the answer is 1.70
        distanceMeters = math.sqrt(0.2 * 0.2 / (1.70 * 0.01 * objectSizePercent))

        # trigonometry: how many meters on the left is our object? (if negative, then it's on the right)
        objectXMeters = -distanceMeters * Rotation2d.fromDegrees(objectXDegrees).tan()
        # if the camera is on the back of the robot, then right and left are swapped
        if self.reverse:
            objectXMeters = -objectXMeters

        # how fast should we swerve to the right or left? use proportional control!
        swerveSpeed = self.speed
        proportionalSpeed = 0.33 * GoToPointConstants.kPTranslate * abs(objectXMeters)
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
