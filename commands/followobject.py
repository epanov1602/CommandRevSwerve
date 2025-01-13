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
