import math

import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d

from commands.gotopoint import GoToPoint, GoToPointConstants
from subsystems.firing_table import FiringTable
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.turret import Turret


class Constants:
    RPM_TOLERANCE_FACTOR = 0.03  # plus minus 3%
    ANGLE_TOLERANCE_DEGREES = 5  # plus minus 5 degrees is fine for drivetrain aiming
    RANGE_TOLERANCE_METERS = 0.1


class GetInRange(commands2.Command):
    """
    Simple example:
    ```
    getInRange = GetInRange(
        goal=self.firingTable,
        drivetrain=self.robotDrive
    )

    self.driverController.buttons(XboxController.Button.kA).whileTrue(getInRange)
    ```

    Longer example (get in range and rev up the shooter at the same time):
    ```
    getInRange = GetInRange(
        goal=self.firingTable,
        drivetrain=self.robotDrive
    )
    getReady = GetReadyToShoot(
        goal=self.firingTable,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None  # if we have a turret, drivetrain=None (otherwise supply drivetrain=self.robotDrive)
    )

    # get in range and rev up the shooter at the same time (and point the turret if you have)
    self.driverController.buttons(XboxController.Button.kA).whileTrue(
       ParallelCommandGroup(getInRange, getReady)
    )

    keepShooting = GetReadyAndKeepShooting(
        goal=self.firingTable,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None  # if we have a turret, drivetrain=None (otherwise supply drivetrain=self.robotDrive)
        indexer=self.indexer
    )
    self.driverController.buttons(XboxController.Button.kB).whileTrue(
       keepShooting
    )

    """
    def __init__(
        self,
        goal: FiringTable,
        drivetrain: DriveSubsystem
    ):
        super().__init__()
        self.goal = goal
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        pass

    def end(self, interrupted: bool):
        self.drivetrain.stop()

    def execute(self):
        direction = self.goal.vectorToGoal
        if direction is None or direction.squaredNorm() == 0:
            self.drivetrain.stop()
            return

        # calculate the velocity needed to get in range (slow down when pretty close)
        velocity, distance = 0.0, direction.norm()
        if distance > self.goal.maximumRangeMeters:
            velocity = (distance - self.goal.maximumRangeMeters) * GoToPointConstants.kPTranslate
        elif distance < self.goal.minimumRangeMeters:
            velocity = -(self.goal.maximumRangeMeters - distance) * GoToPointConstants.kPTranslate
        if GoToPointConstants.kUseSqrtControl:
            velocity = math.sqrt(0.5 * velocity)

        # calculate the correct direction for that velocity and drive the robot that way
        velocityInRobotAxes = Translation2d(velocity, 0.0).rotateBy(
            direction.angle() - self.drivetrain.getHeading())
        self.drivetrain.drive(
            velocityInRobotAxes.x, velocityInRobotAxes.y, 0.0, fieldRelative=False, rateLimit=False)

    def isFinished(self):
        if self.goal.maximumRangeMeters == 0:
            return True  # maximum range does not exist, we are done
        tol = Constants.RANGE_TOLERANCE_METERS

        distance = self.goal.distance()
        return self.goal.minimumRangeMeters - tol < distance < self.goal.maximumRangeMeters + tol


class GetReadyToShoot(commands2.Command):
    """
    Uses the firing table and sets the correct RPM+angle in the shooter,
     aims the turret at the target (if turret is given) or aims the drivetrain there instead.

    Usage example ( put it into configureButtonBindings(...) ):
    ```
    keepAiming = GetReadyToShoot(
        goal=self.firingTable,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None  # drivetrain=None if we already have a turret
    )

    self.driverController.buttons(XboxController.Button.kX).whileTrue(keepAiming)
    ```
    """
    def __init__(
        self,
        goal: FiringTable,
        shooter: Shooter,
        turret: Turret | None,
        drivetrain: DriveSubsystem | None,
        runForever = True,
    ):
        super().__init__()
        self.runForever = runForever
        self.goal = goal
        self.shooter = shooter
        self.turret = turret
        self.drivetrain = drivetrain
        if self.turret is not None:
            assert self.drivetrain is None, "do not supply drivetrain for aiming if we already have a turret"
        self.addRequirements(shooter)
        # DO NOT ADDREQUIREMENTS(drivetrain) or (goal), let the drivers (or another command) drive instead
        if turret is not None:
            self.addRequirements(turret)

        self.drivetrainTarget = None
        self.notReady = "?"

    def execute(self):
        # set the correct RPM (and hood servo position) in the shooter
        rpm = self.goal.recommendedShooterRpm()
        hoodPosition = self.goal.recommendedFiringHoodPosition()
        self.shooter.setVelocityGoal(rpm, rpm * Constants.RPM_TOLERANCE_FACTOR)
        self.shooter.setHoodServoGoal(hoodPosition)

        # aim the turret if we have it
        if self.turret is not None:
            direction = self.goal.recommendedTurretDirection()
            if direction is not None:
                self.turret.setPositionGoal(direction.degrees())

        # if we are not aiming with a turret, we can aim with drivetrain
        elif self.drivetrainTarget is not None:
            goalPoint = self.goal.goal
            if goalPoint is not None and self.drivetrain.startOverrideToFaceThisPoint(goalPoint):
                self.drivetrainTarget = goalPoint

        # check if we are ready to fire or not
        notYet = self.turretNotReady() or self.drivetrainNotReady() or self.shooter.notReady() or self.distanceNotGood()
        self.setNotReady(notYet)

    def end(self, interrupted: bool):
        self.setNotReady("finished")
        self.shooter.stop()
        if self.turret is not None:
            self.turret.stopAndReset()
        if self.drivetrainTarget is not None:
            self.drivetrain.stopOverrideToFaceThisPoint(self.drivetrainTarget)

    def initialize(self):
        self.drivetrainTarget = None
        self.setNotReady("started")

    def isFinished(self):
        if self.notReady or self.runForever:
            return False
        return True

    def setNotReady(self, notReady):
        if notReady != self.notReady:
            SmartDashboard.putString("GetReadyToShoot", notReady)
            self.notReady = notReady

    def turretNotReady(self):
        if self.turret is None:
            return ""
        return self.turret.notReady()

    def drivetrainNotReady(self):
        if self.drivetrain is not None:
            # direction not right?
            notPointing = self.drivetrain.notPointingTo(self.drivetrainTarget, Constants.ANGLE_TOLERANCE_DEGREES)
            if notPointing:
                return notPointing
        # otherwise no problem
        return ""

    def distanceNotGood(self):
        if self.goal.maximumRangeMeters != 0:
            distance = self.goal.distance()
            if distance > self.goal.maximumRangeMeters:
                return "too far"
            if distance < self.goal.minimumRangeMeters:
                return "too close"
        return ""


class GetReadyAndKeepShooting(GetReadyToShoot):
    """
    Usage example ( put it into configureButtonBindings(...) ):
    ```
    shootWhenReady = GetReadyAndKeepShooting(
        goal=self.firingTable,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
        indexer=self.indexer,
        indexerSpeed=0.4
    )

    self.driverController.buttons(XboxController.Button.kY).whileTrue(shootWhenReady)
    ```
    """

    def __init__(
        self,
        goal: FiringTable,
        shooter: Shooter,
        turret: Turret | None,
        drivetrain: DriveSubsystem | None,
        indexer: Indexer,
        indexerSpeed: float = 0.5
    ):
        super().__init__(goal, shooter, turret, drivetrain)
        self.indexer = indexer
        self.indexerSpeed = indexerSpeed
        self.addRequirements(indexer)

    def initialize(self):
        super().initialize()

    def execute(self):
        # let the GetReadyToShoot() command execute
        super().execute()

        # if everything is ready, feed gamepieces into the shooter
        if self.notReady:
            self.indexer.stop()
        else:
            self.indexer.feedGamepieceIntoShooter(self.indexerSpeed)

    def end(self, interrupted: bool):
        super().end(interrupted=interrupted)
        self.indexer.stop()

    def isFinished(self):
        return False
