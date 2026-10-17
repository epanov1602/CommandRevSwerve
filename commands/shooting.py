import math
from typing import List

import commands2
from wpilib import SmartDashboard, Timer, SendableChooser
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from commands.gotopoint import GoToPointConstants
from subsystems.firing_table import FiringTable
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.indexer import Indexer, IndexerConstants
from subsystems.shooter import Shooter
from subsystems.turret import Turret


class Constants:
    RPM_TOLERANCE_FACTOR = 0.12  # plus minus 10%
    TARGET_RADIUS_METERS = 0.34  # in reality the target is 0.52m wide, but let's be conservative
    RANGE_TOLERANCE_METERS = 0.1


class GetInRange(commands2.Command):
    """
    Simple example:
    ```
    getInRange = GetInRange(
        firingTable=self.firingTable,
        drivetrain=self.robotDrive
    )

    self.driverController.buttons(XboxController.Button.kA).whileTrue(getInRange)
    ```

    Longer example (get in range and rev up the shooter at the same time):
    ```
    getInRange = GetInRange(
        firingTable=self.firingTable,
        drivetrain=self.robotDrive
    )
    getReady = GetReadyToShoot(
        firingTable=self.firingTable,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None  # if we have a turret, drivetrain=None (otherwise supply drivetrain=self.robotDrive)
    )

    # get in range and rev up the shooter at the same time (and point the turret if you have)
    self.driverController.buttons(XboxController.Button.kA).whileTrue(
       ParallelCommandGroup(getInRange, getReady)
    )

    keepShooting = GetReadyAndKeepShooting(
        firingTable=self.firingTable,
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
            velocity = 2 * math.copysign(math.sqrt(0.5 * abs(velocity)), velocity)

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
        firingTable=self.firingTable,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None  # drivetrain=None if we already have a turret
    )

    self.driverController.buttons(XboxController.Button.kX).whileTrue(keepAiming)
    ```
    """
    def __init__(
        self,
        firingTable: FiringTable,
        shooter: Shooter,
        turret: Turret | None,
        drivetrain: DriveSubsystem | None,
        rpmFactor: float = 1.0,
        runForever = True,
    ):
        super().__init__()
        self.runForever = runForever
        self.firingTable = firingTable
        self.shooter = shooter
        self.turret = turret
        self.drivetrain = drivetrain
        self.rpmFactor = rpmFactor
        self.readyAfterTime = 0.0
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
        ft = self.firingTable
        distance = ft.distance()
        hoodPosition = ft.recommendedFiringHoodPosition()

        # scale the RPM value up or down
        rpm = ft.recommendedShooterRpm()
        rpm *= ft.factor.getSelected()
        rpm *= self.rpmFactor

        self.shooter.setVelocityGoal(rpm, rpm * Constants.RPM_TOLERANCE_FACTOR)
        self.shooter.setHoodServoGoal(hoodPosition)

        if ft.drivetrain.field is not None and ft.shooterLocation is not None:
            ft.drivetrain.field.getObject("firingDirn").setPoses(_drawArrow(ft.shooterLocation, ft.vectorToGoal))

        # aim the turret if we have it
        if self.turret is not None:
            direction = self.firingTable.recommendedTurretDirection()
            if direction is not None:
                self.turret.setAngleGoal(direction.degrees())

        # if we are not aiming with a turret, we can aim with drivetrain
        elif self.drivetrain is not None:
            goalPoint = self.firingTable.goal
            if goalPoint is not None and self.drivetrain.startOverrideToFaceThisPoint(goalPoint):
                self.drivetrainTarget = goalPoint

        # check if we are ready to fire or not
        notYet = self.turretNotReady() or self.drivetrainNotReady(distance) \
                 or self.shooter.notReady() or self.distanceNotGood(distance)

        now = Timer.getFPGATimestamp()
        if not notYet and self.readyAfterTime == 0.0:
            self.readyAfterTime = now + 0.25  # just wait for another 0.25s
        if now < self.readyAfterTime:
            notYet = "almost ready"

        self.setNotReady(notYet)

    def end(self, interrupted: bool):
        self.setNotReady("finished")
        self.shooter.stop()
        if self.turret is not None:
            self.turret.stopAndReset()
        if self.drivetrainTarget is not None:
            self.drivetrain.stopOverrideToFaceThisPoint(self.drivetrainTarget)
        self.firingTable.drivetrain.field.getObject("firingDirn").setPoses([])
        self.firingTable.resetSmartDashboard()

    def initialize(self):
        self.readyAfterTime = 0.0
        self.drivetrainTarget = None
        self.setNotReady("started")

    def isFinished(self):
        if self.notReady or self.runForever:
            return False
        return True

    def setNotReady(self, notReady):
        if notReady != self.notReady:
            SmartDashboard.putString("WhyNotShooting", notReady)
            self.notReady = notReady

    def turretNotReady(self):
        if self.turret is None:
            return ""
        return self.turret.notReady()

    def drivetrainNotReady(self, distanceMeters):
        if self.drivetrain is not None:
            # direction not right?
            angularToleranceRadians = Constants.TARGET_RADIUS_METERS / distanceMeters
            angularToleranceDegrees = angularToleranceRadians * 57.2958
            notPointing = self.drivetrain.notPointingTo(self.drivetrainTarget, angularToleranceDegrees)
            if notPointing:
                return notPointing
        # otherwise no problem
        return ""

    def distanceNotGood(self, distance):
        if self.firingTable.maximumRangeMeters != 0:
            if distance > self.firingTable.maximumRangeMeters:
                return f"too far, max range {self.firingTable.maximumRangeMeters} meters"
            if distance < self.firingTable.minimumRangeMeters:
                return f"too close, min range {self.firingTable.minimumRangeMeters} meters"
        return ""


class GetReadyAndKeepShooting(GetReadyToShoot):
    """
    Usage example ( put it into configureButtonBindings(...) ):
    ```
    shootWhenReady = GetReadyAndKeepShooting(
        firingTable=self.firingTable,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
        indexer=self.indexer,
    )

    self.driverController.buttons(XboxController.Button.kY).whileTrue(shootWhenReady)
    ```
    """

    def __init__(
        self,
        firingTable: FiringTable,
        shooter: Shooter,
        turret: Turret | None,
        drivetrain: DriveSubsystem | None,
        indexer: Indexer,
        rpmFactor: float | None = 1.0,
    ):
        super().__init__(firingTable, shooter, turret, drivetrain, rpmFactor)
        self.indexer = indexer
        self.addRequirements(indexer)

    def initialize(self):
        super().initialize()

    def execute(self):
        # let the GetReadyToShoot() command execute
        super().execute()

        # if everything is ready, feed gamepieces into the shooter
        if self.notReady:
            self.indexer.startUnjamming()
        else:
            self.indexer.feedGamepieceIntoShooter()

    def end(self, interrupted: bool):
        super().end(interrupted=interrupted)
        self.indexer.stop()

    def isFinished(self):
        return False


def _drawArrow(start: Translation2d, directionVector: Translation2d, nPoints=11, size=0.85, tip=0.1) -> List[Pose2d]:
    result = []
    length = directionVector.norm()
    zero = Rotation2d(0)
    if length > 0:
        end = start
        directionVector = directionVector / length
        for i in range(nPoints):
            end = start + directionVector * (size * i / nPoints)
            result.append(Pose2d(end, zero))
        ray1 = directionVector.rotateBy(Rotation2d.fromDegrees(90)) * tip
        ray2 = directionVector * tip
        ray3 = directionVector.rotateBy(Rotation2d.fromDegrees(-90)) * tip
        result.append(Pose2d(end + ray1, zero))
        result.append(Pose2d(end + ray2, zero))
        result.append(Pose2d(end + ray3, zero))
        result.append(Pose2d(end, zero))
    return result


class KeepHoodDown(commands2.Command):
    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.stop()
        self.shooter.setHoodServoGoal(0)

    def isFinished(self) -> bool:
        return False


class KeepFeederClear(commands2.Command):
    """
    Clears jams in the feeder by slowly rotating it backwards
    """
    def __init__(self, indexer: Indexer):
        super().__init__()
        self.indexer = indexer
        self.addRequirements(indexer)

    def initialize(self):
        if IndexerConstants.kAntiJamEnabled:
            self.indexer.setFeederVelocityGoal(-0.075 * IndexerConstants.kTargetFeederVelocity)
        else:
            self.indexer.stop()

    def end(self, interrupted: bool):
        self.indexer.stop()

    def isFinished(self) -> bool:
        return False


class UnjamFeeder(commands2.Command):
    """
    Clears jams in the feeder by slowly rotating it backwards
    """
    def __init__(self, indexer: Indexer):
        super().__init__()
        self.indexer = indexer
        self.addRequirements(indexer)

    def initialize(self):
        self.indexer.startUnjamming()

    def end(self, interrupted: bool):
        self.indexer.stop()

    def isFinished(self) -> bool:
        return False


class ShootFromFixedPosition(commands2.Command):
    rpm: SendableChooser | None = None

    def __init__(self, turret: Turret, shooter: Shooter, indexer: Indexer, shooterRpm: float | None = None):
        super().__init__()
        self.turret = turret
        self.shooter = shooter
        self.indexer = indexer
        self.shooterRpm = shooterRpm
        self.addRequirements(indexer)
        self.addRequirements(shooter)
        self.addRequirements(turret)
        self.tStart = 0.0
        SmartDashboard.putString("ShootFromFixedPos", "created")

        ShootFromFixedPosition.rpm = SendableChooser()
        for rpm in range(2000, 2500, 50):
            ShootFromFixedPosition.rpm.addOption(str(rpm), rpm)
        ShootFromFixedPosition.rpm.setDefaultOption("2350", 2350)
        SmartDashboard.putData("ShootFromFixedPos/rpmChosen", ShootFromFixedPosition.rpm)

    def initialize(self):
        rpm = self.shooterRpm
        if rpm is None:
            rpm = ShootFromFixedPosition.rpm.getSelected()
        if rpm is None:
            rpm = 2350
        SmartDashboard.putString("ShootFromFixedPos", f"started @ {rpm}")
        self.turret.setAngleGoal(90)
        self.shooter.setHoodServoGoal(0.0)
        self.shooter.setVelocityGoal(rpm, rpm * 0.1)
        self.tStart = Timer.getFPGATimestamp()

    def execute(self):
        t = Timer.getFPGATimestamp()
        if t > self.tStart + 0.05:
            self.indexer.feedGamepieceIntoShooter()
            SmartDashboard.putString("ShootFromFixedPos", f"shooting")
        else:
            self.indexer.startUnjamming()
            SmartDashboard.putString("ShootFromFixedPos", "unjamming")


    def end(self, interrupted: bool):
        SmartDashboard.putString("ShootFromFixedPos", "finished")
        self.indexer.stop()
        self.shooter.stop()
