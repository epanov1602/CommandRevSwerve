import commands2
from wpilib import SmartDashboard

from subsystems.distance_to_goal import FiringTable
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.turret import Turret


class Constants:
    RPM_TOLERANCE_FACTOR = 0.03  # plus minus 3%
    ANGLE_TOLERANCE_DEGREES = 5  # plus minus 5 degrees is fine for drivetrain aiming


class KeepAiming(commands2.Command):
    """
    Usage example ( put it into configureButtonBindings(...) ):
    ```
    keepAiming = KeepAiming(
        goal=self.distanceToGoal,
        shooter=self.shooter,
        turret=self.turret,
        drivetrain=None  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
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
    ):
        super().__init__()
        self.goal = goal
        self.shooter = shooter
        self.turret = turret
        self.drivetrain = drivetrain
        if self.turret is not None:
            assert self.drivetrain is None, "do not supply drivetrain for aiming if we already have a turret"
        self.addRequirements(goal)
        self.addRequirements(shooter)
        # DO NOT ADDREQUIREMENTS(drivetrain), on purpose
        if turret is not None:
            self.addRequirements(turret)
        self.drivetrainTarget = None

    def execute(self):
        rpm = self.goal.recommendedShooterRpm()
        self.shooter.setVelocityGoal(rpm, rpm * Constants.RPM_TOLERANCE_FACTOR)

        # aim with the turret
        if self.turret is not None:
            direction = self.goal.recommendedTurretDirection()
            if direction is not None:
                self.turret.setPositionGoal(direction.degrees())

        # if we are not aiming with a turret, we can aim with drivetrain
        if self.drivetrain is not None and self.drivetrainTarget is None:
            goalPoint = self.goal.goal
            if goalPoint is not None and self.drivetrain.startOverrideToFaceThisPoint(goalPoint):
                self.drivetrainTarget = goalPoint

    def end(self, interrupted: bool):
        self.shooter.stop()
        if self.turret is not None:
            self.turret.stopAndReset()
        if self.drivetrainTarget is not None:
            self.drivetrain.stopOverrideToFaceThisPoint(self.drivetrainTarget)

    def initialize(self):
        self.drivetrainTarget = None

    def isFinished(self):
        return False


class ShootWhenReady(commands2.Command):
    """
    Usage example ( put it into configureButtonBindings(...) ):
    ```
    shootWhenReady = ShootWhenReady(
        goal=self.distanceToGoal,
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
        super().__init__()
        self.goal = goal
        self.shooter = shooter
        self.turret = turret
        self.drivetrain = drivetrain
        self.indexer = indexer
        self.indexerSpeed = indexerSpeed
        self.addRequirements(goal)
        self.addRequirements(shooter)
        self.addRequirements(indexer)
        # DO NOT ADDREQUIREMENTS(drivetrain), on purpose
        if turret is not None:
            self.addRequirements(turret)
        SmartDashboard.putString("FireWhenReady", "")

    def initialize(self):
        self.drivetrainTarget = None
        SmartDashboard.putString("FireWhenReady", "starting")

    def execute(self):
        rpm = self.goal.recommendedShooterRpm()
        self.shooter.setVelocityGoal(rpm, rpm * Constants.RPM_TOLERANCE_FACTOR)

        if self.turret is not None:
            direction = self.goal.recommendedTurretDirection()
            if direction is not None:
                self.turret.setPositionGoal(direction.degrees())

        # the gamepieces should *only* be fed into shooter if everything is ready (can add more pieces)
        turretNotReady = self.turret.notReady() if self.turret is not None else ""
        drivetrainNotReady = ""
        if self.drivetrain is not None:
            drivetrainNotReady = self.drivetrain.notPointingTo(self.drivetrainTarget, Constants.ANGLE_TOLERANCE_DEGREES)
        notReady = turretNotReady or drivetrainNotReady or self.shooter.notReady()
        if notReady:
            self.indexer.stop()
        else:
            self.indexer.feedGamepieceIntoShooter(self.indexerSpeed)
        SmartDashboard.putString("FireWhenReady", notReady if notReady else "firing!")

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.indexer.stop()
        if self.turret is not None:
            self.turret.stopAndReset()
        if self.drivetrainTarget is not None:
            self.drivetrain.stopOverrideToFaceThisPoint(self.drivetrainTarget)
        SmartDashboard.putString("FireWhenReady", "finished")

    def isFinished(self):
        return False
