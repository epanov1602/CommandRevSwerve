from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkBaseConfig, ResetMode, PersistMode
from wpilib import SmartDashboard


class Constants:
    stallCurrentLimit = 5
    freeSpinCurrentLimit = 10
    limitRpm = 500


class Indexer(Subsystem):
    def __init__(self, leaderCanID, leaderInverted=True, followerCanID=None, followerInverted=False) -> None:
        super().__init__()

        # 1. setup the leader motor
        self.motor = SparkMax(leaderCanID, SparkBase.MotorType.kBrushless)

        self.motorConfig = SparkBaseConfig()
        self.motorConfig.inverted(leaderInverted)
        self.motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.motorConfig.smartCurrentLimit(
            Constants.stallCurrentLimit, Constants.freeSpinCurrentLimit, Constants.limitRpm)
        self.motor.configure(self.motorConfig,
                             ResetMode.kResetSafeParameters,
                             PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        if followerCanID is not None:
            self.followerMotor = SparkMax(followerCanID, SparkBase.MotorType.kBrushless)
            followerConfig = SparkBaseConfig()
            followerConfig.follow(leaderCanID, leaderInverted != followerInverted)
            followerConfig.smartCurrentLimit(
                Constants.stallCurrentLimit, Constants.freeSpinCurrentLimit, Constants.limitRpm)
            self.followerMotor.configure(followerConfig,
                                         ResetMode.kResetSafeParameters,
                                         PersistMode.kPersistParameters)

        # 3. safe initial state
        self._setSpeed(0.0)


    def feedGamepieceIntoShooter(self, speed=1.0):
        """
        Rush the gamepiece forward into the shooter, possibly at full speed (100%)
        """
        self._setSpeed(speed)


    def ejectGamepieceBackward(self, speed=0.25, speed2=None):
        """
        Eject the gamepiece back out of the indexer
        """
        self._setSpeed(-speed)


    def stop(self):
        self._setSpeed(0)


    def _setSpeed(self, speed):
        self.motor.set(speed)
        SmartDashboard.putNumber("Indexer", speed)
