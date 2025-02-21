
from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkLowLevel, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard, Timer


class Intake(Subsystem):
    def __init__(self,
                 leaderCanID,
                 leaderInverted=True,
                 followerCanID=None,
                 followerInverted=False,
                 rangeFinder=None,
                 rangeToGamepiece=None) -> None:
        """
        :param leaderCanID: CAN ID of the leader motor (or of your only motor)
        :param leaderInverted: is the leader motor inverted?
        :param followerCanID: CAN ID of the follower motor, if we have it
        :param followerInverted: is follower motor inverted?
        :param rangeFinder: do we have a rangefinder to sense gamepieces? (if using PlayingWithFusion, you can use: from playingwithfusion import TimeOfFlight; rangeFinder = TimeOfFlight(sensorCanId))
        :param rangeToGamepiece: any range closer than this will count as "gamepiece in"
        """
        super().__init__()

        # 0. state
        self.limitSwitchSensingGamepiece = False
        self.rangefinderBlockedByGamepiece = False
        self.sensingGamepiece = False

        self.desiredSpeedL = 0
        self.desiredSpeedF = 0
        self.stopIfSensingGamepiece = False
        self.rangefinderT1 = 0  # timestemp when rangefinger got blocked by an incoming gamepiece
        self.rangefinderT2 = 0  # timestamp when rangefinder got unblocked when incoming gamepiece has entered fully

        motorConfig = SparkBaseConfig()
        motorConfig.inverted(leaderInverted)
        motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(False)

        # 1. setup the leader motor
        self.motor = SparkMax(leaderCanID, SparkLowLevel.MotorType.kBrushless)
        self.motor.configure(motorConfig,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        if followerCanID is not None:
            motorConfig.inverted(followerInverted)
            self.followerMotor = SparkMax(followerCanID, SparkLowLevel.MotorType.kBrushless)
            self.followerMotor.configure(motorConfig,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        # (we want the intake to keep ~working if switch is broken during the game, so using "normally open")
        self._setSpeed(0)

        # 3. if we have a rangefinder, set it up
        self.rangeFinder = rangeFinder
        self.rangeToGamepiece = rangeToGamepiece


    def enableLimitSwitch(self):
        if not self.stopIfSensingGamepiece:
            self.stopIfSensingGamepiece = True
            self.rangefinderT1 = 0
            self.rangefinderT2 = 0


    def disableLimitSwitch(self):
        self.stopIfSensingGamepiece = False


    def isGamepiecePartlyIn(self):
        return self.rangefinderBlockedByGamepiece


    def isGamepieceInside(self) -> bool:
        return self.sensingGamepiece


    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()


    def isUnsafeToMoveElevator(self):
        if self.rangefinderT2 == 0 and self.rangefinderT1 != 0:
            # when gamepiece is only partly in, it is not safe to move elevator
            if self.rangefinderBlockedByGamepiece:
                return "intake not done intaking"


    def isLimitSwitchThinkingGamepieceInside(self):
        return self.limitSwitchSensingGamepiece


    def isRangefinderThinkingGamepieceInside(self):
        return (
                self.rangefinderT2 != 0 and
                self.rangefinderT1 != 0 and
                self.stopIfSensingGamepiece
        )


    def periodic(self):
        # 1. check if limit switch is sensing that gamepiece
        if self.limitSwitch is not None:
            self.limitSwitchSensingGamepiece = self.limitSwitch.get()
            SmartDashboard.putBoolean("intakeSwitchPressed", self.limitSwitchSensingGamepiece)

        # 2. check if rangefinder is sensing that gamepiece
        if self.rangeFinder is not None:
            range = self.rangeFinder.getRange()
            SmartDashboard.putNumber("intakeRangeToGamepiece", range)
            self.rangefinderBlockedByGamepiece = range != 0 and range <= self.rangeToGamepiece
            # manage the entry and exit time of gamepiece near rangefinder
            if self.stopIfSensingGamepiece:
                now = Timer.getFPGATimestamp()
                if self.rangefinderT1 == 0:
                    if self.rangefinderBlockedByGamepiece:
                        print(f"Intake: at t={now}, range={range} => gamepiece partly entered now")
                        self.rangefinderT1 = now
                elif self.rangefinderT2 == 0 and now > self.rangefinderT2 + 0.05:
                    if range > self.rangeToGamepiece:
                        print(f"Intake: at t={now}, range={range} => gamepiece fully entered {now - self.rangefinderT1} seconds later at speed {self.desiredSpeedL}")
                        self.rangefinderT2 = now
            SmartDashboard.putNumber("intakeT1", self.rangefinderT1)
            SmartDashboard.putNumber("intakeT2", self.rangefinderT2)

        # 3. we say we are sensing that gamepiece if either limit switch or rangefinder is sensing it
        limitSwitchThinkingItsInside = self.isLimitSwitchThinkingGamepieceInside()
        rangefinderThinkingItsInside = self.isRangefinderThinkingGamepieceInside()
        self.sensingGamepiece = limitSwitchThinkingItsInside or rangefinderThinkingItsInside

        SmartDashboard.putBoolean("intakeFull", self.sensingGamepiece)
        SmartDashboard.putBoolean("intakeFull_LS", limitSwitchThinkingItsInside)
        SmartDashboard.putBoolean("intakeFull_RF", rangefinderThinkingItsInside)

        # 4. if we are sensing the gamepiece, maybe stop that motor (otherwise, spin it)
        speedL, speedF = self.desiredSpeedL, self.desiredSpeedF
        if self.sensingGamepiece and self.stopIfSensingGamepiece:
            speedL, speedF = 0.0, 0.0
        SmartDashboard.putNumber("intakeDSpeed", self.desiredSpeedL)
        if self.followerMotor is not None:
            SmartDashboard.putNumber("intakeDSpeedF", self.desiredSpeedF)

        self.motor.set(speedL)
        SmartDashboard.putNumber("intakeSpeed", speedL)
        if self.followerMotor is not None:
            self.followerMotor.set(speedF)
            SmartDashboard.putNumber("intakeSpeedF", speedF)


    def intakeGamepiece(self, speed=0.25, speedF=None):
        """
        If the gamepiece is not inside, try to intake it
        """
        self.enableLimitSwitch()
        self._setSpeed(speed, speedF)
        print("Intake::intakeGamepiece")


    def feedGamepieceForward(self, speed=1.0, speedF=None):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(speed, speedF)
        print("Intake::feedGamepieceForward")


    def ejectGamepieceBackward(self, speed=0.25, speedF=None):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        if speedF is None:
            speedF = speed
        self._setSpeed(-speed, -speedF)
        print("Intake::ejectGamepiece")


    def intakeGamepieceDespiteLimitSwitch(self, speed=0.25, speedF=None):
        """
        Even if (possibly broken) limit switch thinks that the gamepiece is already inside, try to intake it
        """
        self.disableLimitSwitch()
        self._setSpeed(speed, speedF)


    def stop(self):
        self.desiredSpeedL, self.desiredSpeedF = 0.0, 0.0
        self.motor.set(0)
        if self.followerMotor is not None:
            self.followerMotor.set(0)
        print("Intake::stop")


    def _setSpeed(self, speedL, speedF=None):
        self.desiredSpeedL = speedL
        if speedF is not None:
            self.desiredSpeedF = speedF
        else:
            self.desiredSpeedF = speedL
