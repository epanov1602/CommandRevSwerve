import typing

from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkLowLevel, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard, Timer

EMA_RATE = 0.05
LED_STRIP_CHANNEL = 1       # PWM channel for LED strip
LED_COLOR_WHEN_FULL = 0.87  # color of LED when sensing gamepiece

class Intake(Subsystem):
    def __init__(self,
                 leaderCanID,
                 leaderInverted=True,
                 followerCanID=None,
                 followerInverted=False,
                 rangeFinder=None,
                 rangeToGamepiece=None,
                 recoilSpeed=0.0,
                 limitSwitchEnabled=False,
    ) -> None:
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
        self.rangefinderConsistentlyBlockedByGamepiece = 0.0
        self.sensingGamepiece = False

        self.desiredSpeedL = 0
        self.desiredSpeedF = 0
        self.stopIfSensingGamepiece = False
        self.rangefinderT1 = 0  # timestemp when rangefinger got blocked by an incoming gamepiece
        self.rangefinderT2 = 0  # timestamp when rangefinder got unblocked when incoming gamepiece has entered fully
        self.rangefinderT3 = 0  # if recoil is specified, timestamp when rangefinder got blocked again in recoil

        motorConfig = SparkBaseConfig()
        motorConfig.inverted(leaderInverted)
        motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(limitSwitchEnabled)

        # 1. setup the leader motor
        self.motor = SparkMax(leaderCanID, SparkLowLevel.MotorType.kBrushless)
        self.motor.configure(motorConfig,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        # (we want the intake to keep ~working if switch is broken or missing, so using "normally open")
        self.limitSwitch = self.motor.getForwardLimitSwitch() if limitSwitchEnabled else None

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        if followerCanID is not None:
            motorConfig.inverted(followerInverted)
            self.followerMotor = SparkMax(followerCanID, SparkLowLevel.MotorType.kBrushless)
            self.followerMotor.configure(motorConfig,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        self._setSpeed(0)

        # 3. if we have a rangefinder, set it up
        self.rangeFinder = rangeFinder
        self.rangeToGamepiece = rangeToGamepiece
        assert rangeToGamepiece > 0 or rangeFinder is None, f"if rangefinder is specified, rangeToGamepiece must be >0"
        self.recoilSpeed = recoilSpeed
        assert recoilSpeed >= 0, f"invalid recoilSpeed={recoilSpeed}, must be nonnegative"
        if recoilSpeed > 0:
            assert rangeFinder is not None, f"if recoilSpeed>0, rangeFinder must be not None"

        self.onSensingGamepiece = None

        # 4. Set up the LED strip
        from subsystems.ledstrip import LedStrip
        self.ledStrip = LedStrip(pwmChannel=LED_STRIP_CHANNEL)


    def setOnSensingGamepiece(self, callback: typing.Callable[[bool], None]):
        self.onSensingGamepiece = callback


    def enableLimitSwitch(self):
        if not self.stopIfSensingGamepiece:
            self.stopIfSensingGamepiece = True
            self.rangefinderT1 = 0
            self.rangefinderT2 = 0
            self.rangefinderT3 = 0


    def disableLimitSwitch(self):
        self.stopIfSensingGamepiece = False


    def isGamepieceInside(self) -> bool:
        return self.sensingGamepiece


    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()


    def assumeGamepieceInside(self) -> None:
        self.enableLimitSwitch()
        self.rangefinderT1, self.rangefinderT2, self.rangefinderT3 = 0.1, 0.2, 0.3


    def isUnsafeToMoveElevator(self):
        if self.rangefinderT2 != 0 and self.stopIfSensingGamepiece:
            return False  # exception: gamepiece is fully inside and nobody tried to eject it yet (2nd condition above)
        if self.rangefinderConsistentlyBlockedByGamepiece > 0.5:
            return f"intake with toy partly in"  # if this causes false alerts, decrease EMA_RATE


    def isLimitSwitchThinkingGamepieceInside(self):
        return self.limitSwitchSensingGamepiece


    def isRangefinderThinkingGamepieceInside(self):
        if self.recoilSpeed > 0:
            return (
                    self.rangefinderT3 != 0 and
                    self.rangefinderT1 != 0 and
                    self.stopIfSensingGamepiece
            )
        else:
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
        recoiling = False
        if self.rangeFinder is not None:
            range = self.rangeFinder.getRange()
            SmartDashboard.putNumber("intakeRange", range)
            self.rangefinderBlockedByGamepiece = range != 0 and range <= self.rangeToGamepiece
            self.updateT1T2T3(range)
            if self.stopIfSensingGamepiece and self.recoilSpeed > 0:
                recoiling = self.rangefinderT2 != 0 and self.rangefinderT3 == 0

            # exponentially weighted moving average (updates 50 times per second)
            self.rangefinderConsistentlyBlockedByGamepiece += EMA_RATE * (
                int(self.rangefinderBlockedByGamepiece) - self.rangefinderConsistentlyBlockedByGamepiece
            )
            SmartDashboard.putNumber("intakeRfBlocked", int(100 * self.rangefinderConsistentlyBlockedByGamepiece + 0.5))

        # 3. we say we are sensing that gamepiece if either limit switch or rangefinder is sensing it
        wasSensingGamepiece = self.sensingGamepiece
        limitSwitchThinkingItsInside = self.isLimitSwitchThinkingGamepieceInside()
        rangefinderThinkingItsInside = self.isRangefinderThinkingGamepieceInside()
        self.sensingGamepiece = limitSwitchThinkingItsInside or rangefinderThinkingItsInside

        # 5. Light the LED strip when sensing gamepiece
        if self.isGamepieceInside():
            self.ledStrip.selectColor(LED_COLOR_WHEN_FULL)
            # (other colors are on manual page 14-17 : https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf )
        else:
            self.ledStrip.selectColor(0)  # no color

        SmartDashboard.putBoolean("intakeRecoiling", recoiling)
        SmartDashboard.putBoolean("intakeFull", self.sensingGamepiece)
        SmartDashboard.putBoolean("intakeFull_LS", limitSwitchThinkingItsInside)
        SmartDashboard.putBoolean("intakeFull_RF", rangefinderThinkingItsInside)

        # 5. if we are sensing the gamepiece, maybe stop that motor (otherwise, spin it)
        speedL, speedF = self.desiredSpeedL, self.desiredSpeedF
        if recoiling:
            speedL, speedF = -self.recoilSpeed, -self.recoilSpeed
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

        if wasSensingGamepiece != self.sensingGamepiece and self.onSensingGamepiece is not None:
            self.onSensingGamepiece(self.sensingGamepiece)


    def updateT1T2T3(self, range):
        now = Timer.getFPGATimestamp()

        # is the sensor definitely blocked by gamepiece? definily not blocked? or unsure?
        definitelyBlocked = 0 < range and range <= self.rangeToGamepiece
        definitelyNotBlocked = range > self.rangeToGamepiece

        if not self.stopIfSensingGamepiece:
            pass  # we are not intaking a new gamepiece at the moment
        elif self.rangefinderT1 == 0:
            if definitelyBlocked:
                self.rangefinderT1 = now
                print(f"Intake: at t={now}, range={range} => gamepiece partly entered, speed {self.desiredSpeedL}")
        elif self.rangefinderT2 == 0:
            if definitelyNotBlocked and now > self.rangefinderT1 + 0.05:
                self.rangefinderT2 = now
                print(f"Intake: at t={now}, range={range} => gamepiece fully entered, speed {self.desiredSpeedL}")
        elif self.rangefinderT3 == 0:
            if definitelyBlocked and now > self.rangefinderT2 + 0.025:
                self.rangefinderT3 = now
                print(f"Intake: at t={now}, range={range} => gamepiece recoiled, speed {-self.recoilSpeed}")

        SmartDashboard.putNumber("intakeT1", self.rangefinderT1)
        SmartDashboard.putNumber("intakeT2", self.rangefinderT2)
        SmartDashboard.putNumber("intakeT3", self.rangefinderT3)


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
