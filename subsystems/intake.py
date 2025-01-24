from commands2 import Subsystem
from rev import CANSparkMax, CANSparkBase, CANSparkLowLevel, SparkLimitSwitch
from wpilib import SmartDashboard


class IntakeConstants:
    kIntakeMotor_CANID = 9

    kIntakeSpeedSetpoint = 0.25  # setpoints are between -1.0 and +1.0 (they are not in RPM units)

class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.motor = CANSparkMax(IntakeConstants.kIntakeMotor_CANID, CANSparkLowLevel.MotorType.kBrushless)
        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(True)
        self.motor.setIdleMode(CANSparkBase.IdleMode.kBrake)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
        self.limitSwitch.enableLimitSwitch(True)

        # we want most of these settings to survive after a "brownout" event, so calling motor.burnFlash()
        self.motor.burnFlash()

        # (we want the intake to keep ~working if switch is broken during the game, so using "normally open")

    def periodic(self):
        pass  # nothing to do

    def isGamepieceInside(self) -> bool:
        return self.limitSwitch.get()

    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()

    def intakeGamepiece(self):
        """
        If the gamepiece is not inside, try to intake it
        """
        self.limitSwitch.enableLimitSwitch(True)
        self._setSpeed(IntakeConstants.kIntakeSpeedSetpoint)
        print("Intake::intakeGamepiece")

    def feedGamepieceForward(self):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.limitSwitch.enableLimitSwitch(False)
        self._setSpeed(1.0)
        print("Intake::feedGamepieceForward")

    def ejectGamepiece(self):
        """
        Eject the gamepiece back out of the intake
        """
        self.limitSwitch.enableLimitSwitch(False)
        self._setSpeed(-IntakeConstants.kIntakeSpeedSetpoint)
        print("Intake::ejectGamepiece")

    def intakeGamepieceDespiteLimitSwitch(self):
        """
        Even if (possibly broken) limit switch thinks that the gamepiece is already inside, try to intake it
        """
        self.limitSwitch.enableLimitSwitch(False)
        self._setSpeed(IntakeConstants.kIntakeSpeedSetpoint)

    def stop(self):
        self._setSpeed(0)
        print("Intake::stop")

    def _setSpeed(self, speed):
        if speed != 0:
            self.motor.setInverted(True)  # motor can forget that it was inverted
        self.motor.set(speed)
        SmartDashboard.putNumber("intakeSpeed", speed)
