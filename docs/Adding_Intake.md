# Code snippet for adding a single-motor intake

- **this goes to a new file `subsystems/intake.py`**
```python
from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkLowLevel, SparkLimitSwitch, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard


class IntakeConstants:
    kIntakeMotor_CANID = 9

    kIntakeSpeedSetpoint = 0.25  # setpoints are between -1.0 and +1.0 (they are not in RPM units)

class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.motor = SparkMax(IntakeConstants.kIntakeMotor_CANID, SparkLowLevel.MotorType.kBrushless)

        self.motorConfig = SparkBaseConfig()
        self.motorConfig.inverted(True)
        self.motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)  # by default disabled (until `intakeGamepiece()`)

        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # (we want the intake to keep ~working if switch is broken during the game, so using "normally open")

    def periodic(self):
        pass  # nothing to do

    def enableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        # ^^ do not reset and do not persist, just enable the switch

    def disableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(False)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        # ^^ do not reset and do not persist, just disable the switch

    def isGamepieceInside(self) -> bool:
        return self.limitSwitch.get()

    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()

    def intakeGamepiece(self):
        """
        If the gamepiece is not inside, try to intake it
        """
        self.enableLimitSwitch()
        self._setSpeed(IntakeConstants.kIntakeSpeedSetpoint)
        print("Intake::intakeGamepiece")

    def feedGamepieceForward(self):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(1.0)
        print("Intake::feedGamepieceForward")

    def ejectGamepiece(self):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        self._setSpeed(-IntakeConstants.kIntakeSpeedSetpoint)
        print("Intake::ejectGamepiece")

    def intakeGamepieceDespiteLimitSwitch(self):
        """
        Even if (possibly broken) limit switch thinks that the gamepiece is already inside, try to intake it
        """
        self.disableLimitSwitch()
        self._setSpeed(IntakeConstants.kIntakeSpeedSetpoint)

    def stop(self):
        self._setSpeed(0)
        print("Intake::stop")

    def _setSpeed(self, speed):
        if speed != 0:
            self.motor.setInverted(True)  # motor can forget that it was inverted
        self.motor.set(speed)
        SmartDashboard.putNumber("intakeSpeed", speed)

```


- **in `robotcontainer.py`, side `__init__` function, we need to add one `Intake` to the list of robot subsystems**
```python
        # these two lines go to __init__ function
        from subsystems.intake import Intake
        self.intake = Intake()
```

- **and finally add two joystick buttons to run simple commands to start/stop/reverse the intake**

(inside of `robotcontainer.py` in `configureButtonBindings` function)
```python
    def configureButtonBindings(self) -> None:
        # ...
 
        # this code must be added: see how "A" and "B" button handlers are defined

        aButton = JoystickButton(self.driverController, XboxController.Button.kA)
        # when "A" button is pressed, start intaking the gamepiece
        aButton.onTrue(InstantCommand(self.intake.intakeGamepiece, self.intake))
        # when "A" button is no longer pressed, stop the intake even if it is not done
        aButton.onFalse(InstantCommand(self.intake.stop, self.intake))

        bButton = JoystickButton(self.driverController, XboxController.Button.kB)
        # when "B" button is pressed, start ejecting the gamepiece
        bButton.onTrue(InstantCommand(self.intake.ejectGamepiece, self.intake))
        # when "B" button is no longer pressed, stop ejecting the gamepiece even if it is still in
        bButton.onFalse(InstantCommand(self.intake.stop, self.intake))

        # end of the code that must be added
```

- **if some of the symbols are showing up as "unresolved" errors, these import lines need to be added in the beginning of `robotcontainer.py`**
```python
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController
```
