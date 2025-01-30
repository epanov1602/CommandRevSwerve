# Code snippet for adding a single-motor or dual-motor intake

- **this goes to a new file `subsystems/intake.py`**
<details>
    <summary>(click to expand)</summary>

```python
from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkLowLevel, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard


class Intake(Subsystem):
    def __init__(self, leaderCanID, leaderInverted=True, followerCanID=None, followerInverted=False) -> None:
        super().__init__()

        # 1. setup the leader motor
        self.motor = SparkMax(leaderCanID, SparkLowLevel.MotorType.kBrushless)

        self.motorConfig = SparkBaseConfig()
        self.motorConfig.inverted(leaderInverted)
        self.motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)  # by default disabled (until `intakeGamepiece()`)
        self.motor.configure(self.motorConfig,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        if followerCanID is not None:
            self.followerMotor = SparkMax(followerCanID, SparkLowLevel.MotorType.kBrushless)
            followerConfig = SparkBaseConfig()
            followerConfig.follow(leaderCanID, leaderInverted != followerInverted)
            self.followerMotor.configure(followerConfig,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        # 3. safe initial state
        self._setSpeed(0.0)


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

    def periodic(self):
        SmartDashboard.putBoolean("intakeFull", self.limitSwitch.get())

    def intakeGamepiece(self, speed=0.25):
        """
        If the gamepiece is not inside, try to intake it
        """
        self.enableLimitSwitch()
        self._setSpeed(speed)
        print("Intake::intakeGamepiece")

    def feedGamepieceForward(self, speed=1.0):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(speed)
        print("Intake::feedGamepieceForward")

    def ejectGamepieceBackward(self, speed=0.25):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        self._setSpeed(-speed)
        print("Intake::ejectGamepiece")

    def intakeGamepieceDespiteLimitSwitch(self, speed=0.25):
        """
        Even if (possibly broken) limit switch thinks that the gamepiece is already inside, try to intake it
        """
        self.disableLimitSwitch()
        self._setSpeed(speed)

    def stop(self):
        self._setSpeed(0)
        print("Intake::stop")

    def _setSpeed(self, speed):
        self.motor.set(speed)
        SmartDashboard.putNumber("intakeSpeed", speed)

```

</details>

- **these few commands you can put into `commands/intakecommands.py`, so you can later import them**
<details>
    <summary>(click to expand)</summary>

```python
from __future__ import annotations
import commands2

from subsystems.intake import Intake


class IntakeGamepiece(commands2.Command):
    def __init__(self, intake: Intake, speed=0.25):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.addRequirements(intake)

    def end(self, interrupted: bool):
        self.intake.stop()  # stop at the end

    def initialize(self):
        self.intake.intakeGamepiece(self.speed)

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        pass


class IntakeFeedGamepieceForward(commands2.Command):
    def __init__(self, intake: Intake, speed=0.25):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.addRequirements(intake)

    def end(self, interrupted: bool):
        self.intake.stop()  # stop at the end

    def initialize(self):
        self.intake.feedGamepieceForward(self.speed)

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        pass



class IntakeEjectGamepieceBackward(commands2.Command):
    def __init__(self, intake: Intake, speed=1.0):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.addRequirements(intake)

    def end(self, interrupted: bool):
        self.intake.stop()  # stop at the end

    def initialize(self):
        self.intake.ejectGamepieceBackward(self.speed)

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        pass

```

</details>

- **in `robotcontainer.py`, side `__init__` function, we need to add one `Intake` to the list of robot subsystems**
```python
        # these two lines go to __init__ function
        from subsystems.intake import Intake
        self.intake = Intake(leaderCanID=9, leaderInverted=True)
        # ^^ you can also specify followerCanID= and followerInverted= if the intake has a follower motor
```

- **and finally add simple joystick buttons to run simple commands to start/stop/reverse the intake**

(inside of `robotcontainer.py` in `configureButtonBindings` function)
```python
    def configureButtonBindings(self) -> None:
        # ...

        # this code must be added: see how "A" and "B" button handlers are defined
        from commands.intakecommands import IntakeGamepiece, IntakeFeedGamepieceForward, IntakeEjectGamepieceBackward
        from commands2.instantcommand import InstantCommand

        # when "A" button is pressed, intake the gamepiece
        aButton = JoystickButton(self.driverController, XboxController.Button.kA)
        intakeCmd = IntakeGamepiece(self.intake, speed=0.2)
        aButton.onTrue(intakeCmd)

        # when "B" button is pressed, start feeding the gamepiece forward and give it 0.3 seconds to complete
        bButton = JoystickButton(self.driverController, XboxController.Button.kB)
        intakeFeedFwdCmd = IntakeFeedGamepieceForward(self.intake, speed=0.5).withTimeout(0.3)
        bButton.onTrue(intakeFeedFwdCmd)

        # end of the code that must be added

        # ...
```

- **if some of the symbols are showing up as "unresolved" errors in `robotcontainer.py`, these import lines need to be added in the beginning of `robotcontainer.py`**
```python
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController
```
