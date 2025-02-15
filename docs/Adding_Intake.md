# Code snippet for adding a single-motor or dual-motor intake

- **this goes to `subsystems/intake.py` file**
<details>
    <summary>(option 1: click to expand code for intake without rangefinder)</summary>

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

    def feedGamepieceForward(self, speed=1.0, speed2=None):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(speed)
        print("Intake::feedGamepieceForward")

    def ejectGamepieceBackward(self, speed=0.25, speed2=None):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        self._setSpeed(-speed)
        print("Intake::ejectGamepiece")

    def intakeGamepieceDespiteLimitSwitch(self, speed=0.25, speed2=None):
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

<details>
    <summary>(option 2: click to expand code for intake that can eject/feed at motor1 speed different from motor2 speed (to spin the object being ejected)</summary>

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
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        self.motor.configure(self.motorConfig,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        self.followerConfig = None
        if followerCanID is not None:
            self.followerConfig = SparkBaseConfig()
            self.followerConfig.follow(leaderCanID, leaderInverted != followerInverted)
            self.notFollowingConfig = SparkBaseConfig()
            self.notFollowingConfig.inverted(followerInverted)
            self.notFollowingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            self.notFollowingConfig.limitSwitch.forwardLimitSwitchEnabled(False)
            self.followerMotor = SparkMax(followerCanID, SparkLowLevel.MotorType.kBrushless)
            self.followerMotor.configure(self.followerConfig,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)
        self.following = self.followerMotor is not None

        # 3. safe initial state
        self._setSpeed(0.0)


    def enableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        if self.followerMotor is not None:
            self.followerMotor.configure(
                self.followerConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters)
            self.followerMotor.resumeFollowerMode()
            self.following = True
        # ^^ do not reset and do not persist, just enable the switch

    def disableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(False)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        if self.followerMotor is not None:
            self.followerMotor.configure(
                self.notFollowingConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters)
            self.followerMotor.pauseFollowerMode()
            self.following = False
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

    def feedGamepieceForward(self, speed=1.0, motor2speed=None):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(speed, motor2speed)
        print("Intake::feedGamepieceForward")

    def ejectGamepieceBackward(self, speed=0.25, motor2speed=None):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        self._setSpeed(-speed, -motor2speed if motor2speed is not None else -speed)
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

    def _setSpeed(self, motor1speed, motor2speed=None):
        self.motor.set(motor1speed)
        if motor2speed is None or self.following:
            motor2speed = motor1speed
        if not self.following:
            self.followerMotor.set(motor2speed)
        SmartDashboard.putNumber("intakeSpeed", motor1speed)
        SmartDashboard.putNumber("intakeSpeed2", motor2speed)

```

</details>

<details>
    <summary>(option 3: click to expand code for intake with an optional rangefinder!)</summary>

```python

from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkLowLevel, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard


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
        self.rangeFinderSensingGamepiece = False
        self.sensingGamepiece = False

        self.desiredSpeedL = 0
        self.desiredSpeedF = 0
        self.stopIfSensingGamepiece = False

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
        self.stopIfSensingGamepiece = True


    def disableLimitSwitch(self):
        self.stopIfSensingGamepiece = False


    def isGamepieceInside(self) -> bool:
        return self.sensingGamepiece


    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()


    def periodic(self):
        # 1. check if limit switch or rangefinder is sensing that gamepiece
        if self.limitSwitch is not None:
            self.limitSwitchSensingGamepiece = self.limitSwitch.get()
            SmartDashboard.putBoolean("intakeSwitchPressed", self.limitSwitchSensingGamepiece)
        if self.rangeFinder is not None:
            range = self.rangeFinder.getRange()
            SmartDashboard.putNumber("intakeRangeToGamepiece", range)
            self.rangeFinderSensingGamepiece = range != 0 and range <= self.rangeToGamepiece

        # 2. we say we are sensing that gamepiece if either limit switch or rangefinder is sensing it
        self.sensingGamepiece = self.limitSwitchSensingGamepiece or self.rangeFinderSensingGamepiece
        SmartDashboard.putBoolean("intakeFull", self.sensingGamepiece)
        SmartDashboard.putNumber("intakeDesiredSpeedL", self.desiredSpeedL)
        if self.followerMotor is not None:
            SmartDashboard.putNumber("intakeDesiredSpeedF", self.desiredSpeedF)

        # 3. if we are sensing the gamepiece, maybe stop that motor (otherwise, spin it)
        speedL, speedF = self.desiredSpeedL, self.desiredSpeedF
        if self.sensingGamepiece and self.stopIfSensingGamepiece:
            speedL, speedF = 0.0, 0.0

        self.motor.set(speedL)
        SmartDashboard.putNumber("intakeSpeedL", speedL)
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
    def __init__(self, intake: Intake, speed=0.25, speed2=None):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.speed2 = speed2
        self.addRequirements(intake)

    def end(self, interrupted: bool):
        self.intake.stop()  # stop at the end

    def initialize(self):
        self.intake.feedGamepieceForward(self.speed, self.speed2)

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        pass



class IntakeEjectGamepieceBackward(commands2.Command):
    def __init__(self, intake: Intake, speed=1.0, speed2=None):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.speed2 = speed2
        self.addRequirements(intake)

    def end(self, interrupted: bool):
        self.intake.stop()  # stop at the end

    def initialize(self):
        self.intake.ejectGamepieceBackward(self.speed, self.speed2)

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

        # when "Y" button is pressed, start feeding the gamepiece forward at intake motor1 speed different from intake motor2 speed
        # (to spin the gamepiece, one intake motor can be spinning faster than another)
        yButton = JoystickButton(self.driverController, XboxController.Button.kY)
        intakeFeedFwdCmd2 = IntakeFeedGamepieceForward(self.intake, speed=0.5, speed2=0.8).withTimeout(0.3)
        yButton.onTrue(intakeFeedFwdCmd2)

        # end of the code that must be added

        # ...
```

- **if some of the symbols are showing up as "unresolved" errors in `robotcontainer.py`, these import lines need to be added in the beginning of `robotcontainer.py`**
```python
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController
```
