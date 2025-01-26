# Code snippet for adding a dual-motor shooter

- **this goes to a new file `subsystems/shooter.py`**
```python
from commands2 import Subsystem
from rev import SparkFlex, SparkBase, SparkLowLevel, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard


class ShooterConstants:
    kShooterMotorA_CANID = 10
    kShooterMotorB_CANID = 11

    initialMinRPM = 600  # minimum sensible nonzero RPM
    initialMaxRPM = 6000
    initialFF = 1.4 / 10000
    initialP = 5.0 / 10000
    initialD = 0.0 / 10000


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.leadMotor = SparkFlex(ShooterConstants.kShooterMotorA_CANID, SparkLowLevel.MotorType.kBrushless)
        self.leadMotor.configure(
            _getLeadMotorConfig(),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        self.followMotor = SparkFlex(ShooterConstants.kShooterMotorB_CANID, SparkLowLevel.MotorType.kBrushless)
        self.followMotor.configure(
            _getFollowMotorConfig(),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        self.pidController = self.leadMotor.getClosedLoopController()
        self.encoder = self.leadMotor.getEncoder()
        self.velocityGoal = 0

        self.reportedVelocityGoal = 0
        self.reportedVelocitySeen = 0

    def getVelocity(self):
        return -self.encoder.getVelocity()

    def getVelocityGoal(self):
        return -self.velocityGoal

    def setVelocityGoal(self, rpm):
        rpm = -rpm  # hack to avoid inverting the motor direction (inversion setting gets lost during brownouts in Rev)
        if rpm < -ShooterConstants.initialMaxRPM or rpm > ShooterConstants.initialMaxRPM:
            print(f"Shooter Illegal Velocity Goal: {-rpm}")
            return
        if rpm != self.velocityGoal:
            print(f"Shooter::setVelocityGoal({-rpm})")
        self.velocityGoal = rpm
        self.pidController.setReference(self.velocityGoal, SparkBase.ControlType.kVelocity)

    def periodic(self):
        seen = self.getVelocity()
        goal = self.getVelocityGoal()
        if goal != self.reportedVelocityGoal or abs(seen - self.reportedVelocitySeen) >= 0.001 * seen:
            SmartDashboard.putNumber("shooter.rpmSeen", seen)
            self.reportedVelocitySeen = seen
            SmartDashboard.putNumber("shooter.rpmGoal", goal)
            self.reportedVelocityGoal = goal

    def stop(self):
        print(f"Shooter::stop")
        self.leadMotor.stopMotor()
        self.velocityGoal = 0


def _getLeadMotorConfig() -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(True)
    config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.closedLoop.pid(ShooterConstants.initialP, 0.0, ShooterConstants.initialD)
    config.closedLoop.velocityFF(ShooterConstants.initialFF)
    config.closedLoop.outputRange(-1, +1)
    return config

def _getFollowMotorConfig():
    followConfig = SparkBaseConfig()
    followConfig.follow(ShooterConstants.kShooterMotorA_CANID, False)
    followConfig.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    return followConfig

```


- **in `robotcontainer.py`, inside `__init__` function, we need to add one `Shooter` to the list of robot subsystems**
```python
        # these two lines go to __init__ function
        from subsystems.shooter import Shooter
        self.shooter = Shooter()
```

- **and finally add a joystick button to run simple commands to start/stop the shooter**

(inside of `robotcontainer.py` in `configureButtonBindings` function)
```python
    def configureButtonBindings(self) -> None:
        # ...
 
        # this code must be added: see how "Y" button handlers are defined
        yButton = JoystickButton(self.driverController, XboxController.Button.kY)
        # when "Y" button is pressed, set the speed goal on the shooter to 1500 rpm
        yButton.onTrue(InstantCommand(lambda: self.shooter.setVelocityGoal(1500), self.shooter))
        # when "Y" button is no longer pressed, stop wasting energy on the shooter
        yButton.onFalse(InstantCommand(self.shooter.stop, self.shooter))
        # end of the code that must be added
```

- **if some of the symbols are showing up as "unresolved" errors, these import lines need to be added in the beginning of `robotcontainer.py`**
```python
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController
```
