# Code snippet for feed-and-shoot command

## Pre-requisites

Does your robot already have a Shooter subsystem in its code?
[Add it first](Adding_Shooter.md)

Does your robot already have an Intake subsystem in its code (to feed the gamepiece into shooter)?
[Add it first](Adding_Intake.md)

## Actual code

- **this goes to a new file `commands/shoot.py`**
  (note how you just need to write what to do: when command starts, when it runs and when it ends)

```python
from commands2 import Command
from wpilib import SmartDashboard, Timer

class ShootGamepiece(Command):
    def __init__(self, intake, shooter, shooterRPM) -> None:
        """
        :param shooter: the shooter subsystem, for shooting the gamepiece out
        :param intake: the intake subsystem to be used for feeding gamepices into shooter
        :param shooterRPM: how fast should the shooter flywheel spin before the gamepiece is fed into it
        """
        super().__init__()
        self.intake = intake
        self.shooter = shooter
        self.shooterRPM = shooterRPM
        self.addRequirements(intake)
        self.addRequirements(shooter)
        self.feedTime = 0

    def initialize(self) -> None:
        """Called when the command is initially scheduled: stop the intake and start spinning the shooter"""
        self.feedTime = 0  # pretend that we didn't feed the gamepiece to the shooter
        self.intake.stop()  # make sure that the intake is stopped

        #if self.intake.noGamepieceInside():
        #    self.cancel()
        #    return

        self.shooter.setVelocityGoal(self.shooterRPM)

    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""
        # if we didn't feed the gamepiece to the shooter
        if self.feedTime == 0:
            SmartDashboard.putNumber("shootActualRPM", self.shooter.getVelocity())
            SmartDashboard.putNumber("shootTargetRPM", self.shooter.getVelocityGoal())
            # ...and if the shooter velocity is pretty close to the target
            if self.shooter.getVelocity() > 0.9 * self.shooter.getVelocityGoal():
                # ...then feed the gamepiece into the shooter and make note of the time
                self.intake.feedGamepieceForward()
                self.feedTime = Timer.getFPGATimestamp()

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        if self.feedTime == 0.0:  # we have not fed the gamepiece into the shooter yet?
            return False  # False = command not finished yet

        if Timer.getFPGATimestamp() < self.feedTime + 0.5:  # we have not waited for 0.5 seconds after shooter feeding?
            return False  # False = command not finished yet

        # otherwise, the command is finished: return True
        return True

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        self.shooter.stop()
        self.intake.stop()

```


- **to run this command on "left bumper" button, this needs to be added to `robotcontainer.py` inside `configureButtonBindings` function**
```python
    def configureButtonBindings(self) -> None:
        # ...

        # the code below must be added:
        
        # create a command to shoot a picked up gamepiece with shooter speed 1500 rpm
        from commands.shoot import ShootGamepiece
        shootCommand = ShootGamepiece(self.intake, self.shooter, shooterRPM=1500).withTimeout(5.0)

        # assign this command to run *while* joystick "left bumper" is pressed (if you release button, command aborts)
        leftBumper = JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
        leftBumper.whileTrue(shootCommand)
```
