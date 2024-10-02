# Code snippet for pick-up-gamepiece command

## Pre-requisites

Does your robot already have an Intake subsystem in its code?
[Add it first](Adding_Intake.md)

## Actual code

- **this goes to a new file `commands/pickup.py`**
  (note how you just need to write what to do: when command starts, when it runs and when it ends)

```python
from commands2 import Command


class PickupGamepiece(Command):
    def __init__(self, intake, drivetrain, drivingSpeed: float) -> None:
        """
        :param intake: the intake subsystem to be used for gamepiece pickup
        :param drivetrain: the drivetrain to bs used for driving towards gamepiece
        :param drivingSpeed: the driving speed
        """
        super().__init__()
        self.drivingSpeed = drivingSpeed
        self.intake = intake
        self.drivetrain = drivetrain
        self.addRequirements(intake)
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        self.intake.intakeGamepiece()

    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""
        self.drivetrain.arcadeDrive(self.drivingSpeed, 0)

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        self.drivetrain.arcadeDrive(0, 0)
        self.intake.stop()

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        if self.intake.isGamepieceInside():
            return True
        else:
            return False

```


- **to run this command on "right bumper" button, this needs to be added to `robotcontainer.py` inside `configureButtonBindings` function**
```python
    def configureButtonBindings(self) -> None:
        # ...

        # the code below must be added:
        
        # create a command to pick up a gamepiece while driving at speed 0.3 towards it
        from commands.pickup import PickupGamepiece
        pickupCommand = PickupGamepiece(self.intake, self.robotDrive, drivingSpeed=0.3)

        # assign this command to run *while* joystick "right bumper" button is pressed
        rightBumper = JoystickButton(self.driverController, XboxController.Button.kRightBumper)
        rightBumper.whileTrue(pickupCommand)
```
