## Blinkin LED Strip Standalone
This example is for adding this LED strip (see page 8 for numeric codes of different colors):

https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf

## This code goes to `subsystems/ledstrip.py`

```python

from wpilib import Spark
from commands2 import Subsystem

class LedStrip(Subsystem):
    def __init__(self, pwmChannel: int) -> None:
        self.blinkin = Spark(pwmChannel)

    def selectColor(self, color: float) -> None:
        """
        :param color: between -1.0 and +1.0, see page 8 in Rev Blinkin manual
        (https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf)
        """
        self.blinkin.set(color)

```

## This code goes to `__init__` function of `robotcontainer.py` 
You can add 1, 2 or more LED strips, just have to connect them to PWM channels of RoboRIO.

```python

# ...

    def __init__(self) -> None:
        # ...

        # add LED strips (ledStrip1, ledStrip2, ...)
        from subsystems.ledstrip import LedStrip
        self.ledStrip1 = LedStrip(pwmChannel=1)
        self.ledStrip2 = LedStrip(pwmChannel=2)
        # you can add ledStrip3, etc.

        # ...

```

## If you want a command to switch LED to blue via a button, something like this goes to `configureButtonBindings` in `robotcontainer.py`
```python

    def configureButtonBindings(self) -> None:
        # ...

        # left bumper button can turn LED blue
        # (but you can also call ledStrip1.selectColor(0.87) from any real command)
        leftBumperButton = JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
        # 0.87 = blue (says page 8 of https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf)
        from commands2 import InstantCommand
        leftBumperButton.onTrue(InstantCommand(lambda: self.ledStrip1.selectColor(0.87), self.ledStrip1))

        # ...

```
