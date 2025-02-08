# LED strip as a part of Intake (light up when it has gamepiece)

Assuming you already have an intake similar to our [Intake Examples](docs/Adding_Intake.md) , this is what you need:

### this goes to `__init__` function of `subsystems/intake.py`

```python3

    def __init__(self):

        # ...

        # add LED strip
        from subsystems.ledstrip import LedStrip
        self.ledStrip = LedStrip(pwmChannel=1)  # if your LED strip is plugged into RoboRIO PWM channel 1

        # ...

```

### this goes to `periodic` function of `subsystems/intake.py`

```python3

     def periodic(self):
        # ...

        if self.isGamepieceInside():
            self.ledStrip.selectColor(0.87)   # blue color
            # (other colors are on manual page 14-17 : https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf )
        else:
            self.ledStrip.selectColor(0)  # no color

        # ...

```
