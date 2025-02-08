# LED strip as a part of Intake (light up when it has gamepiece)

Assuming you already have an intake similar to our [Intake Example](docs/Adding_Intake.md) , this is what you need:

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
            self.ledStrip.selectColor(0.87)   # blue color (see page 14-17 of Blinkin manual)
        else:
            self.ledStrip.selectColor(0)  # no color

        # ...

```
