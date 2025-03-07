import typing
from wpilib import Spark
from commands2 import Subsystem

class LedStrip(Subsystem):
    def __init__(self, pwmChannel: int) -> None:
        super().__init__()
        self.blinkin = Spark(pwmChannel)

    def selectColor(self, color: float) -> None:
        """
        :param color: between -1.0 and +1.0, see page 8 in Rev Blinkin manual
        (https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf)
        """
        self.blinkin.set(color)
