from __future__ import annotations
import commands2

from subsystems.intake import Intake


class IntakeGamepiece(commands2.Command):
    def __init__(self, intake: Intake, speed=0.115):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.addRequirements(intake)

    def end(self, interrupted: bool):
        self.intake.stop()  # stop at the end

    def initialize(self):
        self.intake.intakeGamepiece(self.speed)

    def isFinished(self) -> bool:
        return self.intake.isGamepieceInside()

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
        return False  # never finishes, so you should use it with ".withTimeout(...)" or with "button.whileTrue(...)"

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
        return False  # never finishes, so you should use it with ".withTimeout(...)" or with "button.whileTrue(...)"

    def execute(self):
        pass
