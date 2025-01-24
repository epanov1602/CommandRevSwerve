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
