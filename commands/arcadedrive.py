from __future__ import annotations
import commands2

from wpimath.geometry import Rotation2d, Pose2d, Translation2d


class ArcadeDrive(commands2.Command):
    def __init__(self, driveSpeed, rotationSpeed, drivetrain, assumeManualInput=False):
        """
        Drive the robot at `driveSpeed` and `rotationSpeed` until this command is terminated.
        """
        super().__init__()

        self.driveSpeed = driveSpeed
        if not callable(driveSpeed):
            self.driveSpeed = lambda: driveSpeed

        self.rotationSpeed = rotationSpeed
        if not callable(rotationSpeed):
            self.rotationSpeed = lambda: rotationSpeed

        self.assumeManualInput = assumeManualInput
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        driveSpeed = self.driveSpeed()  # get the drive speed from the joystick or wherever it comes from
        rotationSpeed = self.rotationSpeed()  # get the turn speed from the joystick or wherever it comes from
        self.drivetrain.arcadeDrive(driveSpeed, rotationSpeed, assumeManualInput=self.assumeManualInput)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)  # stop at the end
