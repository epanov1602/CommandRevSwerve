import commands2
from commands.aimtodirection import AimToDirection


class FindObject(commands2.Command):

    def __init__(self, camera, drivetrain, turnDegrees=-45, turnSpeed=1.0, waitSeconds=0.1):
        super().__init__()
        self.camera0 = camera
        self.drivetrain = drivetrain
        self.addRequirements(camera)
        self.addRequirements(drivetrain)

        # to find object, we will be running (wait + turn) + (wait + turn) ... in a long cycle, until object is detected
        wait = commands2.WaitCommand(waitSeconds)
        turn = AimToDirection(
            degrees=lambda: self.drivetrain.getHeading().degrees() + turnDegrees,
            drivetrain=self.drivetrain,
            speed=turnSpeed
        )

        self.subcommand = wait.andThen(turn)

    def initialize(self):
        self.subcommand.initialize()

    def isFinished(self) -> bool:
        return self.camera0.hasDetection()

    def execute(self):
        # 1. just execute the wait+turn subcommand
        self.subcommand.execute()

        # 2. and if that command is finished, start it again (we are doing cycles, right?)
        if self.subcommand.isFinished():
            self.subcommand.end(False)
            self.subcommand.initialize()

    def end(self, interrupted: bool):
        self.subcommand.end(interrupted)
