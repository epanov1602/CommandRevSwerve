# How to add a trajectory picker command

See how it works: https://www.youtube.com/watch?v=eYY4uIxZJlo

### This goes to `commands/trajectory_picker.py`

<details>
  <summary>(click to expand)</summary>

```python3

from __future__ import annotations

from commands2 import SequentialCommandGroup
from wpilib import SmartDashboard, Field2d
import commands2

class TrajectoryPicker(commands2.Command):
    # see the 2025 map of destinations at field map:
    # https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf

    def __init__(self, fieldDashboard: Field2d | None, subsystems: [], dashboardName="trajectory"):
        self.fieldDashboard = fieldDashboard
        self.commands = []
        self.nameToIndex = {}
        self.chosenIndex = 0
        self.dashboardName = dashboardName
        self.running = None
        for subsystem in subsystems:
            self.addRequirements(subsystem)

    def addCommands(self, name, *commands):
        assert name not in self.nameToIndex, f"commands for trajectory {name} were already added"
        index = len(self.commands)
        trajectory = []
        for command in commands:
            if hasattr(command, "trajectoryToDisplay"):
                trajectory = command.trajectoryToDisplay()
                break
        reversed = None
        for command in commands:
            if hasattr(command, "reversed"):
                reversed = command.reversed()
                if not isinstance(reversed, commands2.Command):
                    print("WARNING: command.reversed() didn't return a commands2.Command")
                    continue
                break
        self.commands.append((name, SequentialCommandGroup(*commands), reversed, trajectory))
        self.nameToIndex[name] = index
        if index == 0:
            self.updateDashboard()

    def updateDashboard(self):
        name = "?"
        trajectory = []
        if self.chosenIndex >= 0 and self.chosenIndex < len(self.commands):
            name, command, reversed, trajectory = self.commands[self.chosenIndex]
        if self.fieldDashboard is not None:
            self.fieldDashboard.getObject("traj").setPoses(trajectory)
        SmartDashboard.putString(self.dashboardName, name)

    def pickTrajectory(self, name):
        if name not in self.nameToIndex:
            example = f"example: trajectoryPicker.addCommands('{name}', command1, command2, ...)"
            print(f"WARNING: trajectory {name} was never added to this TrajectoryPicker ({example})")
            SmartDashboard.putString(self.dashboardName, name + "?")  # display anyway
            return  # no such trajectory
        self.chosenIndex = self.nameToIndex[name]
        self.updateDashboard()

    def nextTrajectory(self):
        self.chosenIndex += 1
        if self.chosenIndex >= len(self.commands):
            self.chosenIndex = max([0, len(self.commands) - 1])
        self.updateDashboard()

    def previousTrajectory(self):
        self.chosenIndex -= 1
        if self.chosenIndex < 0:
            self.chosenIndex = 0
        self.updateDashboard()

    def end(self, interrupted: bool):
        if self.running is not None:
            self.running.end(interrupted=True)
            self.running = None

    def initialize(self):
        if self.running is not None:
            self.end(interrupted=True)
        if self.chosenIndex >= 0 and self.chosenIndex < len(self.commands):
            name, command, reversed, trajectory = self.commands[self.chosenIndex]
            self.running = command
            if self.running is not None:
                self.running.initialize()

    def execute(self):
        if self.running is not None:
            self.running.execute()

    def isFinished(self) -> bool:
        if self.running is not None:
            return self.running.isFinished()
        else:
            return False

    def initializeReversed(self):
        if self.running is not None:
            self.end(interrupted=True)
        if 0 <= self.chosenIndex < len(self.commands):
            name, command, reversed, trajectory = self.commands[self.chosenIndex]
            self.running = reversed
            if self.running:
                self.running.initialize()


class ReversedTrajectoryPicker(commands2.Command):
    def __init__(self, trajectoryPicker: TrajectoryPicker, subsystems=None):
        self.trajectoryPicker = trajectoryPicker
        if subsystems is None:
            subsystems = trajectoryPicker.getRequirements()
        for subsystem in subsystems:
            self.addRequirements(subsystem)

    def end(self, interrupted: bool):
        self.trajectoryPicker.end(interrupted)

    def initialize(self):
        self.trajectoryPicker.initializeReversed()

    def execute(self):
        self.trajectoryPicker.execute()

    def isFinished(self) -> bool:
        return self.trajectoryPicker.isFinished()

```
</details>

### Add trajectory picker to your robot inside `configureButtonBindings()` function in your `robotcontainer.py`

<details>
  <summary>(click to expand)</summary>

```python3
    def configureButtonBindings():
    # ...

    # part A: add trajectory picker and add trajectory commands into it
    from commands.trajectory_picker import TrajectoryPicker

    self.trajectoryPicker = TrajectoryPicker(
        self.robotDrive.field,
        # which subsystems must be locked for exclusive use?
        subsystems=[self.robotDrive, self.intake],
    )

    from commands.trajectory import JerkyTrajectory

    goToPos1 = JerkyTrajectory(
        # swerve=True, # uncomment if you have a swerve drive (or you can also use swerve="last-point")
        drivetrain=self.robotDrive,
        endpoint=(5.54, 3.99, -180),
        waypoints=[
            (1.07, 0.77, 60.0),
            (2.41, 1.31, 0.9),
            (3.66, 1.16, -20.0),
            (5.91, 1.034, 0.9),
            (7.00, 1.70, 70.0),
            (7.00, 3.39, 122.97),
        ],
        speed=0.2
    )
    self.trajectoryPicker.addCommands("to-pos1", goToPos1)

    goToPos2 = JerkyTrajectory(
        # swerve=True, # uncomment if you have a swerve drive (you can also use swerve="last-point")
        drivetrain=self.robotDrive,
        endpoint=(3.92, 2.97, 60.0),
        waypoints=[
            (1.07, 0.77, 60.0),
            (2.937, 1.897, 39.66),
            (3.680, 2.580, 60.0),
        ],
        speed=0.2
    )
    self.trajectoryPicker.addCommands("to-pos2", goToPos2)

    # ...
```
</details>

### Decide which joystick buttons to control that trajectory picker (also to `configureButtonBindings()` in `robotcontainer.py`)

```python3
        # part B: which buttons control the trajectory picker?

        # when the "Y" button is pressed, a command from trajectory picker will run
        yButton = self.driverController.button(wpilib.XboxController.Button.kY)
        yButton.whileTrue(self.trajectoryPicker)

        # or you can make it more fancy -- add a picker for scoring location:
        #  yButton.whileTrue(self.trajectoryPicker.andThen(self.scoringLocationCommandPicker))
        #
        # , but for this to work you need to first add self.scoringLocationCommandPicker = TrajectoryPicker(dashboardName="scoring-location", subsystems=[self.elevator, self.arm, self.intake, self.robotDrive])


        # left and right bumper buttons will toggle between trajectories
        leftBumperButton = self.driverController.button(wpilib.XboxController.Button.kLeftBumper)
        leftBumperButton.onTrue(InstantCommand(self.trajectoryPicker.nextTrajectory))

        rightBumperButton = self.driverController.button(wpilib.XboxController.Button.kRightBumper)
        rightBumperButton.onTrue(InstantCommand(self.trajectoryPicker.previousTrajectory))
```

### Can we use trajectory picker to pick other commands (for example, arm position commands)?

Yes we can, but let's call it "position picker" or something, and this can go to `configureButtonBindings(...)`:

```python3
        from commands.trajectory_picker import TrajectoryPicker
        positionPicker = TrajectoryPicker(None, dashboardName="positionPicker", subsystems=[self.arm, self.elevator])

        # add a few position commands into position picker
        from commands.elevatorcommands import MoveElevatorAndArm
        positionPicker.addCommands("lvl-i", MoveElevatorAndArm(self.elevator, 0.0, self.arm, angle=40))
        positionPicker.addCommands("lvl 0", MoveElevatorAndArm(self.elevator, 0.0, self.arm, angle=80))
        positionPicker.addCommands("lvl 1", MoveElevatorAndArm(self.elevator, 6.0, self.arm, angle=80))

        # when the "A" button is pressed, a command from position picker will run
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.whileTrue(positionPicker)

        # "POV down" / "POV up" button to toggle between previous/next position picker commands
        downPOVButton = self.driverController.povDown()
        downPOVButton.onTrue(InstantCommand(positionPicker.previousTrajectory))
        upPOVButton = self.driverController.povUp()
        upPOVButton.onTrue(InstantCommand(positionPicker.nextTrajectory))
```
