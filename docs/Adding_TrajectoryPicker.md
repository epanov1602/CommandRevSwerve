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
        self.commands.append((name, SequentialCommandGroup(*commands), trajectory))
        self.nameToIndex[name] = index
        if index == 0:
            self.updateDashboard()

    def updateDashboard(self):
        name = "?"
        trajectory = []
        if self.chosenIndex >= 0 and self.chosenIndex < len(self.commands):
            name, command, trajectory = self.commands[self.chosenIndex]
        if self.fieldDashboard is not None:
            self.fieldDashboard.getObject("traj").setPoses(trajectory)
        SmartDashboard.putString(self.dashboardName, name)

    def pickTrajectory(self, name):
        if name not in self.nameToIndex:
            example = f"example: trajectoryPicker.addCommands('{name}', command1, command2, ...)"
            print(f"WARNING: trajectory {name} was never added to this TrajectoryPicker ({example})")
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
            self.running = True

    def initialize(self):
        if self.running is not None:
            self.end(interrupted=True)
        command = None
        if self.chosenIndex >= 0 and self.chosenIndex < len(self.commands):
            name, command, trajectory = self.commands[self.chosenIndex]
        self.running = command
        self.running.initialize()

    def execute(self):
        if self.running is not None:
            self.running.execute()

    def isFinished(self) -> bool:
        if self.running is not None:
            return self.running.isFinished()
        else:
            return False

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

        from commands.jerky_trajectory import JerkyTrajectory

        goToPos1 = JerkyTrajectory(
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
        yButton = JoystickButton(self.driverController, XboxController.Button.kY)
        yButton.whileTrue(self.trajectoryPicker)

        # left and right bumper buttons will toggle between trajectories
        leftBumperButton = JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
        leftBumperButton.onTrue(InstantCommand(self.trajectoryPicker.nextTrajectory))

        rightBumperButton = JoystickButton(self.driverController, XboxController.Button.kRightBumper)
        rightBumperButton.onTrue(InstantCommand(self.trajectoryPicker.previousTrajectory))
```
