from __future__ import annotations

from commands2 import SequentialCommandGroup
from wpilib import SmartDashboard, Field2d
import commands2

class TrajectoryPicker(commands2.Command):
    # see the 2025 map of destinations at field map:
    # https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf

    def __init__(self, fieldDashboard: Field2d | None, subsystems: [], dashboardName="trajectory"):
        self.fieldDashboard = fieldDashboard
        if fieldDashboard is not None:
            fieldDashboard.getObject("traj").setPoses([])
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


    def clearDashboard(self):
        if self.fieldDashboard is not None:
            self.fieldDashboard.getObject("traj").setPoses([])


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
