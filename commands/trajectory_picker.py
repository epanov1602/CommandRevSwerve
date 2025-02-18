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
