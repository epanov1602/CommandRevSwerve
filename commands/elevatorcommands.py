from __future__ import annotations
import commands2
from commands2 import InstantCommand
from commands2.waitcommand import WaitCommand
from wpilib import SmartDashboard

from subsystems.elevator import Elevator
from subsystems.arm import Arm, ArmConstants


class MoveElevator(commands2.Command):
    def __init__(self, elevator: Elevator, position: float):
        super().__init__()
        self.position = position
        self.elevator = elevator
        self.addRequirements(elevator)

    def initialize(self):
        self.elevator.setPositionGoal(self.position)
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")

    def isFinished(self) -> bool:
        return self.elevator.isDoneMoving()

    def execute(self):
        pass  # nothing to do

    def end(self, interrupted: bool):
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")
            self.elevator.stopAndReset()
        else:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "finished")

    def succeeded(self) -> bool:
        return self.elevator.reachedThisPositionGoal(self.position, 2)


class MoveArm(commands2.Command):
    def __init__(self, arm: Arm, angle: float):
        super().__init__()
        self.angle = angle
        self.arm = arm
        self.addRequirements(arm)
        self.endTime = 0.0

    def initialize(self):
        self.arm.setAngleGoal(self.angle)
        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")

    def isFinished(self) -> bool:
        return self.arm.isDoneMoving()

    def execute(self):
        pass  # nothing to do

    def end(self, interrupted: bool):
        if interrupted:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")
            self.arm.stopAndReset()
        else:
            SmartDashboard.putString("command/c" + self.__class__.__name__, "finished")

    def succeeded(self) -> bool:
        return self.arm.reachedThisAngleGoal(self.angle, 2)


class MoveElevatorAndArm(commands2.SequentialCommandGroup):
    def __init__(self,
                 elevator: Elevator,
                 position: float,
                 arm: Arm,
                 angle: float | None=None,
                 safeTravelAngle=71.4,
                 additionalTimeoutSeconds=0.0):
        self.arm = arm
        self.angle = angle
        if self.angle is None:
            self.angle = safeTravelAngle
        self.elevator = elevator
        self.position = position
        self.finishedImmediately = False
        # 1. sequential command group with three safe steps: [get to safe travel angle, move to new altitude, move arm]
        super().__init__(
            # - first make sure the arm is at safe travel angle
            self.thenLog("1. arm to safeAngle"),
            MoveArm(arm, angle=safeTravelAngle),

            # - move the elevator to the new altitude (but only if the arm reached the angle we wanted)
            (
                self.thenLog("2. moving elevator").andThen(MoveElevator(elevator, position=position))
            ).onlyIf(lambda: arm.reachedThisAngleGoal(safeTravelAngle, 2.0)),

            # - only then go to the desired arm angle (and only if the elevator didn't emergency-stop along the way)
            (
                self.thenLog("3. moving arm").andThen(MoveArm(arm, angle=angle))
            ).onlyIf(lambda: elevator.reachedThisPositionGoal(position, 2.0)),

            # - and wait a split second for things to stabilize
            WaitCommand(seconds=additionalTimeoutSeconds)
        )
        # 2. assert that we have the correct requirements
        assert set(self.requirements) == {elevator, arm}, "supposed to have {elevator, arm} requirements"

    def succeeded(self) -> bool:
        return self.arm.reachedThisAngleGoal(self.angle, 2) and \
            self.elevator.reachedThisPositionGoal(self.position, 2)

    def initialize(self):
        # are we already at the goal?
        self.finishedImmediately = self.succeeded()
        if self.finishedImmediately:
            self.log("finished immediately")
            super().initialize()

    def isFinished(self) -> bool:
        if self.finishedImmediately:
            return True
        return super().isFinished()

    def execute(self):
        if self.finishedImmediately:
            return
        super().execute()

    def end(self, interrupted: bool):
        if self.finishedImmediately:
            return
        if interrupted:
            self.log("interrupted")
        super().end(interrupted)

    def log(self, status):
        SmartDashboard.putString("command/c" + self.__class__.__name__, status)

    def thenLog(self, status):
        return InstantCommand(lambda: self.log(status))
