from __future__ import annotations
import commands2
from commands2.waitcommand import WaitCommand
from wpilib import Timer

import constants
from subsystems.elevator import Elevator
from subsystems.arm import Arm


class MoveElevator(commands2.Command):
    def __init__(self, elevator: Elevator, position: float):
        super().__init__()
        self.position = position
        self.elevator = elevator
        self.addRequirements(elevator)

    def initialize(self):
        self.elevator.setPositionGoal(self.position)

    def isFinished(self) -> bool:
        return self.elevator.isDoneMoving()

    def execute(self):
        pass  # nothing to do

    def end(self, interrupted: bool):
        if interrupted:
            self.elevator.stopAndReset()


class MoveArm(commands2.Command):
    def __init__(self, arm: Arm, angle: float):
        super().__init__()
        self.angle = angle
        self.arm = arm
        self.addRequirements(arm)
        self.endTime = 0.0

    def initialize(self):
        self.arm.setAngleGoal(self.angle)

    def isFinished(self) -> bool:
        return self.arm.isDoneMoving()

    def execute(self):
        pass  # nothing to do

    def end(self, interrupted: bool):
        if interrupted:
            self.arm.stopAndReset()


class MoveElevatorAndArm(commands2.SequentialCommandGroup):
    def __init__(self,
                 elevator: Elevator,
                 position: float,
                 arm: Arm,
                 angle: float | None,
                 safeTravelAngle=71.4,
                 additionalTimeoutSeconds=0.0):
        # 1. sequential command group with three safe steps: [get to safe travel angle, move to new altitude, move arm]
        super().__init__(
            # - first make sure the arm is at safe travel angle
            MoveArm(arm, angle=safeTravelAngle),

            # - then move the elevator to the new altitude (but only if the arm reached the angle we wanted)
            MoveElevator(elevator, position=position).onlyIf(lambda: arm.reachedThisAngleGoal(safeTravelAngle, 2.0)),

            # - only then go to the desired arm angle (and only if the elevator didn't emergency-stop along the way)
            MoveArm(arm, angle=angle).onlyIf(lambda: elevator.reachedThisPositionGoal(position, 2.0)),

            # - and wait a split second for things to stabilize
            WaitCommand(seconds=additionalTimeoutSeconds)
        )
        # 2. assert that we have the correct requirements
        assert set(self.requirements) == {elevator, arm}, "supposed to have {elevator, arm} requirements"
