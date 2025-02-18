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
            # - then move the elevator to the new altitude
            MoveElevator(elevator, position=position),
            # - and only then go to the desired arm angle and wait a split second for things to stabilize
            MoveArm(arm, angle=angle),
            WaitCommand(seconds=additionalTimeoutSeconds)
        )
        # 2. assert that we have the correct requirements
        assert set(self.requirements) == {elevator, arm}, "supposed to have {elevator, arm} requirements"


class MoveElevatorAndArmV1(commands2.Command):
    def __init__(self, elevator: Elevator, position: float, arm: Arm, angle: float, additionalTimeoutSeconds=0.0):
        super().__init__()
        # elevator stuff
        self.positionGoal = position
        self.elevator = elevator
        self.addRequirements(elevator)
        # arm stuff
        self.angleGoal = angle
        self.arm = arm
        self.addRequirements(arm)
        # additional timeout at the end
        self.additionalTimeoutSeconds = additionalTimeoutSeconds
        self.endTime = 0.0

    def initialize(self):
        self.endTime = 0.0

    def isFinished(self) -> bool:
        return self.endTime != 0 and Timer.getFPGATimestamp() >= self.endTime

    def execute(self):
        if self.endTime != 0.0:
            return
        nextAngleGoal, nextPositionGoal = self._safeAngleGoal()

        if self.arm.getAngleGoal() != nextAngleGoal:
            # case 1: must move the arm out of the way
            self.arm.setAngleGoal(nextAngleGoal)
            print(f"MoveElevatorAndArm: next arm angle goal {nextAngleGoal} (for elevator position {nextPositionGoal})")
        elif not self.arm.isDoneMoving():
            pass  # and wait for arm to finish the move

        elif self.elevator.getPositionGoal() != nextPositionGoal and not self.elevator.unsafeToMove:
            # case 2: can proceed with moving the elevator further
            self.elevator.setPositionGoal(self.positionGoal)
            print(f"MoveElevatorAndArm: next elevator position goal {self.positionGoal} (for angle goal {nextAngleGoal})")
        elif not self.elevator.isDoneMoving():
            pass  # and wait for that elevator to finish the move

        else:
            # case 3: both subsystems cannot move further, whether they reached their real goals or not
            if nextAngleGoal != self.angleGoal:
                print(f"WARNING: MoveElevatorAndArm is done, but safe arm angle {nextAngleGoal} is different from angle goal {self.angleGoal}")
            if nextPositionGoal != self.positionGoal:
                print(f"WARNING: MoveElevatorAndArm is done, but safe elevator position {nextPositionGoal} is different from its goal {self.positionGoal}")
            self.endTime = Timer.getFPGATimestamp() + self.additionalTimeoutSeconds

    def end(self, interrupted: bool):
        if interrupted:
            self.elevator.stopAndReset()
            self.arm.stopAndReset()

    def _safeAngleGoal(self, intervals=5):
        assert intervals > 0
        # start from the angle goal and walk backwards towards current elevator position
        # (if that angle goal bumps against limits, adjust it)
        safeAngle = self.angleGoal
        safePosition = self.positionGoal
        start = self.elevator.getPosition()
        interval = (self.positionGoal - start) / intervals
        positions = [start + i * interval for i in range(intervals)]
        for elevatorPosition in reversed(positions + [self.positionGoal]):
            lowest, highest = constants.safeArmAngleRange(elevatorPosition)
            padding = 0.05 * (highest - lowest)
            if safeAngle < lowest + padding:
                safeAngle = lowest + padding
                safePosition = elevatorPosition
            if safeAngle > highest - padding:
                safeAngle = highest - padding
                safePosition = elevatorPosition
        return safeAngle, safePosition
