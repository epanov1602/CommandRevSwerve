from __future__ import annotations

import commands2

class SetElevatorPosition(commands2.Command):
    def __init__(self, elevator, position, toleranceInches=0.5):
        super().__init__()

        # position must be callable
        self.position = position
        if not callable(self.position):
            self.position = lambda: position
        self.toleranceInches = toleranceInches
        assert toleranceInches > 0, f"given toleranceInches={toleranceInches} is not positive, but should be"

        self.direction = None
        self.elevator = elevator
        self.addRequirements(elevator)

    def initialize(self):
        positionGoal = self.position()
        self.direction = positionGoal - self.elevator.getPosition()
        self.elevator.setPositionGoal(positionGoal)

    def isFinished(self) -> bool:
        distanceToGoal = self.elevator.getPositionGoal() - self.elevator.getPosition()
        if abs(distanceToGoal) < self.toleranceInches:
            return True  # close enough
        if abs(distanceToGoal) < 4 * self.toleranceInches and self.elevator.getVelocity() * self.direction <= 0:
            return True  # kind of close, but looks like moving in the opposite direction already

    def end(self, interrupted: bool):
        pass

    def execute(self):
        pass
