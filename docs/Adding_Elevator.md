## Code examples for adding an elevator

**READ FIRST**

* Your elevator must have limit switches, ideally normally-closed which is safest (if you don't have them or they are normally-open, you need to change `limitSwitchType=` to normally-open in the `__init__` function in `subsystems/elevator.py` below, but it is not safe to have normally-open or missing limit switches)

* Before you run this first time, check the values of every constant in `ElevatorConstants`

* Before you run this first time, in `ElevatorConstants` set `kP = 0.0001` (you can later increase it by doubling and doubling and doubling again)

* You might need to assign different addresses (CAN IDs), inside the snippet below

* All that said, set `calibrating=True` in `ElevatorConstants` and follow the comments around `calibrating=True` in that code to calibrate all directions

* When `calibrating=True`, open SmartDashboard or Elastic and watch three variables:
  * `elevState` (push limit switches with hand, ensure it's doing what you want)
  * `elevGoal`
  * `elevPosn`
 
* After all above looks good in testing, you can set `calibrating=False`

**Adding elevator subsystem to `subsystems/elevator.py`**

<details>
<summary>(click to expand)</summary>

```python

from __future__ import annotations

from rev import SparkBaseConfig, SparkBase, SparkMax, LimitSwitchConfig, ClosedLoopConfig, SparkLowLevel
from wpilib import SmartDashboard
from commands2 import Subsystem

# constants right here, to simplify
class ElevatorConstants:
    # very scary setting! (if set wrong, the arm will escape equilibrium and break something)
    absoluteEncoderInverted = False

    # if using relative encoder, how many motor revolutions are needed to move the elevator by one inch?
    GEAR_RATIO = 25
    PI = 3.1416
    GEAR_DIAMETER = 2.0
    motorRevolutionsPerInch = GEAR_RATIO / (GEAR_DIAMETER * PI)

    # if using absolute encoder on output shaft, how many output shaft revolutions needed to move elevator by an inch?
    absEncoderRevolutionsPerInch = motorRevolutionsPerInch / GEAR_RATIO  # is gear ratio == 20?

    # other settings
    leadMotorInverted = True
    followMotorInverted = False
    findingZeroSpeed = 0.1

    # calibrating? (at first, set it =True and calibrate all the constants above)
    calibrating = False

    # to calibrate, set calibrating = True and add this in robotcontainer.py __init__(...) function
    # self.elevator.setDefaultCommand(
    #    commands2.RunCommand(lambda: self.elevator.drive(self.driverController.getRightY()), self.elevator)
    # )

    # which range of motion we want from this elevator? (inside what's allowed by limit switches)
    minPositionGoal = 0.5  # inches
    maxPositionGoal = 32  # inches
    positionTolerance = 0.2  # inches

    # if we have an arm, what is the minimum and maximum safe angle for elevator to move
    # (we don't want to move with arm extended unsafely)
    minArmSafeAngleDegrees = 15
    maxArmSafeAngleDegrees = 80

    # PID configuration (after you are done with calibrating=True)
    kP = 0.02  # at first make it very small like this, then start tuning by increasing from there
    kD = 0.0  # at first start from zero, and when you know your kP you can start increasing kD from some small value >0
    kStaticGain = 0  # make it 3.5?
    kMaxOutput = 1.0


class Elevator(Subsystem):
    def __init__(
            self,
            leadMotorCANId: int,
            followMotorCANId: int | None = None,
            presetSwitchPositions: tuple = (),
            useAbsoluteEncoder: bool = False,
            motorClass=SparkMax,
            limitSwitchType=LimitSwitchConfig.Type.kNormallyClosed,
            arm=None,
    ) -> None:
        """
        Constructs an elevator.
        Please be very, very careful with setting kP and kD in ElevatorConstants (elevators are dangerous)
        :param arm: if you want elevator to freeze and not move at times when this arm is in unsafe positions
        """
        super().__init__()

        self.zeroFound = False
        self.positionGoal = None
        self.positionGoalSwitchIndex = 0
        self.presetSwitchPositions = presetSwitchPositions

        # do we have an arm what we must watch for safe angles?
        self.arm = arm
        self.unsafeToMove = ""  # empty string = not unsafe
        self.armUnsafeFreezePositionGoal = None

        # initialize the motors and switches
        self.leadMotor = motorClass(
            leadMotorCANId, SparkBase.MotorType.kBrushless
        )
        leadMotorConfig = _getLeadMotorConfig(
            inverted=ElevatorConstants.leadMotorInverted,
            limitSwitchType=limitSwitchType,
            relPositionFactor=1.0 / ElevatorConstants.motorRevolutionsPerInch,
            absPositionFactor=1.0 / ElevatorConstants.absEncoderRevolutionsPerInch,
            useAbsEncoder=useAbsoluteEncoder
        )
        self.leadMotor.configure(
            leadMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)
        self.forwardLimit = self.leadMotor.getForwardLimitSwitch()
        self.reverseLimit = self.leadMotor.getReverseLimitSwitch()

        if followMotorCANId is not None:
            self.followMotor = motorClass(
                followMotorCANId, SparkBase.MotorType.kBrushless
            )
            followConfig = SparkBaseConfig()
            inverted = ElevatorConstants.leadMotorInverted != ElevatorConstants.followMotorInverted
            followConfig.follow(leadMotorCANId, inverted)
            self.followMotor.configure(
                followConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters,
            )

        # initialize pid controller and encoder(s)
        self.absoluteEncoder = None
        self.pidController = None
        self.relativeEncoder = self.leadMotor.getEncoder()  # this encoder can be used instead of absolute, if you know!
        if useAbsoluteEncoder:
            self.absoluteEncoder = self.leadMotor.getAbsoluteEncoder()
            if not ElevatorConstants.calibrating:
                self.pidController = self.leadMotor.getClosedLoopController()
                self.zeroFound = True  # if using absolute encoder, zero is already found and we can set position goals

        # set the initial elevator goal (if absolute encoder, current position = goal)
        goal = ElevatorConstants.minPositionGoal
        if self.pidController is not None and self.absoluteEncoder is not None:
            goal = self.absoluteEncoder.getPosition()
        self.setPositionGoal(goal)

    def switchDown(self):
        if self.presetSwitchPositions:
            self.positionGoalSwitchIndex = self.positionGoalSwitchIndex - 1
            self.positionGoalSwitchIndex = max([self.positionGoalSwitchIndex, 0])
            self.setPositionGoal(self.presetSwitchPositions[self.positionGoalSwitchIndex])

    def switchUp(self):
        if self.presetSwitchPositions:
            self.positionGoalSwitchIndex = self.positionGoalSwitchIndex + 1
            self.positionGoalSwitchIndex = min([self.positionGoalSwitchIndex, len(self.presetSwitchPositions) - 1])
            self.setPositionGoal(self.presetSwitchPositions[self.positionGoalSwitchIndex])

    def setPositionGoal(self, goalInches: float) -> None:
        if self.unsafeToMove:
            return

        if goalInches < ElevatorConstants.minPositionGoal:
            goalInches = ElevatorConstants.minPositionGoal
        if goalInches > ElevatorConstants.maxPositionGoal:
            goalInches = ElevatorConstants.maxPositionGoal
        self.positionGoal = goalInches

        if self.pidController is not None:
            self.pidController.setReference(goalInches + ElevatorConstants.kStaticGain,
                                            SparkLowLevel.ControlType.kPosition)

    def getPositionGoal(self) -> float:
        return self.positionGoal

    def getPosition(self) -> float:
        if self.absoluteEncoder is not None:
            return self.absoluteEncoder.getPosition()
        else:
            return self.relativeEncoder.getPosition()

    def isDoneMoving(self) -> bool:
        return abs(self.positionGoal - self.getPosition()) <= ElevatorConstants.positionTolerance

    def getVelocity(self) -> float:
        if self.absoluteEncoder is not None:
            return self.absoluteEncoder.getVelocity()
        else:
            return self.relativeEncoder.getVelocity()

    def stopAndReset(self) -> None:
        self.leadMotor.stopMotor()
        if self.followMotor is not None:
            self.followMotor.stopMotor()
        self.leadMotor.clearFaults()
        if self.followMotor is not None:
            self.followMotor.clearFaults()

    def drive(self, speed, deadband=0.1, maxSpeedInchesPerSecond=5):
        # 1. driving is not allowed in these situations
        if not self.zeroFound and not ElevatorConstants.calibrating:
            return  # if we aren't calibrating, zero must be found first (before we can drive)
        if self.unsafeToMove:
            return  # driving is not allowed if arm is at an unsafe angle

        # 2. speed is assumed to be between -1.0 and +1.0, with a deadband
        if abs(speed) < deadband:
            speed = 0
        speed = speed * abs(speed)  # quadratic scaling, easier for humans

        # 3. use the speed to drive
        if self.pidController is None:
            self.leadMotor.set(speed) # if we don't we have a PID controller, we use a speed setpoint
        elif speed != 0: # if we have a PID controller, we control the position goal instead
            self.setPositionGoal(self.positionGoal + speed * maxSpeedInchesPerSecond / 50.0)  # we have 50 decisions/sec


    def findZero(self):
        # did we find the zero previously?
        if self.zeroFound:
            return
        # is it unsafe to move?
        if ElevatorConstants.calibrating or self.unsafeToMove:
            return
        # did we find the zero just now?
        if self.reverseLimit.get() and not self.forwardLimit.get():
            self.zeroFound = True
            self.leadMotor.set(0)  # zero setpoint now
            self.relativeEncoder.setPosition(0.0)  # reset the relative encoder
            self.pidController = self.leadMotor.getClosedLoopController()
            self.setPositionGoal(self.positionGoal)
            return
        # otherwise, continue finding it
        self.leadMotor.set(-ElevatorConstants.findingZeroSpeed)


    def getState(self) -> str:
        if self.unsafeToMove:
            return self.unsafeToMove
        if self.forwardLimit.get():
            return "forward limit" if not self.reverseLimit.get() else "both limits (CAN disconn?)"
        if self.reverseLimit.get():
            return "reverse limit"
        if not self.zeroFound:
            return "finding zero"
        # otherwise, everything is ok
        return "ok"


    def isUnsafeToMove(self):
        if self.arm is not None:
            angle = self.arm.getAngle()
            if angle < ElevatorConstants.minArmSafeAngleDegrees:
                return "arm angle too low"
            if angle > ElevatorConstants.maxArmSafeAngleDegrees:
                return "arm angle too high"
            angleGoal = self.arm.angleGoal
            if angleGoal < ElevatorConstants.minArmSafeAngleDegrees:
                return "arm anglegoal too low"
            if angleGoal > ElevatorConstants.maxArmSafeAngleDegrees:
                return "arm anglegoal too high"


    def periodic(self):
        # 1. do we need to stop the elevator because arm is at unsafe angle?
        unsafeToMove = self.isUnsafeToMove()
        if unsafeToMove and not self.unsafeToMove:
            self.leadMotor.set(0)
            self.setPositionGoal(self.getPosition())
        self.unsafeToMove = unsafeToMove
        # 2. do we need to find zero?
        if not self.zeroFound:
            self.findZero()
        # 3. report to the dashboard
        SmartDashboard.putString("elevState", self.getState())
        SmartDashboard.putNumber("elevGoal", self.getPositionGoal())
        SmartDashboard.putNumber("elevPosn", self.getPosition())


def _getLeadMotorConfig(
    inverted: bool,
    limitSwitchType: LimitSwitchConfig.Type,
    relPositionFactor: float,
    absPositionFactor: float,
    useAbsEncoder: bool,
) -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(inverted)
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(True)
    config.limitSwitch.reverseLimitSwitchEnabled(True)
    config.limitSwitch.forwardLimitSwitchType(limitSwitchType)
    config.limitSwitch.reverseLimitSwitchType(limitSwitchType)
    config.encoder.positionConversionFactor(relPositionFactor)
    config.encoder.velocityConversionFactor(relPositionFactor / 60)  # 60 seconds per minute
    if useAbsEncoder:
        config.absoluteEncoder.positionConversionFactor(absPositionFactor)
        config.absoluteEncoder.velocityConversionFactor(absPositionFactor / 60)  # 60 seconds per minute
        config.absoluteEncoder.inverted(ElevatorConstants.absoluteEncoderInverted)
        config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    else:
        config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    config.closedLoop.pid(ElevatorConstants.kP, 0.0, ElevatorConstants.kD)
    config.closedLoop.velocityFF(0.0)
    config.closedLoop.outputRange(-ElevatorConstants.kMaxOutput, +ElevatorConstants.kMaxOutput)
    return config

```

</details>


**Adding a command to use the elevator (especially in autonomous) to `commands/setelevatorposition.py`**
<details>
<summary>(click to expand the command code)</summary>

```python
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

```

</details>

**Adding elevator to your robot in `__init__(...)` within `robotcontainer.py`**

```python
    def __init__(self) -> None:
        # The robot's subsystems
        from subsystems.elevator import Elevator
        self.elevator = Elevator(leadMotorCANId=9, presetSwitchPositions=(15, 20, 25), motorClass=rev.CANSparkMax)
        
        # or you can do any of these:
        # self.elevator = Elevator(leadMotorCANId=9, followMotorCANId=10, presetSwitchPositions=(15, 20, 25), motorClass=rev.CANSparkMax)
        # self.elevator = Elevator(leadMotorCANId=9, followMotorCANId=10, useAbsoluteEncoder=True, presetSwitchPositions=(15, 20, 25), motorClass=rev.CANSparkMax)
        ...
```

**Adding buttons to control that elevator in `configureButtonBindings()` function**

```python
    def configureButtonBindings(self) -> None:
        ...
        from commands2 import InstantCommand, RunCommand

        # left bumper and right bumper will move elevator between presetSwitchPositions (see above) 
        leftBumper = JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
        leftBumper.onTrue(InstantCommand(self.elevator.switchUp, self.elevator))
        rightBumper = JoystickButton(self.driverController, XboxController.Button.kRightBumper)
        rightBumper.onTrue(InstantCommand(self.elevator.switchDown, self.elevator))

        # the "A" button will request elevator to go to a special position of 33.0 inches
        aButton = JoystickButton(self.driverController, XboxController.Button.kA)
        aButton.onTrue(InstantCommand(lambda: self.elevator.setPositionGoal(33.0), self.elevator))
        ...
```

**Advanced elevator commands (you can put them into `commands/elevatorcommands.py`)**

<details>
<summary>(click to expand)</summary>

```python

from __future__ import annotations
import commands2
from wpilib import Timer

import constants
from subsystems.elevator import Elevator

class MoveElevator(commands2.Command):
    def __init__(self, elevator: Elevator, position: float, additionalTimeoutSeconds=0.0):
        super().__init__()
        self.position = position
        self.elevator = elevator
        self.addRequirements(elevator)
        self.additionalTimeoutSeconds = additionalTimeoutSeconds
        self.endTime = 0.0

    def initialize(self):
        self.endTime = 0.0
        self.elevator.setPositionGoal(self.position)

    def isFinished(self) -> bool:
        return self.endTime != 0 and Timer.getFPGATimestamp() >= self.endTime

    def execute(self):
        if self.endTime == 0.0 and self.elevator.isDoneMoving():
            self.endTime = Timer.getFPGATimestamp() + self.additionalTimeoutSeconds

    def end(self, interrupted: bool):
        pass



class MoveElevatorAndArm(commands2.Command):
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
        self.elevator.setPositionGoal(self.positionGoal)
        self.arm.setAngleGoal(self.angleGoal)

    def isFinished(self) -> bool:
        return self.endTime != 0 and Timer.getFPGATimestamp() >= self.endTime

    def execute(self):
        if self.endTime != 0.0:
            return
        nextAngleGoal = self._safeAngleGoal()
        if self.arm.getAngleGoal() != nextAngleGoal:
            # case 1: must move the arm out of the way
            self.arm.setAngleGoal(nextAngleGoal)
            print(f"MoveElevatorAndArm: next arm angle goal {nextAngleGoal}")
        elif self.elevator.getPositionGoal() != self.positionGoal and not self.elevator.unsafeToMove:
            # case 2: can proceed with moving the elevator further
            self.elevator.setPositionGoal(self.positionGoal)
            print(f"MoveElevatorAndArm: next elevator position goal {self.positionGoal}")
        elif self.elevator.isDoneMoving() and self.arm.isDoneMoving():
            # case 3: both subsystems cannot move further
            if nextAngleGoal != self.angleGoal:
                print(f"WARNING: MoveElevatorAndArm is done, but safe arm angle {nextAngleGoal} is different from angle goal {self.angleGoal}")
            self.endTime = Timer.getFPGATimestamp() + self.additionalTimeoutSeconds

    def end(self, interrupted: bool):
        pass

    def _safeAngleGoal(self, intervals=5):
        assert intervals > 0
        # start from the angle goal and walk backwards towards current elevator position
        # (if that angle goal bumps against limits, adjust it)
        safeAngle = self.angleGoal
        start = self.elevator.getPosition()
        interval = (self.positionGoal - start) / intervals
        positions = [start + i * interval for i in range(intervals)]
        for elevatorPosition in reversed(positions + [self.positionGoal]):
            lowest, highest = constants.safeArmAngleRange(elevatorPosition)
            padding = 0.1 * (highest - lowest)
            if safeAngle < lowest + padding:
                safeAngle = lowest + padding
            if safeAngle > highest - padding:
                safeAngle = highest - padding
        return safeAngle

```
</details>


For the second of the commands to work, you need to add something like this to the end of your `constants.py`:
```python

def safeArmAngleRange(elevatorPosition: float):
    if elevatorPosition < 5:
        return 80, 110
    elif elevatorPosition < 15:
        return 90, 120
    else:
        return 100, 120

```

And you can use such elevator command together with 'gamepiece eject' command from `configureButtonBindings()` this way:
```python

        # the "B" button: make elevator to go to position=20.0 inches, wait until it's there and eject gamepiece
        bButton = JoystickButton(self.driverController, XboxController.Button.kB)

        from commands.elevatorcommands import MoveElevator
        moveElevator = MoveElevator(self.elevator, position=20, additionalTimeoutSeconds=0.5)

        from commands.intakecommands import IntakeFeedGamepieceForward
        ejectGamepiece = IntakeFeedGamepieceForward(self.intake).withTimeout(0.5)

        bButton.onTrue(moveElevator.andThen(ejectGamepiece))

```
