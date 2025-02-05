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
<summary>(click to expand the subsystem code) </summary>

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
    motorRevolutionsPerInch = 3.92

    # if using absolute encoder on output shaft, how many output shaft revolutions needed to move elevator by an inch?
    absEncoderRevolutionsPerInch = motorRevolutionsPerInch / 20  # is gear ratio == 20?

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
    minPositionGoal = 15  # inches
    maxPositionGoal = 70  # inches

    # PID configuration (only after you are done with calibrating=True)
    kP = 0.02  # at first make it very small, then start tuning by increasing from there
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
    ) -> None:
        """Constructs an elevator. Be very, very careful with setting PIDs -- elevators are dangerous"""
        super().__init__()

        self.zeroFound = False
        self.positionGoal = None
        self.positionGoalSwitchIndex = 0
        self.presetSwitchPositions = presetSwitchPositions

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
        self.followMotor = None

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
        # if we aren't calibrating, zero must be found first (before we can drive)
        if not self.zeroFound and not ElevatorConstants.calibrating:
            return
        # speed is assumed to be between -1.0 and +1.0
        if abs(speed) < deadband:
            speed = 0
        speed = speed * abs(speed)  # quadratic scaling, easier for humans
        if self.pidController is None:
            self.leadMotor.set(speed)
            return
        # we are here if we have a pid controller
        if speed != 0:
            self.setPositionGoal(self.positionGoal + speed * maxSpeedInchesPerSecond / 50.0)  # we have 50 decisions/sec

    def findZero(self):
        # did we find the zero previously?
        if self.zeroFound:
            return
        # did we find the zero just now?
        if self.reverseLimit.get():
            self.zeroFound = True
            self.leadMotor.set(0)  # zero setpoint now
            self.relativeEncoder.setPosition(0.0)  # reset the relative encoder
            self.pidController = self.leadMotor.getClosedLoopController()
            self.setPositionGoal(self.positionGoal)
            return
        # otherwise, continue finding it
        self.leadMotor.set(-ElevatorConstants.findingZeroSpeed)

    def getState(self) -> str:
        if self.forwardLimit.get():
            return "forward limit" if not self.reverseLimit.get() else "both limits"
        elif self.reverseLimit.get():
            return "reverse limit"
        elif not self.zeroFound:
            return "finding zero"
        else:
            return "ok"

    def periodic(self):
        if not self.zeroFound and not ElevatorConstants.calibrating:
            self.findZero()
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
        self.elevator = Elevator(leadMotorCANId=9, presetSwitchPositions=(10, 20, 70), motorClass=rev.CANSparkMax)
        
        # or you can do any of these:
        # self.elevator = Elevator(leadMotorCANId=9, followMotorCANId=10, presetSwitchPositions=(10, 20, 70), motorClass=rev.CANSparkMax)
        # self.elevator = Elevator(leadMotorCANId=9, followMotorCANId=10, useAbsoluteEncoder=True, presetSwitchPositions=(10, 20, 70), motorClass=rev.CANSparkMax)
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
