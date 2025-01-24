## Code examples for adding an elevator (completely untested for now)

**WARNINGS**

* Before you run this first time, check the values of every constant in `ElevatorConstants`

* Before you run this first time, in `ElevatorConstants` set `kP = 0.0001` (you can later increase it by doubling)

* You might need to assign different addresses (CAN IDs), inside the snippet below

**Adding elevator subsystem**

<details>
<summary>This code snippet can go to `subsystems/elevator.py` </summary>

```python

from __future__ import annotations

from rev import CANSparkMax, CANSparkBase, SparkLimitSwitch, SparkAbsoluteEncoder
from wpilib import SmartDashboard
from commands2 import Subsystem


# constants right here, to simplify
class ElevatorConstants:
    # very scary setting! (if set wrong, the arm will escape equilibrium and break something)
    absoluteEncoderInverted = False

    # if using relative encoder, how many motor revolutions are needed to move the elevator by one inch?
    motorRevolutionsPerInch = 3.92

    # if using absolute encoder on output shaft, how many output shaft revolutions are needed to move elevtr by an inch?
    absEncoderRevolutionsPerInch = motorRevolutionsPerInch / 20  # is gear ratio == 20?

    # other settings
    leadMotorInverted = False
    followMotorInverted = True
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

    # PID coefficients
    kP = 0.02  # at first make it very small, then start tuning by increasing from there
    kD = 0.0  # at first start from zero, and when you know your kP you can start increasing kD from some small value >0
    kStaticGain = 0  # make it 3.5?

    # we want velocities in inches per second (not "per minute")
    positionToVelocityFactor = 1.0 / 60

    # Smart Motion parameters, unused now, but just in case we ever decide to use Smart Motion mode
    initialMaxVel = 2000  # rpm
    initialMinVel = -2000  # rpm
    initialMaxAcc = 2500
    initialAllowedError = .02  # was 0.02


class Elevator(Subsystem):
    def __init__(
            self,
            leadMotorCANId: int,
            followMotorCANId: int | None = None,
            presetSwitchPositions: tuple = (),
            useAbsoluteEncoder: bool = False,
            motorClass=CANSparkMax,
            limitSwitchType=SparkLimitSwitch.Type.kNormallyClosed,
    ) -> None:
        """Constructs an elevator. Be very, very careful with setting PIDs -- elevators are dangerous"""
        super().__init__()

        self.zeroFound = False
        self.positionGoal = None
        self.positionGoalSwitchIndex = 0
        self.presetSwitchPositions = presetSwitchPositions

        # initialize the motors and switches
        self.leadMotor = motorClass(leadMotorCANId, CANSparkBase.MotorType.kBrushless)
        self.leadMotor.restoreFactoryDefaults()
        self.forwardLimit = self.leadMotor.getForwardLimitSwitch(limitSwitchType)
        self.reverseLimit = self.leadMotor.getReverseLimitSwitch(limitSwitchType)
        self.followMotor = None
        if followMotorCANId is not None:
            self.followMotor = motorClass(followMotorCANId, CANSparkBase.MotorType.kBrushless)
            self.followMotor.restoreFactoryDefaults()
        self.setMotorDirections()

        # initialize pid controller and encoder(s)
        self.absoluteEncoder = None
        self.relativeEncoder = None
        self.pidController = None
        self.initEncoders(useAbsoluteEncoder)
        if self.zeroFound:
            self.initPidController()

        # set the initial elevator goal
        goal = ElevatorConstants.minPositionGoal
        if self.absoluteEncoder is not None:
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
                                            CANSparkBase.ControlType.kPosition)

    def getPositionGoal(self) -> float:
        return self.positionGoal

    def getPosition(self) -> float:
        if self.absoluteEncoder is not None:
            return self.absoluteEncoder.getPosition()
        else:
            return self.relativeEncoder.getPosition()

    def getAngleVelocity(self) -> float:
        if self.absoluteEncoder is not None:
            return self.absoluteEncoder.getVelocity()
        else:
            return self.relativeEncoder.getVelocity()

    def stopAndReset(self) -> None:
        self.leadMotor.stopMotor()
        if self.followMotor is not None:
            self.followMotor.stopMotor()
        self.setMotorDirections()
        self.leadMotor.clearFaults()
        if self.followMotor is not None:
            self.followMotor.clearFaults()

    def setMotorDirections(self) -> None:
        self.leadMotor.setInverted(ElevatorConstants.leadMotorInverted)
        self.leadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        if self.followMotor is not None:
            self.followMotor.setInverted(ElevatorConstants.leadMotorInverted)  # yes, setting it = "lead motor inverted"
            invert = ElevatorConstants.leadMotorInverted != ElevatorConstants.followMotorInverted
            self.followMotor.follow(self.leadMotor, invert)
            self.followMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)

    def initEncoders(self, useAbsoluteEncoder):
        if useAbsoluteEncoder:
            self.absoluteEncoder = self.leadMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
            self.absoluteEncoder.setPositionConversionFactor(1.0 / ElevatorConstants.absEncoderRevolutionsPerInch)
            self.absoluteEncoder.setVelocityConversionFactor(
                ElevatorConstants.positionToVelocityFactor / ElevatorConstants.absEncoderRevolutionsPerInch)
            self.absoluteEncoder.setInverted(ElevatorConstants.absoluteEncoderInverted)
            self.zeroFound = True  # zero is found by definition, if we are using absolute encoder (knows its zero)
        self.relativeEncoder = self.leadMotor.getEncoder()  # this encoder can be used instead of absolute, if you know!
        self.relativeEncoder.setPositionConversionFactor(1.0 / ElevatorConstants.motorRevolutionsPerInch)
        self.relativeEncoder.setVelocityConversionFactor(
            ElevatorConstants.positionToVelocityFactor / ElevatorConstants.motorRevolutionsPerInch)

    def initPidController(self):
        self.pidController = self.leadMotor.getPIDController()
        self.pidController.setP(ElevatorConstants.kP)
        self.pidController.setD(ElevatorConstants.kD)
        self.pidController.setFF(0)
        self.pidController.setOutputRange(-1, 1)
        self.pidController.setIZone(0)
        self.pidController.setI(0)
        self.pidController.setIMaxAccum(0, 0)  # not playing with integral terms, they can explode
        self.pidController.setIAccum(0)  # not playing with integral terms, they can explode
        if self.absoluteEncoder is not None:
            self.pidController.setFeedbackDevice(self.absoluteEncoder)
        else:
            self.pidController.setFeedbackDevice(self.relativeEncoder)

        smartMotionSlot = 0
        self.pidController.setSmartMotionMaxVelocity(ElevatorConstants.initialMaxVel, smartMotionSlot)
        self.pidController.setSmartMotionMinOutputVelocity(ElevatorConstants.initialMinVel, smartMotionSlot)
        self.pidController.setSmartMotionMaxAccel(ElevatorConstants.initialMaxAcc, smartMotionSlot)
        self.pidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.initialAllowedError, smartMotionSlot)
        self.leadMotor.burnFlash()  # otherwise the "inverted" setting will not survive the brownout
        if self.followMotor is not None:
            self.followMotor.burnFlash()  # otherwise the "inverted" setting will not survive the brownout

    def drive(self, speed):
        self.leadMotor.set(speed)

    def findZero(self):
        # did we find the zero previously?
        if self.zeroFound:
            return
        # did we find the zero just now?
        if self.reverseLimit.get():
            self.zeroFound = True
            self.leadMotor.set(0)  # zero setpoint now
            self.relativeEncoder.setPosition(0.0)  # reset the relative encoder
            self.initPidController()
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
