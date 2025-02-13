## Code snippet for adding an arm

**WARNINGS**

* Make sure that your absolute encoder is zeroed in such a way that the arm never has to go through zero (or get even close to it) during normal operation

* You can safely verify the direction of angle encoder by checking if `armAngle` and `armPosition` move in same direction on SmartDashboard (when you move that arm by hand, while robot is disabled for safety)

* For safety, use low value of `initialP` and zero `initialD` while verifying the directions, please watch the video

* After you are confident in encoder direction, to tune initialP keep increasing it by factor of no more than 2 until you see oscillations (each robot geometry/mass/gear-ratio needs its own P)

* If you see oscillations at the level of `initialP` that you prefer, you can set nonzero value of `initialD` to dampen them (start `initialD` with a small value and keep doubling it until you see desired effect).

* If you are seeing arm not exactly reaching its angle goal (on SmartDashboard), you need to change `initialStaticGainTimesP`

(if you need a more systematic way to tune P and D without involving I, look up Cohen-Coon or Ziegler-Nichols method on Wikipedia, use the version with I=0)


**This code snippet can go to `subsystems/arm.py`**

```python

from rev import SparkMax, SparkBase, SparkBaseConfig, LimitSwitchConfig, ClosedLoopConfig
from wpimath.geometry import Rotation2d
from commands2 import Subsystem
from wpilib import SmartDashboard

# constants right here, to simplify the video
class ArmConstants:
    # very scary setting! if set wrong, the arm will escape equilibrium and break something
    kEncoderInverted = False

    # one full revolution = 360 units (since we want degree units)
    kEncoderPositionFactor = 360

    # we want speed in degrees per second, not RPM
    kEncoderPositionToVelocityFactor = 1.0 / 60

    # calculating how many motor revolutions are needed to move arm by one degree
    chainSprocket = 60  # teeth
    driveSprocket = 14  # teeth
    gearReduction = 12.0
    chainReduction = chainSprocket / driveSprocket
    fudgeFactor = 1  # empirical, if needed
    motorRevolutionsPerDegree = gearReduction * chainReduction / 360 * fudgeFactor

    kArmMinAngle = 15
    kArmMaxAngle = 130
    kArmMaxWeightAngle = 30
    kAngleTolerance = 2.0

    # PID coefficients
    initialStaticGainTimesP = 3.5  # we are normally this many degrees off because of static forces
    initialD = 25e-2 * 0.2
    initialP = 0.8e-2 * 1.0
    additionalPMult = 3.0  # when close to target angle

    initialMaxOutput = 1
    initialMinOutput = -1
    initialMaxRPM = 5700

    # Smart Motion Coefficients, but maybe they apply outside of SmartMotion too?
    initialMaxVel = 2000  # rpm
    initialMinVel = -2000  # rpm
    initialMaxAcc = 2500
    initialAllowedError = .02  # was 0.02

    # Hacks
    kAngleGoalRadius = 10
    kExtraDelayForOscillationsToStop = 0.1  # seconds (until the PID coefficients below are tuned to avoid oscillations)


class Arm(Subsystem):
    def __init__(
        self,
        leadMotorCANId: int,
        followMotorCANId: int,
        dontSlam: bool=False,
        limitSwitchType=LimitSwitchConfig.Type.kNormallyOpen,  # make NormallyOpen if you don't have limit switches yet
        dashboardPrefix="",  # set it to "front_" if this is a front arm in a multi-arm robot
    ) -> None:
        """Constructs an arm. Be very very careful with setting PIDs -- arms are dangerous"""
        super().__init__()
        self.dontSlam = dontSlam
        self.armAngleLabel = dashboardPrefix + "armAngle"
        self.armAngleGoalLabel = dashboardPrefix + "armAngleGoal"
        self.armPositionLabel = dashboardPrefix + "armPosition"
        self.armStateLabel = dashboardPrefix + "armState"

        self.leadMotor = SparkMax(leadMotorCANId, SparkMax.MotorType.kBrushless)
        self.leadMotor.configure(
            _getLeadMotorConfig(limitSwitchType, ArmConstants.kEncoderPositionFactor, ArmConstants.kEncoderInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        self.forwardLimit = self.leadMotor.getForwardLimitSwitch()
        self.reverseLimit = self.leadMotor.getReverseLimitSwitch()

        self.followMotor = None
        if followMotorCANId is not None:
            self.followMotor = SparkMax(followMotorCANId, SparkMax.MotorType.kBrushless)
            followConfig = SparkBaseConfig()
            followConfig.follow(leadMotorCANId, invert=True)
            self.followMotor.configure(
                followConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)

        # now initialize pid controller and encoder
        self.pidController = self.leadMotor.getClosedLoopController()
        self.encoder = self.leadMotor.getAbsoluteEncoder()
        self.relativeEncoder = self.leadMotor.getEncoder()

        # first angle goal
        self.angleGoal = ArmConstants.kArmMinAngle
        self.setAngleGoal(self.angleGoal)


    def periodic(self) -> None:
        SmartDashboard.putNumber(self.armAngleGoalLabel, self.angleGoal)
        SmartDashboard.putNumber(self.armAngleLabel, self.getAngle())
        SmartDashboard.putNumber(self.armPositionLabel, self.relativeEncoder.getPosition())
        SmartDashboard.putString(self.armStateLabel, self.getState())


    def getState(self) -> str:
        if self.forwardLimit.get():
            return "forward limit" if not self.reverseLimit.get() else "both limits (CAN disconn?)"
        if self.reverseLimit.get():
            return "reverse limit"
        # otherwise, everything is ok
        return "ok"


    def isDoneMoving(self) -> bool:
        return abs(self.getAngleGoal() - self.getAngle()) < ArmConstants.kAngleTolerance


    def getAngleGoal(self) -> float:
        return self.angleGoal


    def getAngle(self) -> float:
        return self.encoder.getPosition()


    def getAngleVelocity(self) -> float:
        return self.encoder.getVelocity()


    def setAngleGoal(self, angle: float) -> None:
        self.angleGoal = angle
        if self.angleGoal < ArmConstants.kArmMinAngle:
            self.angleGoal = ArmConstants.kArmMinAngle
        if self.angleGoal > ArmConstants.kArmMaxAngle:
            self.angleGoal = ArmConstants.kArmMaxAngle

        if self.dontSlam and self.angleGoal <= ArmConstants.kArmMinAngle:
            # we don't want to slam the arm on the floor, but the target angle is pretty low
            if self.getAngle() > ArmConstants.kArmMinAngle:
                self.pidController.setReference(ArmConstants.kArmMinAngle, SparkBase.ControlType.kPosition)
            else:
                self.stopAndReset()
        else:
            # static forces for the arm depend on arm angle (e.g. if it's at the top, no static forces)
            adjustment = ArmConstants.initialStaticGainTimesP * \
                         Rotation2d.fromDegrees(self.angleGoal - ArmConstants.kArmMinAngle).cos()
            self.pidController.setReference(self.angleGoal + adjustment, SparkBase.ControlType.kPosition)


    def stopAndReset(self) -> None:
        self.leadMotor.stopMotor()
        if self.followMotor is not None:
            self.followMotor.stopMotor()
        self.leadMotor.clearFaults()
        if self.followMotor is not None:
            self.followMotor.clearFaults()


def _getLeadMotorConfig(
    limitSwitchType: LimitSwitchConfig.Type,
    absPositionFactor: float,
    absEncoderInverted: bool,
) -> SparkBaseConfig:

    config = SparkBaseConfig()
    config.inverted(True)
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(True)
    config.limitSwitch.reverseLimitSwitchEnabled(True)
    config.limitSwitch.forwardLimitSwitchType(limitSwitchType)
    config.limitSwitch.reverseLimitSwitchType(limitSwitchType)

    relPositionFactor = 1.0  # can also make it = 1.0 / ArmConstants.motorRevolutionsPerDegree
    config.encoder.positionConversionFactor(relPositionFactor)
    config.encoder.velocityConversionFactor(relPositionFactor / 60)  # 60 seconds per minute

    config.absoluteEncoder.positionConversionFactor(absPositionFactor)
    config.absoluteEncoder.velocityConversionFactor(absPositionFactor / 60)  # 60 seconds per minute
    config.absoluteEncoder.inverted(absEncoderInverted)
    config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

    assert ArmConstants.kArmMaxAngle > ArmConstants.kArmMinAngle, (
        f"ArmConstants.kArmMaxAngle={ArmConstants.kArmMaxAngle} is not greater than " +
        f"ArmConstants.kArmMinAngle={ArmConstants.kArmMinAngle}"
    )
    config.softLimit.reverseSoftLimit(ArmConstants.kArmMinAngle)
    config.softLimit.forwardSoftLimit(ArmConstants.kArmMaxAngle)

    config.closedLoop.pid(ArmConstants.initialP, 0.0, ArmConstants.initialD)
    config.closedLoop.velocityFF(0.0)  # because position control
    config.closedLoop.outputRange(ArmConstants.initialMinOutput, ArmConstants.initialMaxOutput)

    return config

```

**This code snippet can go inside of the function `configureButtonBindings` in your `robotcontainer.py`, so you can drive the arm with the joystick:**
```python
    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        ## start of arm joystick control code
        from commands2.button import JoystickButton
        from commands2 import RunCommand

        aButton = JoystickButton(self.driverController, wpilib.XboxController.Button.kA)
        aButton.onTrue(commands2.InstantCommand(lambda: self.arm.setAngleGoal(ArmConstants.kArmMinAngle)))

        yButton = JoystickButton(self.driverController, wpilib.XboxController.Button.kY)
        yButton.onTrue(commands2.InstantCommand(lambda: self.arm.setAngleGoal(70)))
        ## end of arm joystick control code
```

**This code adds one arm to __init__() function in `robotcontainer.py`**
```python
    def __init__(self) -> None:
        self.arm = Arm(CANIds.kArmMotorRight, CANIds.kArmMotorLeft, True)
        # ... then the rest of the function
```

**This code goes to the top of `robotcontainer.py`, so robotcontainer knows what Arm is**
```python
from subsystems.arm import Arm, ArmConstants
```
