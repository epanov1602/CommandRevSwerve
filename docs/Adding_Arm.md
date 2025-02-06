## Code snippet for adding an arm

**WARNINGS**

* YOU DEFINITELY NEED TO CHECK THE SENSOR/MOTOR DIRECTIONS at low values of initialP and zero initialD, please watch the video

* YOU DEFINITELY NEED TO TUNE THE initialP and initialD constants yourself, for your own robot geometry mass and gear ratio

* You might need to assign different addresses (CAN IDs), inside the snippet below


**This code snippet can go to `subsystems/arm.py`**

```python
from rev import SparkMax, SparkBase, SparkBaseConfig, LimitSwitchConfig, ClosedLoopConfig
from wpimath.geometry import Rotation2d
from commands2 import Subsystem

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
    ) -> None:
        """Constructs an arm. Be very very careful with setting PIDs -- arms are dangerous"""
        super().__init__()
        self.dontSlam = dontSlam

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

        # first angle goal
        self.angleGoal = ArmConstants.kArmMinAngle
        self.setAngleGoal(self.angleGoal)


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

    config.absoluteEncoder.positionConversionFactor(absPositionFactor)
    config.absoluteEncoder.velocityConversionFactor(absPositionFactor / 60)  # 60 seconds per minute
    config.absoluteEncoder.inverted(absEncoderInverted)
    config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

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
