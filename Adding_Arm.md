## Code snippet for adding an arm

*WARNINGS*

* YOU DEFINITELY NEED TO CHECK THE SENSOR/MOTOR DIRECTIONS at low values of initialP and zero initialD, please watch the video

* YOU DEFINITELY NEED TO TUNE THE initialP and initialD constants yourself, for your own robot geometry mass and gear ratio

* You might need to assign different addresses (CAN IDs), inside the snippet below


**This code snippet can go to `subsystems/arm.py`**
```python
from rev import CANSparkMax, CANSparkBase, SparkLimitSwitch, SparkAbsoluteEncoder
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

    kArmAngleToEjectIntoAmp = 106  # start ejecting note into amp from this angle
    kArmAngleToPushIntoAmp = 79  # after ejecting note into, drop the arm to this angle to push the note in
    kArmAgleToSaveEnergy = 75  # increase after we use both absolute and relative encoders
    kArmAngleToShootDefault = 55
    kArmMinAngle = 15
    kArmMaxAngle = 130

    # PID coefficients
    initialStaticGainTimesP = 3.5  # we are normally this many degrees off because of static forces
    initialD = 25e-2 * 0.2
    initialP = 2e-2 * 1.0
    additionalPMult = 3.0  # when close to target angle

    # 8 * 700 = 2800
    initialFF = 0
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
        self, leadMotorCANId: int, followMotorCANId: int, dontSlam: bool
    ) -> None:
        """Constructs an arm. Be very very careful with setting PIDs -- arms are dangerous"""
        super().__init__()
        self.dontSlam = dontSlam

        self.leadMotor = CANSparkMax(leadMotorCANId, CANSparkMax.MotorType.kBrushless)
        self.leadMotor.restoreFactoryDefaults()

        self.forwardLimit = self.leadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
        self.reverseLimit = self.leadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)

        self.followMotor = CANSparkMax(followMotorCANId, CANSparkMax.MotorType.kBrushless)
        self.followMotor.restoreFactoryDefaults()

        self.setMotorDirections()

        # now initialize pid controller and encoder
        self.pidController = self.leadMotor.getPIDController()
        self.encoder = self.leadMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

        self.relativeEncoder = self.leadMotor.getEncoder()  # this encoder can be used instead of absolute, if you know!
        self.encoder.setPositionConversionFactor(ArmConstants.kEncoderPositionFactor)
        self.encoder.setVelocityConversionFactor(
            ArmConstants.kEncoderPositionFactor * ArmConstants.kEncoderPositionToVelocityFactor)
        self.encoder.setInverted(ArmConstants.kEncoderInverted)

        self.relativeEncoder.setPositionConversionFactor(1 / ArmConstants.motorRevolutionsPerDegree)
        self.relativeEncoder.setVelocityConversionFactor(
            ArmConstants.kEncoderPositionToVelocityFactor / ArmConstants.motorRevolutionsPerDegree)

        self.pidController.setP(ArmConstants.initialP)
        self.pidController.setD(ArmConstants.initialD)
        self.pidController.setFF(ArmConstants.initialFF)
        self.pidController.setOutputRange(ArmConstants.initialMinOutput, ArmConstants.initialMaxOutput)
        self.pidController.setIZone(0)
        self.pidController.setI(0)

        self.pidController.setIMaxAccum(0, 0)  # not playing with integral terms, they can explode
        self.pidController.setIAccum(0)  # not playing with integral terms, they can explode

        smartMotionSlot = 0
        self.pidController.setFeedbackDevice(self.encoder)
        self.pidController.setSmartMotionMaxVelocity(ArmConstants.initialMaxVel, smartMotionSlot)
        self.pidController.setSmartMotionMinOutputVelocity(ArmConstants.initialMinVel, smartMotionSlot)
        self.pidController.setSmartMotionMaxAccel(ArmConstants.initialMaxAcc, smartMotionSlot)
        self.pidController.setSmartMotionAllowedClosedLoopError(ArmConstants.initialAllowedError, smartMotionSlot)

        self.leadMotor.burnFlash()  # otherwise the "inverted" setting will not survive the brownout
        self.followMotor.burnFlash()  # otherwise the "inverted" setting will not survive the brownout

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
                self.pidController.setP(ArmConstants.initialP * 0.4)
                self.pidController.setReference(ArmConstants.kArmMinAngle, CANSparkMax.ControlType.kPosition)
            else:
                self.stopAndReset()
        else:
            self.pidController.setP(ArmConstants.initialP * 0.4)
            # regular case: use adjustment for static forces
            # (to mimic what static gain would do, because SparkMax doesn't support static gain directly)
            adjustment = ArmConstants.initialStaticGainTimesP * \
                         Rotation2d.fromDegrees(self.angleGoal - ArmConstants.kArmMinAngle).cos() / \
                         self.getAngleDependentPFudgeFactor(self.angleGoal)
            self.pidController.setReference(self.angleGoal + adjustment, CANSparkMax.ControlType.kPosition)


    def stopAndReset(self) -> None:
        self.leadMotor.stopMotor()
        self.followMotor.stopMotor()
        self.setMotorDirections()
        self.leadMotor.clearFaults()
        self.followMotor.clearFaults()


    def getAngleDependentPFudgeFactor(self, angleDegrees: float) -> float:
        """
        Big hack: multiplier on top of the P gain, depending on angle
        :param angleDegrees: angle to be used for that fudge factor
        :return:
        """
        error = (angleDegrees - self.angleGoal) / ArmConstants.kAngleGoalRadius
        if error > 1:
          return 0.4
        if max((self.angleGoal, angleDegrees)) > ArmConstants.kArmMinAngle + 90:
          return 0.5
        altitude = Rotation2d.fromDegrees(angleDegrees - ArmConstants.kArmMinAngle).sin()
        additional = 3.0 * (1 - altitude * altitude) * max((0, 1 - abs(error)))
        return 1 + additional

    def setMotorDirections(self) -> None:
        self.leadMotor.setInverted(True)
        self.leadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.followMotor.follow(self.leadMotor, True)
        self.followMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
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

        aButton = JoystickButton(self.driverController, wpilib.XboxController.Button.kY)
        aButton.onTrue(commands2.InstantCommand(lambda: self.arm.setAngleGoal(ArmConstants.kArmMinAngle)))

        yButton = JoystickButton(self.driverController, wpilib.XboxController.Button.kA)
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
