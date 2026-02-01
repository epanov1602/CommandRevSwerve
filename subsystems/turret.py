# constants right here, to simplify
from commands2 import Subsystem
from rev import SparkBaseConfig, LimitSwitchConfig, SparkBase, SparkMax, SparkFlex, ResetMode, PersistMode, \
    ClosedLoopConfig
from wpilib import SmartDashboard


class TurretConstants:
    # if using relative encoder, how many motor revolutions are needed to move the turret by one Degree?
    GEAR_RATIO = 25
    REVOLUTION = 360
    DEGREE = 1
    GEAR_DIAMETER = 2.0
    TURRET_DIAMETER = 20.0
    motorRevolutionsPerDegree = GEAR_RATIO * (TURRET_DIAMETER * DEGREE) / (GEAR_DIAMETER * REVOLUTION)

    # other settings
    leadMotorInverted = True
    findingZeroSpeed = 0.1

    # calibrating? (at first, set it =True and calibrate all the constants above)
    calibrating = False

    # to calibrate, set calibrating = True and add this in robotcontainer.py __init__(...) function
    # self.turret.setDefaultCommand(
    #    commands2.RunCommand(lambda: self.turret.drive(self.driverController.getRawAxis(XboxController.Axis.kRightY)), self.turret)
    # )

    # which range of motion we want from this turret? (inside what's allowed by limit switches)
    hardStopMinPosition = 30  # Degrees (location where the reverse limit switch is installed)
    minPositionGoal = 45  # Degrees
    maxPositionGoal = 315  # Degrees
    positionTolerance = 2.0  # Degrees

    # PID configuration (after you are done with calibrating=True)
    kP = 0.02  # at first make it very small like this, then start tuning by increasing from there
    kD = 0.0  # at first start from zero, and when you know your kP you can start increasing kD from some small value >0
    kMaxOutput = 1.0

    stallCurrentLimit = 5  # amps, must be integer for Rev


assert TurretConstants.hardStopMinPosition <= TurretConstants.minPositionGoal, "hard stop cant be above lowest position"


class Turret(Subsystem):
    def __init__(
        self,
        leadMotorCANId: int,
        motorClass=SparkFlex,
        limitSwitchType=LimitSwitchConfig.Type.kNormallyClosed,
    ) -> None:
        """
        Constructs an turret.
        Please be very, very careful with setting kP and kD in TurretConstants (it's as dangerous as arms and elevators)
        """
        super().__init__()

        self.zeroFound = False
        self.positionGoal = None

        # initialize the motors and switches
        self.leadMotor = motorClass(
            leadMotorCANId, SparkBase.MotorType.kBrushless
        )
        leadMotorConfig = _getLeadMotorConfig(
            inverted=TurretConstants.leadMotorInverted,
            limitSwitchType=limitSwitchType,
            relPositionFactor=1.0 / TurretConstants.motorRevolutionsPerDegree,
            forwardLimitEnabled=True,
        )
        self.leadMotor.configure(
            leadMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)
        self.reverseLimit = self.leadMotor.getReverseLimitSwitch()
        self.forwardLimit = self.leadMotor.getForwardLimitSwitch()

        # initialize pid controller and encoder(s)
        self.pidController = None
        self.relativeEncoder = self.leadMotor.getEncoder()  # this encoder can be used instead of absolute, if you know!

        # set the initial turret goal to be the minimum
        self.setPositionGoal(TurretConstants.minPositionGoal)


    def notReady(self) -> str:
        if not self.zeroFound:
            return "turret zero not found"
        elif abs(self.positionGoal - self.getPosition()) > TurretConstants.positionTolerance:
            return "turret not at target angle"
        else:
            return ""

    def setPositionGoal(self, goalDegrees: float) -> None:
        if goalDegrees < TurretConstants.minPositionGoal:
            goalDegrees = TurretConstants.minPositionGoal
        if goalDegrees > TurretConstants.maxPositionGoal:
            goalDegrees = TurretConstants.maxPositionGoal
        self.positionGoal = goalDegrees

        if self.pidController is not None:
            self.pidController.setReference(goalDegrees, SparkBase.ControlType.kPosition)

    def getPositionGoal(self) -> float:
        return self.positionGoal

    def getPosition(self) -> float:
        return self.relativeEncoder.getPosition()

    def getVelocity(self) -> float:
        return self.relativeEncoder.getVelocity()

    def stopAndReset(self) -> None:
        self.leadMotor.stopMotor()
        self.leadMotor.clearFaults()

    def drive(self, speed, deadband=0.1, maxSpeedDegreesPerSecond=5):
        # 1. driving is not allowed in these situations
        if not self.zeroFound and not TurretConstants.calibrating:
            return  # if we aren't calibrating, zero must be found first (before we can drive)

        # 2. speed is assumed to be between -1.0 and +1.0, with a deadband
        if abs(speed) < deadband:
            speed = 0
        speed = speed * abs(speed)  # quadratic scaling, easier for humans

        # 3. use the speed to drive
        if self.pidController is None:
            self.leadMotor.set(speed) # if we don't we have a PID controller, we use a speed setpoint
        elif speed != 0: # if we have a PID controller, we control the position goal instead
            self.setPositionGoal(self.positionGoal + speed * maxSpeedDegreesPerSecond / 50.0)  # we have 50 decisions/sec


    def findZero(self):
        # did we find the zero previously?
        if self.zeroFound:
            return
        # are we calibrating the directions?
        if TurretConstants.calibrating:
            return
        # did we find the zero just now?
        if self.reverseLimit.get() and not self.forwardLimit.get():
            self.zeroFound = True
            self.leadMotor.set(0)  # stop! the hard stop reached, the "zero" is found
            self.relativeEncoder.setPosition(TurretConstants.hardStopMinPosition)  # reset the relative encoder
            self.pidController = self.leadMotor.getClosedLoopController()
            self.setPositionGoal(TurretConstants.minPositionGoal)
            return
        # otherwise, continue finding it
        self.leadMotor.set(-TurretConstants.findingZeroSpeed)


    def getState(self) -> str:
        if self.forwardLimit.get():
            return "fwdLimit" if not self.reverseLimit.get() else "both limits (CAN disconn?)"
        if self.reverseLimit.get():
            return "revLimit" if self.forwardLimit is not None else "revLimit or CAN disconn"
        if not self.zeroFound:
            return "finding"
        # otherwise, everything is ok
        return "ok"


    def periodic(self):
        # 1. do we need to find zero?
        if not self.zeroFound:
            self.findZero()
        # 2. report to the dashboard
        SmartDashboard.putString("Turret/state", self.getState())
        SmartDashboard.putNumber("Turret/goal", self.getPositionGoal())
        SmartDashboard.putNumber("Turret/pos", self.getPosition())


def _getLeadMotorConfig(
    inverted: bool,
    limitSwitchType: LimitSwitchConfig.Type,
    relPositionFactor: float,
    forwardLimitEnabled: bool,
) -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(inverted)
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.reverseLimitSwitchEnabled(True)
    config.limitSwitch.reverseLimitSwitchType(limitSwitchType)
    config.limitSwitch.forwardLimitSwitchEnabled(forwardLimitEnabled)
    if forwardLimitEnabled:
        config.limitSwitch.forwardLimitSwitchType(limitSwitchType)
    config.encoder.positionConversionFactor(relPositionFactor)
    config.encoder.velocityConversionFactor(relPositionFactor / 60)  # 60 seconds per minute
    config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    config.closedLoop.pid(TurretConstants.kP, 0.0, TurretConstants.kD)
    config.closedLoop.velocityFF(0.0)
    config.closedLoop.outputRange(-TurretConstants.kMaxOutput, +TurretConstants.kMaxOutput)
    config.smartCurrentLimit(TurretConstants.stallCurrentLimit)
    return config
