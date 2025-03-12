import rev
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
    gearReduction = 12 # 12.0
    chainReduction = chainSprocket / driveSprocket
    fudgeFactor = 1  # empirical, if needed
    motorRevolutionsPerDegree = gearReduction * chainReduction / 360 * fudgeFactor

    kArmMinAngle = 35
    kArmMaxAngle = 245

    kAngleTolerance = 1.0  # keep tolerance high for now, to avoid arm stuck in never getting within tolerance from goal

    # PID controller settings
    initialStaticGainTimesP = 0  # we are normally this many degrees off because of static forces
    initialD = 0  # 25e-2 * 0.2
    initialP = 0.0128 * 1.0  # 0.0128 was very strong, 0.05 of that is safe starting value
    initialMaxOutput = 0.3
    initialMinOutput = -0.3
    additionalPMult = 3.0  # unused, but we might want to use it when close to target angle?

    kArmIntakeAngle = 56.0 # spare has 42
    kArmSafeTravelAngle = 73.8  # spare has 71.4
    kArmLevel4ReleaseAngle = 137.4  # spare has 135
    kArmMaxWeightAngle = 86.6  # spare has 84.2 - 90
    kArmAlgaeIntakeAngle = 230  # spare arm does not support this
    # ^^ warning: you are not done if you are changing these constants, look at the function below too

    kArmAlgaeElevatorPosition1 = 14.0
    kArmAlgaeElevatorPosition2 = 18.5


def safeArmAngleRange(elevatorPosition: float):
    offset = 2.4  # spare has zero

    if abs(elevatorPosition - ArmConstants.kArmAlgaeElevatorPosition1) < 3:
        return 65 + offset, 240 + offset
    if abs(elevatorPosition - ArmConstants.kArmAlgaeElevatorPosition2) < 3:
        return 65 + offset, 240 + offset

    if elevatorPosition < 0.5:
        return 32 + offset, 160 + offset
    elif elevatorPosition < 28:
        return 65 + offset, 75 + offset
    else:
        return 35 + offset, 160 + offset

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

        self.defaultLeadMotorConfig = _getLeadMotorConfig(
            limitSwitchType, ArmConstants.kEncoderPositionFactor, ArmConstants.kEncoderInverted,
            ArmConstants.initialP
        )
        self.highGainLeadMotorConfig = _getLeadMotorConfig(
            limitSwitchType, ArmConstants.kEncoderPositionFactor, ArmConstants.kEncoderInverted,
            ArmConstants.initialP * ArmConstants.additionalPMult
        )

        self.leadMotor = SparkMax(leadMotorCANId, SparkMax.MotorType.kBrushless)
        self.leadMotor.configure(
            self.defaultLeadMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        self.forwardLimit = None  # self.leadMotor.getForwardLimitSwitch()
        self.reverseLimit = None  # self.leadMotor.getReverseLimitSwitch()

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
        self.safeAngleRangeFunction = None

        # first angle goal
        assert ArmConstants.kArmSafeTravelAngle <= ArmConstants.kArmMaxAngle
        assert ArmConstants.kArmSafeTravelAngle >= ArmConstants.kArmMinAngle
        self.angleGoal = ArmConstants.kArmSafeTravelAngle
        self.setAngleGoal(self.angleGoal)


    def setSafeAngleRangeFunction(self, safeAngleRangeFunction):
        self.safeAngleRangeFunction = safeAngleRangeFunction


    def isUnsafeToMoveElevator(self):
        if self.safeAngleRangeFunction is not None:
            minSafeAngle, maxSafeAngle = self.safeAngleRangeFunction()
            angle = self.getAngle()
            if angle < minSafeAngle:
                return "arm angle too low"
            if angle > maxSafeAngle:
                return "arm angle too high"
            angleGoal = self.getAngleGoal()
            if angleGoal < minSafeAngle:
                return "arm anglegoal too low"
            if angleGoal > maxSafeAngle:
                return "arm anglegoal too high"


    def periodic(self) -> None:
        SmartDashboard.putNumber(self.armAngleGoalLabel, self.angleGoal)
        SmartDashboard.putNumber(self.armAngleLabel, self.getAngle())
        SmartDashboard.putNumber(self.armPositionLabel, self.relativeEncoder.getPosition())
        SmartDashboard.putString(self.armStateLabel, self.getState())


    def getState(self) -> str:
        forward = self.forwardLimit is not None and self.forwardLimit.get()
        reverse = self.reverseLimit is not None and self.reverseLimit.get()
        if forward:
            return "forward limit" if not reverse else "both limits (CAN disconn?)"
        if reverse:
            return "reverse limit"
        # otherwise, everything is ok
        return "ok"


    def reachedThisAngleGoal(self, goal, toleranceMult=1) -> bool:
        return abs(goal - self.getAngle()) < ArmConstants.kAngleTolerance * toleranceMult


    def isDoneMoving(self) -> bool:
        return self.reachedThisAngleGoal(self.angleGoal)


    def getAngleGoal(self) -> float:
        return self.angleGoal


    def getAngle(self) -> float:
        return self.encoder.getPosition()


    def getAngleVelocity(self) -> float:
        return self.encoder.getVelocity()


    def setAngleGoal(self, angle: float) -> None:
        self.angleGoal = angle
        if self.safeAngleRangeFunction is not None:
            minSafe, maxSafe = self.safeAngleRangeFunction()
            # pad with 5% if we are close to the limits of safe angle range
            padding = min([0.05 * (maxSafe - minSafe), 2 * ArmConstants.kAngleTolerance])
            if padding > 0:
                minSafe += padding
                maxSafe -= padding
            if self.angleGoal < minSafe:
                print(f"WARNING: arm goal {self.angleGoal} clamped to safe range ({minSafe}, {maxSafe}), ={minSafe}")
                self.angleGoal = minSafe
            if self.angleGoal > maxSafe:
                print(f"WARNING: arm goal {self.angleGoal} clamped to safe range ({minSafe}, {maxSafe}), ={maxSafe}")
                self.angleGoal = maxSafe
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
            adjustment = ArmConstants.initialStaticGainTimesP * \
                         Rotation2d.fromDegrees(self.angleGoal - ArmConstants.kArmMaxWeightAngle).cos()
            # ^^ static forces for the arm depend on arm angle
            # (example: if the arm is at the top there are no static forces and pid controller will make arm reach goal
            # , but if the arm is hanging on its side its weight will cause a bias between position goal and reality
            # => classic angle-dependent adjustment or feed-forward gain is needed to correct for this)
            self.pidController.setReference(self.angleGoal + adjustment, SparkBase.ControlType.kPosition)

            # yes, ChatGPT recommends adding a constant feed-forward term proportional to cos(angle - maxWeightAngle)
            # (try asking ChatGPT "how to correct pid controller for static forces in an arm that changes angle")
            # , trouble is rev.ClosedLoopConfig only supports feed-forward terms are proportional angle, not cos(angle)
            # (good news is that mathematically such feed-forward term can also be represented as `adjustment` above)


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
    pGain: float,
) -> SparkBaseConfig:

    config = SparkBaseConfig()
    config.inverted(True)
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(False)  # limit switches are not installed, only angle encoder
    config.limitSwitch.reverseLimitSwitchEnabled(False)  # limit switches are not installed, only angle encoder
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

    config.closedLoop.pid(pGain, 0.0, ArmConstants.initialD)
    config.closedLoop.velocityFF(0.0)  # because position control
    config.closedLoop.outputRange(ArmConstants.initialMinOutput, ArmConstants.initialMaxOutput)

    return config
