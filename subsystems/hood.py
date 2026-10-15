# constants right here, to simplify
import math

from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration, Slot0Configs, CurrentLimitsConfigs
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, InvertedValue
from rev import SparkBaseConfig, LimitSwitchConfig, SparkBase, SparkMax, ResetMode, PersistMode, ClosedLoopConfig, \
    FeedbackSensor, SparkRelativeEncoder
from wpilib import SmartDashboard, RobotState
from wpimath.filter import SlewRateLimiter


class Constants:
    # other settings
    motorInverted = False
    findingZeroSpeed = 0.06  # for Rev please use 0.14
    stallCurrentLimit = 12  # amps (must be an integer for Rev)
    findingZeroCurrentLimit = stallCurrentLimit * 0.7

    # calibrating? (at first, set it =True and calibrate all the constants above)
    calibrating = False

    # to calibrate, set calibrating = True above, and add this at the end of configureButtonBindings(...) in robotcontainer.py
    # self.driverController.button(XboxController.Button.kA).whileTrue(
    #     # will this make the hood go towards its zero, until it hits it and hits max current?
    #     RunCommand(lambda: self.hood.drive(speed=-0.1), self.hood)
    # ).onFalse(
    #     InstantCommand(lambda: self.hood.stopAndReset(), self.hood)
    # )
    #
    # after this is added, add "Hood" in Elastic, and see what kind of Hood/current is observed
    # when hitting the hard limit at speed -0.1 or you chosen speed
    # (set findingZeroCurrentLimit to half of that value, set calibrating=False and your hood is ready)

    # which range of motion we want from this hood?
    minPosition = -0.55  # motor revolutions
    maxPosition = -0.01  # motor revolutions
    initialPositionGoal = 0.1 * (minPosition - maxPosition) + maxPosition   # closest to zero (out of the two)
    positionTolerance = 0.01  # motor revolutions

    # PID configuration (after you are done with calibrating=True)
    kP = 6.0  # at first make it very small like this, then start tuning by increasing from there
    kD = 0.0  # at first start from zero, and when you know your kP you can start increasing kD from some small value >0
    kMaxOutput = 1.0


class Hood(Subsystem):
    def __init__(
        self,
        leadMotorCANId: int,
        motorClass=SparkMax,
    ) -> None:
        """
        Constructs a hood.
        Please be very, very careful with setting kP and kD in Constants (it's as dangerous as arms and elevators)
        """
        super().__init__()

        self.zeroFound = False
        self.positionGoal = None

        self.revPidController = None
        self.revMotor: SparkBase | None = None
        self.revEncoder: SparkRelativeEncoder | None = None

        self.talonMotor: TalonFX | None = None
        self.talonPositionRequest = PositionVoltage(0, slot=0)

        # initialize the motors and switches
        if motorClass == TalonFX:
            self.talonMotor = TalonFX(leadMotorCANId)
            _configureTalonMotor(self.talonMotor)
            self.talonMotor.set_position(0.0)
            self.talonMotor.clear_sticky_faults()
        else:
            cfg = _revMotorConfig(relPositionFactor=1.0)
            self.revMotor = motorClass(leadMotorCANId, SparkBase.MotorType.kBrushless)
            self.revMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            self.revEncoder = self.revMotor.getEncoder()  # this encoder can be used instead of absolute, if you know!
            self.revMotor.clearFaults()

        # initialize pid controller and encoder(s)

        # the logic of finding the zero needs to be a little smooth
        assert Constants.findingZeroSpeed > 0
        self.findingZeroRateLimiter = SlewRateLimiter(rateLimit=1.0 * Constants.findingZeroSpeed)
        self.findingZeroRateLimiter.reset(0.0)

        # set the initial hood goal to be the minimum
        self.setPositionGoal(Constants.initialPositionGoal)


    def forgetZero(self):
        self.zeroFound = False
        self.revPidController = None
        self.stopAndReset()


    def notReady(self) -> str:
        if not self.zeroFound:
            return "hood zero not found"
        elif abs(self.positionGoal - self.getPosition()) > Constants.positionTolerance:
            return "hood not at target angle"
        else:
            return ""


    def set(self, goal: float):
        self.setPositionGoal(goal)


    def setPositionGoal(self, goal: float) -> None:
        if goal < Constants.minPosition:
            goal = Constants.minPosition
        if goal > Constants.maxPosition:
            goal = Constants.maxPosition
        self.positionGoal = goal
        if not self.zeroFound:
            return

        if self.revPidController:
            self.revPidController.setReference(goal, SparkBase.ControlType.kPosition)
        else:
            self.talonMotor.set_control(self.talonPositionRequest.with_position(goal))


    def getPositionGoal(self) -> float:
        return self.positionGoal


    def getPosition(self) -> float:
        if self.revEncoder is not None:
            return self.revEncoder.getPosition()
        elif self.talonMotor is not None:
            return self.talonMotor.get_position().value
        else:
            return float('nan')


    def getVelocity(self) -> float:
        if self.revEncoder is not None:
            return self.revEncoder.getVelocity()
        elif self.talonMotor is not None:
            return self.talonMotor.get_velocity().value
        else:
            return float('nan')


    def drive(self, speed, deadband=0.05, maxSpeedRPS=5.0):
        print(f"Hood:drive(entering, speed={speed})")
        # 1. driving is not allowed in these situations
        if not self.zeroFound and not Constants.calibrating:
            return  # if we aren't calibrating, zero must be found first (before we can drive)

        # 2. speed is assumed to be between -1.0 and +1.0, with a deadband
        if abs(speed) < deadband:
            speed = 0

        # 3. use the speed to drive
        if self.zeroFound:
            self.setPositionGoal(self.positionGoal + speed * maxSpeedRPS / 50.0)  # we have 50 decisions/sec
        elif self.revMotor is not None:
            self.revMotor.set(speed)  # if we don't we have PID control, we use a speed setpoint
        else:
            print(f"Hood:drive(speed={speed})")
            self.talonMotor.set(speed)


    def stopAndReset(self) -> None:
        self.findingZeroRateLimiter.reset(0.0)
        if self.revMotor is not None:
            self.revMotor.stopMotor()
            self.revMotor.clearFaults()
        if self.talonMotor is not None:
            self.talonMotor.stopMotor()
            self.talonMotor.clear_sticky_faults()


    def findZero(self):
        # did we find the zero previously?
        if self.zeroFound:
            return
        # are we calibrating the directions?
        if Constants.calibrating:
            return

        # did we find the zero just now?
        spike = False
        if self.revMotor is not None and self.revMotor.getOutputCurrent() > Constants.findingZeroCurrentLimit:
            spike = True
        if self.talonMotor is not None and self.talonMotor.get_stator_current().value > Constants.findingZeroCurrentLimit:
            spike = True

        if spike:
            self.zeroFound = True
            self.stopAndReset()
            if self.revMotor is not None:
                self.revEncoder.setPosition(0.0)  # found the zero position
                self.revPidController = self.revMotor.getClosedLoopController()
            if self.talonMotor is not None:
                self.talonMotor.set_position(0.0)
            self.setPositionGoal(self.positionGoal)
            return

        # otherwise, continue finding it
        if RobotState.isEnabled():
            speed = self.findingZeroRateLimiter.calculate(Constants.findingZeroSpeed)
            SmartDashboard.putNumber("Hood/findingSpeed", speed)
            motor = self.revMotor or self.talonMotor
            motor.set(math.copysign(speed, -Constants.minPosition))
            # ^^ if minPosition is negative, positive speed is needed for homing (and vice versa)
        else:
            self.findingZeroRateLimiter.reset(0.0)
            SmartDashboard.putNumber("Hood/findingSpeed", 0.0)
            motor = self.revMotor or self.talonMotor
            motor.set(0)


    def getState(self) -> str:
        if not self.zeroFound:
            return "finding"
        # otherwise, everything is ok
        return "ok"


    def periodic(self):
        # 1. do we need to find zero?
        if not self.zeroFound:
            self.findZero()
        # 2. report to the dashboard
        SmartDashboard.putString("Hood/state", self.getState())
        if self.revMotor is not None:
            SmartDashboard.putNumber("Hood/current", self.revMotor.getOutputCurrent())
        elif self.talonMotor is not None:
            SmartDashboard.putNumber("Hood/current", (
                    self.talonMotor.get_stator_current().value + self.talonMotor.get_supply_current().value
            ))
        SmartDashboard.putNumber("Hood/goal", self.getPositionGoal())
        SmartDashboard.putNumber("Hood/pos", self.getPosition())


def _revMotorConfig(relPositionFactor: float = 1.0) -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.encoder.positionConversionFactor(relPositionFactor)
    config.encoder.velocityConversionFactor(relPositionFactor / 60)  # 60 seconds per minute
    config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    config.closedLoop.pid(Constants.kP, 0.0, Constants.kD)
    config.closedLoop.velocityFF(0.0)
    config.closedLoop.outputRange(-Constants.kMaxOutput, +Constants.kMaxOutput)
    config.smartCurrentLimit(Constants.stallCurrentLimit)
    config.inverted(Constants.motorInverted)
    return config


def _configureTalonMotor(motor: TalonFX) -> None:
    config = TalonFXConfiguration()
    config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    config.motor_output.inverted = (
        InvertedValue.CLOCKWISE_POSITIVE if Constants.motorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    config.slot0.k_p = 100 * Constants.kP
    config.slot0.k_i = 0
    config.slot0.k_d = 0
    motor.configurator.apply(config)

    current = CurrentLimitsConfigs()
    current.stator_current_limit = Constants.stallCurrentLimit
    current.stator_current_limit_enable = True
    current.supply_current_limit = Constants.stallCurrentLimit
    current.supply_current_limit_enable = True
    motor.configurator.apply(current)
