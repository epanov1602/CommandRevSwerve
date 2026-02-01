import math

from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from phoenix6.controls import PositionVoltage, MotionMagicVelocityVoltage
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.signals import NeutralModeValue, InvertedValue
from rev import SparkMax, SparkFlex, SparkLowLevel, SparkBase, SparkClosedLoopController, SparkRelativeEncoder, \
    ResetMode, PersistMode
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants, getSwerveDrivingMotorConfig, getSwerveTurningMotorConfig


DEBUG_ANGLE_FUSION = False


class SwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        cancoderCANId: int = -1,
        turnMotorInverted = True,
        revControllerType = SparkFlex,
        drivingIsTalon = False,
        turningIsTalon = False,
        placement: str = "",
    ) -> None:
        """Constructs a swerve module using Rev (SparMax/SparkFlex) or Talon (Kraken) motor controllers.
        The driving motor can either be Rev or Kraken (set `drivingIsKraken=True` to enable Kraken).
        """
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        # Do we have an independent absolute encoder? Use it for angle fusion!
        self.turningCancoder = None
        self.turningRelPositionRadians = 0.0
        self.fusedAngle = None
        if cancoderCANId >= 0:
            self.turningCancoder = CANcoder(cancoderCANId) if cancoderCANId >= 0 else None
            self.fusedAngle = FusedTurningAngle(placement)

        useAbsoluteAngleGoals = self.fusedAngle is None   # if angle fusion is not enabled, angle goals will be absolute

        # turning encoder and PID controller
        if turningIsTalon:
            assert self.turningCancoder is not None, "if turning motor is Talon, cancoder is mandatory"
            self.turningTalonMotor = TalonFX(turningCANId)

            config = TalonFXConfiguration()
            config.motor_output.neutral_mode = NeutralModeValue.BRAKE
            config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            config.slot0.k_p = 0.5 * ModuleConstants.kTurningP * math.tau
            config.slot0.k_i = 0
            config.slot0.k_d = 0
            self.turningTalonMotor.configurator.apply(config)

            current = CurrentLimitsConfigs()
            current.stator_current_limit = ModuleConstants.kTurningMotorCurrentLimit * 1.5
            current.stator_current_limit_enable = True
            current.supply_current_limit = ModuleConstants.kTurningMotorCurrentLimit
            current.supply_current_limit_enable = True
            self.turningTalonMotor.configurator.apply(current)

            self.turningTalonPositionRequest = PositionVoltage(0, slot=0)
            self.turningTalonMotor.set_position(0)
            self.turningRevAbsEncoder, self.turningRevRelEncoder = None, None
            self.turningRevAbsController, self.turningRevRelController = None, None
            self.turningRevMotor = None
        else:
            self.turningTalonMotor = None
            self.turningRevMotor = revControllerType(
                turningCANId, SparkLowLevel.MotorType.kBrushless
            )
            self.turningRevMotor.configure(
                getSwerveTurningMotorConfig(turnMotorInverted, useAbsoluteEncoderGoals=useAbsoluteAngleGoals),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters)
            self.turningRevRelEncoder = self.turningRevMotor.getEncoder()
            self.turningRevAbsEncoder = self.turningRevMotor.getAbsoluteEncoder()
            self.turningRevAbsController, self.turningRevRelController = None, None
            if useAbsoluteAngleGoals:
                self.turningRevAbsController = self.turningRevMotor.getClosedLoopController()
            else:
                self.turningRevRelController = self.turningRevMotor.getClosedLoopController()

        # At first, set the desired state to match our current position
        self.desiredState.angle = Rotation2d(self.getAbsoluteRadians())

        # Driving encoder and PID controller (Talon or Rev)

        # -- is it Talon?
        self.drivingTalonMotor: TalonFX | None = None
        self.drivingTalonVelocityRequest: MotionMagicVelocityVoltage | None = None
        self.drivingTalonRotationsToMeters = (
            ModuleConstants.kWheelCircumferenceMeters / ModuleConstants.kDrivingMotorReduction
        )

        # -- is it Rev?
        self.drivingRevMotor: SparkBase | None = None
        self.drivingRevEncoder: SparkRelativeEncoder | None = None
        self.drivingRevPIDController: SparkClosedLoopController | None = None

        self.drivingCanId = drivingCANId
        if drivingIsTalon:
            # Talon
            self.drivingTalonMotor = TalonFX(drivingCANId)

            config = TalonFXConfiguration()
            config.motor_output.neutral_mode = NeutralModeValue.BRAKE
            config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            config.slot0.k_p = ModuleConstants.kDrivingP * 0.55
            config.slot0.k_i = 0
            config.slot0.k_d = 0
            config.slot0.k_v = ModuleConstants.kDrivingFF * 0.55
            self.drivingTalonMotor.configurator.apply(config)

            current = CurrentLimitsConfigs()
            current.stator_current_limit = ModuleConstants.kDrivingMotorCurrentLimit * 1.5
            current.stator_current_limit_enable = True
            current.supply_current_limit = ModuleConstants.kDrivingMotorCurrentLimit
            current.supply_current_limit_enable = True
            self.drivingTalonMotor.configurator.apply(current)

            self.drivingTalonVelocityRequest = MotionMagicVelocityVoltage(
                0, acceleration=250, slot=0
            )
            self.drivingTalonMotor.set_position(0)

        else:
            # REV
            self.drivingRevMotor = revControllerType(
                drivingCANId, SparkLowLevel.MotorType.kBrushless
            )
            self.drivingRevMotor.configure(
                getSwerveDrivingMotorConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
            self.drivingRevPIDController = self.drivingRevMotor.getClosedLoopController()
            self.drivingRevEncoder = self.drivingRevMotor.getEncoder()
            self.drivingRevEncoder.setPosition(0)


        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.


    def getRelativeRadians(self) -> float:
        if self.turningRevRelEncoder is not None:
            return self.turningRevRelEncoder.getPosition() / ModuleConstants.kTurningReductionRatio * math.tau
        else:
            return self.turningTalonMotor.get_position().value / ModuleConstants.kTurningReductionRatio * math.tau


    def getAbsoluteRadians(self) -> float:
        if self.fusedAngle is not None:
            self.turningRelPositionRadians = rel = self.getRelativeRadians()
            return self.fusedAngle.to_absolute_radians(rel)
        else:
            return self.turningRevAbsEncoder.getPosition()


    def setRelativeRadiansGoal(self, angle) -> None:
        position = angle / math.tau * ModuleConstants.kTurningReductionRatio
        if self.turningTalonMotor is not None:
            self.turningTalonMotor.set_control(
                self.turningTalonPositionRequest.with_position(position)
            )
        else:
            self.turningRevRelController.setReference(
                position, SparkLowLevel.ControlType.kPosition
            )


    def setAbsoluteRadiansGoal(self, goal) -> None:
        if self.fusedAngle is not None:
            relative = self.fusedAngle.to_relative_radians(goal, self.turningRelPositionRadians)
            self.setRelativeRadiansGoal(relative)
        else:
            # setting absolute radian goal is only allowed if we have Rev
            self.turningRevAbsController.setReference(goal, SparkLowLevel.ControlType.kPosition)


    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        angle = self.getAbsoluteRadians()
        if self.drivingRevEncoder is not None:
            return SwerveModuleState(self.drivingRevEncoder.getVelocity(), Rotation2d(angle))
        else:
            rps = self.drivingTalonMotor.get_velocity().value
            mps = rps * self.drivingTalonRotationsToMeters
            return SwerveModuleState(mps, Rotation2d(angle))


    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        angle = self.getAbsoluteRadians()
        if self.drivingRevEncoder is not None:
            return SwerveModulePosition(self.drivingRevEncoder.getPosition(), Rotation2d(angle))
        else:
            rotations = self.drivingTalonMotor.get_position().value
            meters = rotations * self.drivingTalonRotationsToMeters
            return SwerveModulePosition(meters, Rotation2d(angle))


    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.

        """
        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            # if WPILib doesn't want us to move at all, don't bother to bring the wheels back to zero angle yet
            # (causes brownout protection when battery is lower: https://youtu.be/0Xi9yb1IMyA)
            inXBrake = abs(abs(desiredState.angle.degrees() % 90) - 45) < 0.01
            if not inXBrake:
                self.stop()
                return

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = desiredState
        SwerveModuleState.optimize(optimizedDesiredState, Rotation2d(self.getAbsoluteRadians()))

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.setAbsoluteRadiansGoal(optimizedDesiredState.angle.radians())

        if self.drivingRevPIDController is not None:
            self.drivingRevPIDController.setReference(
                optimizedDesiredState.speed, SparkLowLevel.ControlType.kVelocity
            )
        else:
            rps = optimizedDesiredState.speed / self.drivingTalonRotationsToMeters
            self.drivingTalonMotor.set_control(
                self.drivingTalonVelocityRequest.with_velocity(rps)
            )

        self.desiredState = desiredState


    def stop(self):
        """
        Stops the module in place to conserve energy and avoid unnecessary brownouts
        """
        if self.drivingRevPIDController is not None:
            self.drivingRevPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
        else:
            self.drivingTalonMotor.set_control(self.drivingTalonVelocityRequest.with_velocity(0))
            #self.drivingTalonMotor.stopMotor()
        angle = self.getAbsoluteRadians()
        self.setAbsoluteRadiansGoal(angle)
        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=Rotation2d(angle))


    def resetEncoders(self) -> None:
        """
        Zeroes the driving SwerveModule encoders.
        """
        if self.drivingRevEncoder is not None:
            self.drivingRevEncoder.setPosition(0)
        else:
            self.drivingTalonMotor.set_position(0)


    def fuseAngles(self):
        if self.fusedAngle is not None:
            # use cancoder if we have it
            if self.turningCancoder is not None:
                cancoder = self.turningCancoder.get_position()
                if cancoder.status.is_ok():
                    absolute = cancoder.value * math.tau  # radians
                    self.fusedAngle.observe(absolute, self.getRelativeRadians())
            # use Rev absolute encoder if we don't
            elif self.turningRevAbsEncoder is not None:
                absolute = self.turningRevAbsEncoder.getPosition()
                self.fusedAngle.observe(absolute, self.getRelativeRadians())


class FusedTurningAngle:
    """
    Converts the absolute rotations of the relative encoder into absolute rotations of absolute, and vice versa.
    (by observing the values these angles take)
    """
    def __init__(self, place: str):
        self.place = place
        self.relative_minus_absolute = 0.0
        self.not_ready = "no observations"

    def to_relative_radians(self, absolute_radians: float, current_rel_radians) -> float:
        result = absolute_radians + self.relative_minus_absolute
        while result - current_rel_radians > math.pi:  # unlikely but possible
            result -= math.tau
        while result - current_rel_radians < -math.pi:  # unlikely but possible
            result += math.tau
        return result

    def to_absolute_radians(self, relative_radians: float) -> float:
        return relative_radians - self.relative_minus_absolute

    def observe(self, absolute_radians: float, relative_radians: float) -> None:
        """
        Makes an observation of the distance between absolute and relative rotation sensors.

        :absolute_radians: (float): the reading of absolute encoder (in radians)
        :relative_radians: (float): the reading of relative encoder (in radians)
        """
        if self.not_ready:
            self.relative_minus_absolute = relative_radians - absolute_radians
            self.complain("")
            return

        observation = relative_radians - absolute_radians
        while observation - self.relative_minus_absolute > math.pi:
            observation -= math.tau  # wrap around
        while observation - self.relative_minus_absolute < -math.pi:
            observation += math.tau  # wrap around
        if ModuleConstants.kTurningKalmanGain > 0:
            correction = ModuleConstants.kTurningKalmanGain * (observation - self.relative_minus_absolute)
            self.relative_minus_absolute += correction

        if DEBUG_ANGLE_FUSION:
            SmartDashboard.putNumber(f"fusedAngle_{self.place}/absolute", absolute_radians)
            SmartDashboard.putNumber(f"fusedAngle_{self.place}/relative", relative_radians)
            SmartDashboard.putNumber(f"fusedAngle_{self.place}/rel_minus_abs", self.relative_minus_absolute)


    def complain(self, reason):
        if reason != self.not_ready:
            SmartDashboard.putString(f"fusedAngle_{self.place}/_not_ready", reason)
            self.not_ready = reason
