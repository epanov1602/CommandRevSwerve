from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, InvertedValue
from rev import SparkMax, SparkFlex, SparkLowLevel, SparkBase, SparkClosedLoopController, SparkRelativeEncoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants, getSwerveDrivingMotorConfig, getSwerveTurningMotorConfig


class MAXSwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        chassisAngularOffset: float,
        turnMotorInverted = True,
        motorControllerType = SparkFlex,
        drivingIsTalon = False,
    ) -> None:
        """Constructs a MAXSwerveModule where turning motor is Rev and uses rev absolute encoder.
        The driving motor can either be Rev or Kraken (set `drivingIsKraken=True` to enable Kraken).
        """
        self.chassisAngularOffset = 0
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        # turning encoder and PID controller
        self.turningRevMotor = motorControllerType(
            turningCANId, SparkLowLevel.MotorType.kBrushless
        )
        self.turningRevMotor.configure(
            getSwerveTurningMotorConfig(turnMotorInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)
        self.turningEncoder = self.turningRevMotor.getAbsoluteEncoder()
        self.turningPIDController = self.turningRevMotor.getClosedLoopController()

        # driving encoder and PID controller (Talon or Rev)

        # -- is it Talon?
        self.drivingTalonMotor: TalonFX | None = None
        self.drivingTalonVelocityRequest: VelocityVoltage | None = None
        self.drivingTalonRotationsToMeters = (
            ModuleConstants.kWheelCircumferenceMeters / ModuleConstants.kDrivingMotorReduction
        )

        # -- is it Rev?
        self.drivingRevMotor: SparkBase | None = None
        self.drivingRevEncoder: SparkRelativeEncoder | None = None
        self.drivingRevPIDController: SparkClosedLoopController | None = None

        if drivingIsTalon:
            # Talon
            self.drivingTalonMotor = TalonFX(drivingCANId)
            config = TalonFXConfiguration()
            config.motor_output.neutral_mode = NeutralModeValue.BRAKE
            config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            config.slot0.k_p = ModuleConstants.kDrivingP * 7.5
            config.slot0.k_i = 0
            config.slot0.k_d = 0
            self.drivingTalonMotor.configurator.apply(config)

            self.drivingTalonVelocityRequest = VelocityVoltage(0, slot=0)
            self.drivingTalonMotor.set_position(0)

        else:
            # REV
            self.drivingRevMotor = motorControllerType(
                drivingCANId, SparkLowLevel.MotorType.kBrushless
            )
            self.drivingRevMotor.configure(
                getSwerveDrivingMotorConfig(),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
            self.drivingRevPIDController = self.drivingRevMotor.getClosedLoopController()
            self.drivingRevEncoder = self.drivingRevMotor.getEncoder()
            self.drivingRevEncoder.setPosition(0)


        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())


    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        if self.drivingRevEncoder is not None:
            return SwerveModuleState(
                self.drivingRevEncoder.getVelocity(),
                Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
            )
        else:
            rps = self.drivingTalonMotor.get_velocity().value
            mps = rps * self.drivingTalonRotationsToMeters
            return SwerveModuleState(mps, Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset))


    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        if self.drivingRevEncoder is not None:
            return SwerveModulePosition(
                self.drivingRevEncoder.getPosition(),
                Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
            )
        else:
            rotations = self.drivingTalonMotor.get_position().value
            meters = rotations * self.drivingTalonRotationsToMeters
            return SwerveModulePosition(
                meters,
                Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset)
            )


    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.

        """
        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            # if WPILib doesn't want us to move at all, don't bother to bring the wheels back to zero angle yet
            # (causes brownout protection when battery is lower: https://youtu.be/0Xi9yb1IMyA)
            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
            if not inXBrake:
                self.stop()
                return

        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(
            self.chassisAngularOffset
        )

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = correctedDesiredState
        SwerveModuleState.optimize(
            optimizedDesiredState, Rotation2d(self.turningEncoder.getPosition())
        )

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.turningPIDController.setReference(
            optimizedDesiredState.angle.radians(), SparkLowLevel.ControlType.kPosition
        )

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
            self.drivingTalonMotor.set_control(
                self.drivingTalonVelocityRequest.with_velocity(0)
            )
        self.turningPIDController.setReference(self.turningEncoder.getPosition(), SparkLowLevel.ControlType.kPosition)
        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)


    def resetEncoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        if self.drivingRevEncoder is not None:
            self.drivingRevEncoder.setPosition(0)
        else:
            self.drivingTalonMotor.set_position(0)
