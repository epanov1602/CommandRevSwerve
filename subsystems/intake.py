from commands2 import Subsystem
from rev import SparkBaseConfig, SparkBase, SparkFlex, ResetMode, PersistMode
from wpilib import SmartDashboard, Servo


class IntakeConstants:
    kIntakeMotorA_CANID = 50 # Can ID not choosen yet
    kIntakeMotorB_CANID = 51 # Can ID not choosen yet
    kFollowMotorInverted = False

    maxRPM = 6000
    kFF = 18.5 / 10000
    kP = 4.0 / 10000
    kD = 0.0 / 10000
    targetRPM = -1500

    stallCurrentLimit = 50  # amps, and it must be integer for Rev


class Intake(Subsystem):
    """
    The easiest way to test the intake is to put this into configureButtonBindings():
    ```

    self.driverController.button(XboxController.Button.kA).onTrue(
            InstantCommand(lambda: self.intake.setVelocityGoal(2000, 1000))
    ).onFalse(
            InstantCommand(lambda: self.intake.stop())
    )

    ```

    """
    def __init__(self, inverted=True) -> None:
        super().__init__()

        self.leadMotor = SparkFlex(IntakeConstants.kIntakeMotorA_CANID, SparkBase.MotorType.kBrushless)
        self.leadMotor.configure(
            _getLeadMotorConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.followMotor = SparkFlex(IntakeConstants.kIntakeMotorB_CANID, SparkBase.MotorType.kBrushless)
        self.followMotor.configure(
            _getFollowMotorConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.pidController = self.leadMotor.getClosedLoopController()
        self.encoder = self.leadMotor.getEncoder()
        self.velocityGoal = 0
        self.velocityTolerance = 0
        self.inverted = -1 if inverted else 1
        # best is to not invert the lead motor, because such config is lost in brownout

        self.reportedVelocityGoal = 0
        self.reportedVelocitySeen = 0

    def notReady(self) -> str:
        velocity = self.getVelocity()
        if velocity < self.velocityGoal - self.velocityTolerance:
            return f"intake under velocity goal: {velocity} < {self.velocityGoal}"
        elif velocity > self.velocityGoal + self.velocityTolerance:
            return f"intake above velocity goal: {velocity} > {self.velocityGoal}"
        else:
            return ""  # intake is ready


    def setVelocityGoal(self, rpm, rpmTolerance):
        self.velocityTolerance = rpmTolerance
        self.velocityGoal = max(-IntakeConstants.maxRPM, min(IntakeConstants.maxRPM, rpm))
        self.pidController.setReference(self.velocityGoal * self.inverted, SparkBase.ControlType.kVelocity)

    def getVelocity(self):
        return self.encoder.getVelocity() * self.inverted

    def getVelocityGoal(self):
        return self.velocityGoal * self.inverted

    def periodic(self):
        seen = self.getVelocity()
        goal = self.getVelocityGoal()
        if goal != self.reportedVelocityGoal or abs(seen - self.reportedVelocitySeen) >= 0.001 * seen:
            SmartDashboard.putNumber("Intake/rpmSeen", seen)
            self.reportedVelocitySeen = seen
            SmartDashboard.putNumber("Intake/rpmGoal", goal)
            self.reportedVelocityGoal = goal

    def stop(self):
        self.leadMotor.stopMotor()
        self.velocityTolerance = 0
        self.velocityGoal = 0


def _getLeadMotorConfig() -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(True)
    config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.closedLoop.pid(IntakeConstants.kP, 0.0, IntakeConstants.kD)
    config.closedLoop.velocityFF(IntakeConstants.kFF)
    config.closedLoop.outputRange(-1, +1)
    config.smartCurrentLimit(IntakeConstants.stallCurrentLimit)
    return config

def _getFollowMotorConfig():
    followConfig = SparkBaseConfig()
    followConfig.follow(IntakeConstants.kIntakeMotorA_CANID, IntakeConstants.kFollowMotorInverted)  # True = inverted when following
    followConfig.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    followConfig.smartCurrentLimit(IntakeConstants.stallCurrentLimit)
    return followConfig
