from commands2 import Subsystem
from rev import SparkBaseConfig, SparkBase, SparkFlex, ResetMode, PersistMode
from wpilib import SmartDashboard, Servo


class ShooterConstants:
    kShooterMotorA_CANID = 41
    kShooterMotorB_CANID = 40

    maxRPM = 6000
    kFF = 18.5 / 10000
    kP = 0.5 / 10000
    kD = 0.0 / 10000


class Shooter(Subsystem):
    """
    The easiest way to test the shooter is to put this into configureButtonBindings():
    ```

    self.driverController.button(XboxController.Button.kA).onTrue(
            InstantCommand(lambda: self.shooter.setVelocityGoal(2000, 1000))
    ).onFalse(
            InstantCommand(lambda: self.shooter.stop())
    )

    ```

    """
    def __init__(self, inverted=True, hoodServo: Servo | None = None) -> None:
        super().__init__()

        self.hoodServo = hoodServo
        # just in case the bounds were not set, set them
        self.hoodServo.setBounds(2000, 1500, 1500, 1500, 1000)
        self.hoodServoGoal = 0.0
        if hoodServo is not None:
            self.hoodServoGoal = hoodServo.get()
            self.setHoodServoGoal(self.hoodServoGoal)

        self.leadMotor = SparkFlex(ShooterConstants.kShooterMotorA_CANID, SparkBase.MotorType.kBrushless)
        self.leadMotor.configure(
            _getLeadMotorConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.followMotor = SparkFlex(ShooterConstants.kShooterMotorB_CANID, SparkBase.MotorType.kBrushless)
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
            return f"shooter under velocity goal: {velocity} < {self.velocityGoal}"
        elif velocity > self.velocityGoal + self.velocityTolerance:
            return f"shooter above velocity goal: {velocity} > {self.velocityGoal}"
        else:
            return ""  # shooter is ready

    def setHoodServoGoal(self, goal):
        self.hoodServoGoal = max(0.0, min(1.0, goal))
        SmartDashboard.putNumber("Shooter/hoodServoGoal", goal)
        if self.hoodServo is not None:
            self.hoodServo.set(self.hoodServoGoal)

    def setVelocityGoal(self, rpm, rpmTolerance):
        self.velocityTolerance = rpmTolerance
        self.velocityGoal = max(-ShooterConstants.maxRPM, min(ShooterConstants.maxRPM, rpm))
        self.pidController.setReference(self.velocityGoal * self.inverted, SparkBase.ControlType.kVelocity)

    def getVelocity(self):
        return self.encoder.getVelocity() * self.inverted

    def getVelocityGoal(self):
        return self.velocityGoal * self.inverted

    def periodic(self):
        seen = self.getVelocity()
        goal = self.getVelocityGoal()
        if goal != self.reportedVelocityGoal or abs(seen - self.reportedVelocitySeen) >= 0.001 * seen:
            SmartDashboard.putNumber("Shooter/rpmSeen", seen)
            self.reportedVelocitySeen = seen
            SmartDashboard.putNumber("Shooter/rpmGoal", goal)
            self.reportedVelocityGoal = goal

    def stop(self):
        self.leadMotor.stopMotor()
        self.velocityTolerance = 0
        self.velocityGoal = 0

    def driveHoodServo(self, velocity):
        """
        Usage example in configureButtonBindings(...):
        ```

        # for calibrataion, drive that servo using the right stick of the joystick up/down
        self.shooter.setDefaultCommand(RunCommand(
           lambda: self.shooter.driveHoodServo(
               self.driverController.getRawAxis(XboxController.Axis.kRightY)
           )
        ))

        ```
        :param velocity:
        """
        self.setHoodServoGoal(self.hoodServoGoal + velocity * 0.01)


def _getLeadMotorConfig() -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(True)
    config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.closedLoop.pid(ShooterConstants.kP, 0.0, ShooterConstants.kD)
    config.closedLoop.velocityFF(ShooterConstants.kFF)
    config.closedLoop.outputRange(-1, +1)
    return config

def _getFollowMotorConfig():
    followConfig = SparkBaseConfig()
    followConfig.follow(ShooterConstants.kShooterMotorA_CANID, True)  # True = inverted when following
    followConfig.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    return followConfig
