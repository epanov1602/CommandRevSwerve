from dataclasses import dataclass

from navx import AHRS
from commands2 import Subsystem, TimedCommandRobot
from wpilib import SmartDashboard, Timer
from phoenix6 import StatusCode, StatusSignal


@dataclass
class Signal(object):
    value: float = 0.0
    status: StatusCode = StatusCode.OK


class NavxGyro(Subsystem):
    """
    An attempt to extend NavX gyro to take a calibrated correction for overshoot, and work better in simulation.
    Imitates get_yaw/set_yaw interfaces of Pigeon2, so you can easily switch to Pigeon2 from this.
    """
    def __init__(self, gyroOvershootFraction: float = 0.0) -> None:
        """
        :param gyroOvershootFraction: if the gyro overshoots by 3.5 degrees on every full 360, set this to 3.5/360
        """
        super().__init__()

        self.gyro = AHRS.create_spi()
        self.overshoot = gyroOvershootFraction

        self.simulation = TimedCommandRobot.isSimulation()
        self.time = 0
        self.yaw = 0
        self.adjustment = 0
        self.state = "ok"

        self.yaw_signal = Signal(0.0, StatusCode.OK)
        self.speed_signal = Signal(0.0, StatusCode.OK)


    def reset(self):
        self.gyro.reset()
        self.gyro.setAngleAdjustment(0)
        self.adjustment = 0
        self.time = 0
        self.yaw = 0


    def get_yaw(self) -> Signal:
        """Returns the heading of the gyro, in degrees, tries to be smart when gyro is disconnected

        :returns: Z axis heading as degrees
        """
        if self.simulation:
            self.yaw_signal.value = self.yaw
            return self.yaw_signal

        now = Timer.getFPGATimestamp()
        past = self.time
        state = "ok"

        if not self.gyro.isConnected():
            state = "disconnected"
        else:
            notCalibrating = True
            if self.gyro.isCalibrating():
                notCalibrating = False
                state = "calibrating"
            gyroAngle = self.gyro.getAngle()

            # correct for gyro drift
            if self.overshoot != 0.0 and self.yaw != 0 and notCalibrating:
                angleMove = gyroAngle - self.yaw
                if abs(angleMove) > 15:  # if less than 10 degrees, adjust (otherwise it's some kind of glitch or reset)
                    print(f"WARNING: big angle move {angleMove} from {self.yaw} to {gyroAngle}")
                else:
                    adjustment = -angleMove * self.overshoot
                    self.adjustment += adjustment
                    self.gyro.setAngleAdjustment(max(-359, min(+359, self.adjustment)))
                    # ^^ NavX code doesn't like angle adjustments outside of (-360, +360) range

            self.yaw = gyroAngle
            self.time = now

        if state != self.state:
            SmartDashboard.putString("gyro", f"{state} after {int(now - past)}s")
            self.state = state

        self.yaw_signal.value = self.yaw
        return self.yaw_signal


    def set_yaw(self, yaw: float) -> None:
        assert self.simulation
        self.yaw = yaw


    def get_angular_velocity_z_device(self) -> Signal:
        self.speed_signal.value = self.gyro.getRate()
        return self.speed_signal
