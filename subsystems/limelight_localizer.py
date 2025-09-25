from dataclasses import dataclass
from typing import Dict

import wpilib
from commands2 import Subsystem
from wpilib import Timer, SmartDashboard, SendableChooser, DriverStation
from wpimath.geometry import Rotation2d, Translation3d, Pose2d, Translation2d

from subsystems.limelight_camera import LimelightCamera


U_TURN = Rotation2d.fromDegrees(180)
LEARNING_RATE = 0.3


@dataclass
class CameraState:
    camera: LimelightCamera
    cameraPoseOnRobot: Translation3d
    cameraHeadingOnRobot: Rotation2d
    minPercentFrame: float


class LimelightLocalizer(Subsystem):
    def __init__(self, drivetrain, flipIfRed=False):
        super().__init__()

        assert hasattr(drivetrain, "getHeading"), "drivetrain must have getHeading() for localizer to work"
        assert hasattr(drivetrain, "adjustOdometry"), "drivetrain must have adjustOdometry() for localizer to work"
        assert hasattr(drivetrain, "getPose"), "drivetrain must have getPose() for localizer to work"
        self.drivetrain = drivetrain

        from getpass import getuser

        self.username = getuser()
        self.flipIfRed = flipIfRed

        self.learningRateMult = SendableChooser()
        self.learningRateMult.addOption("100%", 1.0)
        self.learningRateMult.addOption("30%", 0.3)
        self.learningRateMult.setDefaultOption("10%", 0.1)
        self.learningRateMult.addOption("3%", 0.03)
        self.learningRateMult.addOption("1%", 0.01)
        self.learningRateMult.addOption("0.1%", 0.001)
        SmartDashboard.putData("LocLearnRate", self.learningRateMult)

        self.enabled = None
        self.allowed = True
        self.cameras: Dict[str, CameraState] = dict()  # list of Limelight cameras


    def addCamera(
        self,
        camera: LimelightCamera,
        cameraPoseOnRobot: Translation3d,
        cameraHeadingOnRobot: Rotation2d,
        minPercentFrame: float = 0.07,
    ) -> None:
        """
        :param camera: camera to add
        :param cameraPoseOnRobot: is camera x=0.3 meters to the front of the robot center and y=-0.2 meters to right?
        :param cameraHeadingOnRobot: is this camera looking straight forward (Rotation2d.fromDegrees(0)), or maybe right (Rotation2d.fromDegrees(-90)) ?
        :param maxDistanceMeters: only use this camera for localization if distance to tag is under so many meters
        :param minPercentFrame: if tags are too small (for example smaller than 0.07% of frame), do not use them
        """
        assert isinstance(camera, LimelightCamera), "you can only add LimelightCamera(s) to LimelightLocalizer"
        assert camera.cameraName not in self.cameras, f"camera {camera.cameraName} already added to LimelightLocalizer"
        self.cameras[camera.cameraName] = CameraState(camera, cameraPoseOnRobot, cameraHeadingOnRobot, minPercentFrame)
        camera.addLocalizer()


    def setAllowed(self, value: bool):
        self.allowed = value


    def periodic(self) -> None:
        enabled, flipped = None, False
        if self.enabled is None:
            self.initEnabledChooser()
        if self.enabled is not None:
            enabled, flipped = self.enabled.getSelected()
        if not self.allowed:
            enabled = False

        if not enabled:
            return

        learningRate = LEARNING_RATE * self.learningRateMult.getSelected()
        odometryPos: Pose2d = self.drivetrain.getPose()
        heading: Rotation2d = self.drivetrain.getHeading()
        assert heading is not None

        for c in self.cameras.values():
            camera = c.camera
            if not camera.ticked:
                continue

            p = c.cameraPoseOnRobot
            camera.cameraPoseSetRequest.set([p.x, p.y, p.z, c.cameraHeadingOnRobot.degrees(), 0.0, 0.0])

            # Limelight4-only (does nothing on Limelight 3)
            camera.imuModeRequest.set(0)
            # 0 - use external imu (the only option available on Limelight 3)
            # 1 - use external imu, seed internal imu
            # 2 - use internal
            # 3 - use internal with MT1 assisted convergence
            # 4 - use internal IMU with external IMU assisted convergence

            if flipped:
                yaw = (heading + U_TURN).degrees()
                camera.robotOrientationSetRequest.set([yaw, 0.0, 0.0, 0.0, 0.0, 0.0])
                botpose = camera.botPoseFlipped.get()
            else:
                yaw = heading.degrees()
                camera.robotOrientationSetRequest.set([yaw, 0.0, 0.0, 0.0, 0.0, 0.0])
                botpose = camera.botPose.get()

            if len(botpose) >= 11:
                # Translation (X,Y,Z), Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
                x, y, z, roll, pitch, yaw, latencyMillisec, count, span, distance, percentage = botpose[0:11]
                # SmartDashboard.putNumber("Localizer/" + c.camera.cameraName, percentage)
                if count > 0 and percentage > c.minPercentFrame and not (x == 0 and y == 0):
                    shift = Translation2d(x - odometryPos.x, y - odometryPos.y) * learningRate
                    self.drivetrain.adjustOdometry(shift, Rotation2d.fromDegrees(0))


    def initEnabledChooser(self):
        flipped = None
        if self.username == "lvuser" and self.flipIfRed is not None:
            # if we are running on RoboRIO, wait until driver station gives us alliance color
            color = DriverStation.getAlliance()
            if color is None:
                return  # we cannot yet decide on whether the field should be flipped
            flipped = (color == wpilib.DriverStation.Alliance.kRed) and self.flipIfRed
            print("Localizer: color={}, flipped={}".format(color, flipped))
        print("Localizer will assume flipped={} (username={}, flipIfRed={})".format(flipped, self.username, self.flipIfRed))

        self.enabled = SendableChooser()
        self.enabled.addOption("off", (None, False))
        if flipped in (None, False):
            self.enabled.setDefaultOption("on", (True, False))
        if flipped in (None, True):
            self.enabled.setDefaultOption("on-flipped", (True, True))
        SmartDashboard.putData("Localizer", self.enabled)
