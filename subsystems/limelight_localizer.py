import math
from dataclasses import dataclass
from typing import Dict

from commands2 import Subsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, Timer
from wpimath.geometry import Rotation2d, Translation3d, Pose2d, Translation2d

from subsystems.limelight_camera import LimelightCamera
from subsystems.photon_tag_camera import PhotonTagCamera, PhotonTagCameraSim

U_TURN = Rotation2d.fromDegrees(180)
LEARNING_RATE = 1.0
TYPICAL_PERCENT_FRAME = 0.7  # when the tag is ~2m away
EMPHASIZE_TAGS_NEARBY = False

@dataclass
class CameraState:
    camera: LimelightCamera
    cameraPoseOnRobot: Translation3d
    cameraHeadingOnRobot: Rotation2d
    cameraPitchAngleDegrees: float
    minPercentFrame: float
    maxRotationSpeed: float
    enabled: bool


class LimelightLocalizer(Subsystem):
    def __init__(self, drivetrain, flipIfRed=False):
        super().__init__()

        assert hasattr(drivetrain, "getHeading"), "drivetrain must have getHeading() for localizer to work"
        assert hasattr(drivetrain, "adjustOdometry"), "drivetrain must have adjustOdometry() for localizer to work"
        assert hasattr(drivetrain, "getPose"), "drivetrain must have getPose() for localizer to work"
        self.drivetrain = drivetrain

        self.flipIfRed = flipIfRed

        self.learningRateMult = SendableChooser()
        self.learningRateMult.addOption("300%", 300.0)
        self.learningRateMult.addOption("100%", 1.0)
        self.learningRateMult.addOption("30%", 0.3)
        self.learningRateMult.setDefaultOption("10%", 0.1)
        self.learningRateMult.addOption("3%", 0.03)
        self.learningRateMult.addOption("1%", 0.01)
        SmartDashboard.putData("LocaLearnRate", self.learningRateMult)

        self.flipped = False
        self.enabled = SendableChooser()
        self.enabled.addOption("off", False)
        self.enabled.setDefaultOption("on", True)
        SmartDashboard.putData("Localizer", self.enabled)

        self.allowed = True
        self.cameras: Dict[str, CameraState] = dict()  # list of Limelight cameras
        self.cycle = 0


    def addCamera(
        self,
        camera: LimelightCamera | PhotonTagCamera,
        cameraPoseOnRobot: Translation3d,
        cameraHeadingOnRobot: Rotation2d,
        cameraPitchAngleDegrees: float = 0.0,
        minPercentFrame: float = 0.07,
        maxRotationSpeed: float = 120,
    ) -> None:
        """
        :param camera: camera to add
        :param cameraPoseOnRobot: is camera x=0.3 meters to the front of the robot center and y=-0.2 meters to right?
        :param cameraHeadingOnRobot: is this camera looking straight forward (Rotation2d.fromDegrees(0)), or maybe right (Rotation2d.fromDegrees(-90)) ?
        :param cameraPitchAngleDegrees: is this camera pitched 10 degrees upwards? then set to +10.0
        :param maxDistanceMeters: only use this camera for localization if distance to tag is under so many meters
        :param minPercentFrame: if tags are too small (for example smaller than 0.07% of frame), do not use them
        :param maxRotationSpeed: when robot spins too fast (in degrees per second), camera will be ignored
        """
        assert isinstance(camera, (LimelightCamera, PhotonTagCamera, PhotonTagCameraSim)), \
            "you can only add LimelightCamera or PhotonTagCamera to LimelightLocalizer"
        assert camera.cameraName not in self.cameras, f"camera {camera.cameraName} already added to LimelightLocalizer"
        self.cameras[camera.cameraName] = CameraState(
            camera,
            cameraPoseOnRobot,
            cameraHeadingOnRobot,
            cameraPitchAngleDegrees,
            minPercentFrame,
            maxRotationSpeed,
            enabled=True
        )
        camera.addLocalizer()


    def setAllowed(self, value: bool):
        self.allowed = value


    def setCameraEnabled(self, name: str, enabled: bool):
        found = self.cameras.get(name)
        if found:
            found.enabled = enabled


    def setNewCameraPositionOnRobot(self, name: str, pos: Translation3d, heading: Rotation2d, pitchDegrees: float):
        found = self.cameras.get(name)
        if found:
            found.cameraPoseOnRobot = pos
            found.cameraHeadingOnRobot = heading
            found.cameraPitchAngleDegrees = pitchDegrees
            found.camera.setCameraPoseOnRobot(pos.x, pos.y, pos.z, heading.degrees(), 0.0, pitchDegrees)


    def infrequent(self) -> None:
        for c in self.cameras.values():
            p = c.cameraPoseOnRobot
            c.camera.setCameraPoseOnRobot(
                p.x, p.y, p.z, c.cameraPitchAngleDegrees, 0.0, c.cameraHeadingOnRobot.degrees()
            )
            # this resets the camera settings in case it got restarted


    def periodic(self) -> None:
        self.cycle += 1
        if self.cycle % 10 == 0:
            self.infrequent()

        if len(self.cameras) == 0:
            return

        enabled = self.enabled.getSelected() and self.allowed
        if not enabled:
            return

        learningRate: float = LEARNING_RATE * self.learningRateMult.getSelected()
        flipped = self.flipIfRed and DriverStation.getAlliance() == DriverStation.Alliance.kRed
        odometryPos: Pose2d = self.drivetrain.getPose()
        rotationSpeed: float = self.drivetrain.getTurnRate()  # rotation speed in degrees per second
        maxGain = 1.0 / len(self.cameras)  # avoiding oscillations if many cameras

        now = Timer.getFPGATimestamp()
        heading = odometryPos.rotation()
        if flipped:
            heading += U_TURN

        for c in self.cameras.values():
            c.camera.updateRobotHeading(now, heading)
            if not c.enabled or not c.camera.ticked or abs(rotationSpeed) > c.maxRotationSpeed:
                continue

            x, y, area, count = c.camera.getXYAPositionEstimate(flipped)
            if area > 0 and count > 0:
                # SmartDashboard.putNumber("Localizer/" + c.camera.cameraName, area)
                if area > c.minPercentFrame and not (x == 0 and y == 0):
                    gain = area / TYPICAL_PERCENT_FRAME  # tags nearby have more say than tags far away
                    if not EMPHASIZE_TAGS_NEARBY:
                        gain = math.sqrt(gain)
                    shift = Translation2d(x - odometryPos.x, y - odometryPos.y) * min(learningRate * gain, maxGain)
                    self.drivetrain.adjustOdometry(shift, Rotation2d.fromDegrees(0))
