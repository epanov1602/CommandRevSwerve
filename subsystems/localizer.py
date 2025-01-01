import commands2
from wpilib import Timer, Field2d

# new dependencies!
from robotpy_apriltag import AprilTagFieldLayout
from photonlibpy.photonCamera import PhotonCamera

from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from dataclasses import dataclass

@dataclass
class CameraState:
    name: str
    directionDegrees: float
    photonCamera: PhotonCamera
    lastProcessedResult: float
    lastVisibleTags: list

class Localizer(commands2.Subsystem):

    def __init__(self, drivetrain, fieldLayoutFile: str):
        super().__init__()

        self.cameras = {}  # empty list of cameras, until cameras are added using addPhotonCamera
        self.drivetrain = drivetrain
        self.field2d: Field2d = drivetrain.field

        from os.path import isfile
        assert isfile(fieldLayoutFile), f"file with field layout {fieldLayoutFile} does not exist"
        self.fieldLayout = AprilTagFieldLayout(fieldLayoutFile)
        print(f"loaded field layout with {len(self.fieldLayout.getTags())} tags")

    def addPhotonCamera(self, name, directionDegrees):
        """
        :param name: name of the camera in PhotonVision
        :param directionDegrees: how many degrees to the left (front camera = 0, left camera = 90, right cam = -90 etc.)
        :return: None
        """
        assert name not in self.cameras, f"camera '{name}' was already added"
        photonCamera = PhotonCamera(name)
        self.cameras[name] = CameraState(name, directionDegrees, photonCamera, 0, [])

    def periodic(self):
        now = Timer.getFPGATimestamp()
        drivetrainPose: Pose2d = self.drivetrain.getPose()
        robotLocation2d = drivetrainPose.translation()
        robotDirection2d = drivetrainPose.rotation()

        for name, c in self.cameras.items():
            photonCamera: PhotonCamera = c.photonCamera
            if photonCamera.isConnected():
                detections = photonCamera.getLatestResult()
                detectionTime = detections.getTimestamp()
                for tag in detections.getTargets():
                    tagFieldPosition = self.fieldLayout.getTagPose(tag.getFiducialId())
                    if tagFieldPosition is None:
                        print(f"WARNING: tag {tag.getFiducialId()} is unknown on our field layout")
                        continue  # our field map does not know this tag

                    tagLocation2d = tagFieldPosition.toPose2d().translation()
                    distanceToTag = tagLocation2d.distance(robotLocation2d)

                    # now let's do math:
                    # say robot is already facing 30 degrees left and left camera is pointing 90 degrees further left
                    # , but tag X is actually seen 10 degrees to the right of that (on that camera frame)
                    #
                    # ...then, of course, the tag is really 90 + 30 - 10 = 110 degrees to the left, correct?
                    observedTagDirectionDegrees = c.directionDegrees + robotDirection2d.degrees() - tag.getYaw()
                    observedDirectionToTag = Rotation2d.fromDegrees(observedTagDirectionDegrees)
                    observedVectorToTag = Translation2d(distanceToTag, 0).rotateBy(observedDirectionToTag)

                    # but odometry says that this below is our vector to tag! (is odometry correct?)
                    odometryVectorToTag = tagLocation2d - robotLocation2d

                    dPose = odometryVectorToTag - observedVectorToTag
                    dRot = odometryVectorToTag.angle().degrees() - observedVectorToTag.angle().degrees()
                    dRot = Rotation2d.fromDegrees((dRot + 180) % 360 - 180)  # avoid situation when dRot=+350 degrees when it really is -10
                    self.drivetrain.adjustOdometry(dPose * 0.1, dRot * 0.1)

                    # let's draw it:
                    #  - a line going from the tag, and following back along actual direction seen on camera
                    observedLineFromTag = _make_line(40, tagLocation2d, tagLocation2d - observedVectorToTag)
                    #  - and a line going from the tag, and following back along actual direction seen on camera
                    odometryLineToTag = _make_line(10, robotLocation2d, tagLocation2d)

                    self.field2d.getObject(f"to-tag{tag.getFiducialId()}").setPoses(odometryLineToTag)
                    self.field2d.getObject(f"from-tag{tag.getFiducialId()}").setPoses(observedLineFromTag)


def _make_line(nPoints: int, start: Translation2d, end: Translation2d):
    result = []
    direction = end - start
    angle = direction.angle()
    for point in range(nPoints):
        percentage = (point + 1.0) / nPoints
        result.append(Pose2d(start + direction * percentage, angle))
    return result
