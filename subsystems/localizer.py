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
    lastProcessedResultId: float
    lastDrawnTags: list
    lastRedrawTime: float


class Localizer(commands2.Subsystem):
    REDRAW_DASHBOARD_FREQUENCY = 5  # how often to redraw the tags in shuffleboard
    IMPORTANT_TAG_WEIGHT = 3.0

    def __init__(self, drivetrain, fieldLayoutFile: str, ignoreTagIDs=(), importantTagIDs=()):
        super().__init__()

        self.cameras = {}  # empty list of cameras, until cameras are added using addPhotonCamera
        self.drivetrain = drivetrain
        self.fieldDrawing: Field2d = drivetrain.field

        from os.path import isfile
        assert isfile(fieldLayoutFile), f"file with field layout {fieldLayoutFile} does not exist"
        self.fieldLayout = AprilTagFieldLayout(fieldLayoutFile)
        print(f"Localizer: loaded field layout with {len(self.fieldLayout.getTags())} tags")

        self.ignoreTagIDs = set(ignoreTagIDs)
        self.importantTagIDs = set(importantTagIDs)

        self.trustGyro = False
        self.learningRate = 0.5
        self.maxAngularDeviationDegrees = 45  # if the tag is further away than 45 degrees, ignore it

    def addPhotonCamera(self, name, directionDegrees):
        """
        :param name: name of the camera in PhotonVision
        :param directionDegrees: how many degrees to the left (front camera = 0, left camera = 90, right cam = -90 etc.)
        :return: None
        """
        assert name not in self.cameras, f"camera '{name}' was already added"
        photonCamera = PhotonCamera(name)
        self.cameras[name] = CameraState(name, directionDegrees, photonCamera, 0, [], 0)

    def periodic(self):
        now = Timer.getFPGATimestamp()
        drivetrainPose: Pose2d = self.drivetrain.getPose()
        robotLocation2d = drivetrainPose.translation()
        robotDirection2d = drivetrainPose.rotation()

        for name, c in self.cameras.items():
            # should we skip the results from this camera?
            if not c.photonCamera.isConnected():
                continue  # skip! this camera is not connected
            detections = c.photonCamera.getLatestResult()
            detectionTime = detections.getTimestamp()
            if c.lastProcessedResultId == detectionTime:
                continue  # skip! we already saw this camera frame result
            c.lastProcessedResultId = detectionTime

            # should we redraw the tags from this camera on Shuffleboard/Elastic?
            redrawing = False
            if (now - c.lastRedrawTime) * Localizer.REDRAW_DASHBOARD_FREQUENCY > 1:
                c.lastRedrawTime = now
                redrawing = True

            # proceed!
            tagsSeen = []
            for tag in detections.getTargets():
                    tagId = tag.getFiducialId()
                    if tagId in self.ignoreTagIDs:
                        continue
                    tagsSeen.append(tagId)

                    tagFieldPosition = self.fieldLayout.getTagPose(tagId)
                    if tagFieldPosition is None:
                        print(f"WARNING: tag {tagId} is unknown on our field layout")
                        continue  # our field map does not know this tag

                    odometryAdjustment = self.calculateOdometryAdjustment(
                        c, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition)

                    if odometryAdjustment is not None:
                        shift, turn = odometryAdjustment
                        self.drivetrain.adjustOdometry(shift, turn)

            # - finally, remove the drawn lines for the tag we no longer see
            if redrawing:
                for tagId in c.lastDrawnTags:
                    if tagId not in tagsSeen:
                        lineToTag, lineFromTag = self.tagLineNames(c.name, tagId)
                        self.fieldDrawing.getObject(lineToTag).setPoses([])
                        self.fieldDrawing.getObject(lineFromTag).setPoses([])
                c.lastDrawnTags = tagsSeen

    def tagLineNames(self, cameraName, tagId):
        toTag = f"cam{cameraName}-fromtag{tagId}"
        fromTag = f"cam{cameraName}-totag{tagId}"
        return toTag, fromTag

    def calculateOdometryAdjustment(self, camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition):
        tagId = tag.getFiducialId()
        tagLocation2d = tagFieldPosition.toPose2d().translation()
        distanceToTag = tagLocation2d.distance(robotLocation2d)

        # now let's do math:
        # say robot is already facing 30 degrees left and left camera is pointing 90 degrees further left
        # , but tag X is actually seen 10 degrees to the right of that (on that camera frame)
        #
        # ...then, of course, the tag is really 90 + 30 - 10 = 110 degrees to the left, correct?
        observedTagDirectionDegrees = camera.directionDegrees + robotDirection2d.degrees() - tag.getYaw()
        observedDirectionToTag = Rotation2d.fromDegrees(observedTagDirectionDegrees)
        observedVectorToTag = Translation2d(distanceToTag, 0).rotateBy(observedDirectionToTag)

        # but if odometry says that this vector below is our direction to tag
        odometryVectorToTag = tagLocation2d - robotLocation2d

        # how much do we need to adjust the odometry (X, Y) to match the direction that we see?
        shiftMeters = odometryVectorToTag - observedVectorToTag
        turnDegrees = odometryVectorToTag.angle().degrees() - observedVectorToTag.angle().degrees()
        turnDegrees = Rotation2d.fromDegrees((turnDegrees + 180) % 360 - 180)  # avoid dRot=+350 degrees if it's really -10

        # use "learning rate" (for example, 0.05) to apply this X, Y adjustment only partially (+do it again next time)
        learningRate = self.learningRate
        if tagId in self.importantTagIDs:
            learningRate = self.learningRate * Localizer.IMPORTANT_TAG_WEIGHT

        if abs(turnDegrees.degrees()) > self.maxAngularDeviationDegrees:
            learningRate = 0

        # do we need to redraw the lines on the screen?
        if redrawing:
            lineToTag, lineFromTag = self.tagLineNames(camera.name, tagId)

            #  - if we have zero trust in this tag, draw a line that says we ignore it
            if learningRate == 0:
                self.fieldDrawing.getObject(lineToTag).setPoses(drawLine(10, robotLocation2d, tagLocation2d))
                self.fieldDrawing.getObject(lineFromTag).setPoses([])

            #  - a line going from odometry position to the tag
            lineFromOdometryPositionToTag = drawLine(40, robotLocation2d, tagLocation2d)
            self.fieldDrawing.getObject(lineToTag).setPoses(lineFromOdometryPositionToTag)

            #  - and line going back from the tag to robot's real position, using real actual tag direction from camera
            lineFromTagToRealPosition = drawLine(40, tagLocation2d, tagLocation2d - observedVectorToTag)
            self.fieldDrawing.getObject(lineFromTag).setPoses(lineFromTagToRealPosition)

        # adjust the (X, Y) in the drivetrain odometry
        shift = shiftMeters * learningRate
        turn = turnDegrees * learningRate * 0.25  # 0.25 because we still kind of trust the gyro
        return shift, turn

def drawLine(nPoints: int, start: Translation2d, end: Translation2d):
    result = []
    direction = end - start
    angle = direction.angle()
    for point in range(nPoints):
        percentage = (point + 1.0) / nPoints
        result.append(Pose2d(start + direction * percentage, angle))
    return result
