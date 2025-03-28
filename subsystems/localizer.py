from __future__ import annotations

import math
import typing

import commands2
from wpilib import Timer, Field2d, SmartDashboard, SendableChooser, DriverStation

# new dependencies!
from robotpy_apriltag import AprilTagFieldLayout
from photonlibpy.photonCamera import PhotonCamera

from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from dataclasses import dataclass
from collections import deque

@dataclass
class CameraState:
    name: str
    cameraPoseOnRobot: Pose2d
    fovRatio: float
    photonCamera: PhotonCamera
    lastProcessedCameraTime: float
    lastDrawnTags: list
    lastRedrawTime: float
    recentRobotLocations: deque  # (time, pose) tuples


class Localizer(commands2.Subsystem):
    LEARNING_RATE = 0.1  # to converge faster you may want to increase this, but location can become more unstable
    ONLY_WORK_IF_SEEING_MULTIPLE_TAGS = True  # avoid making adjustments to odometry if only one tag is seen
    TAG_TOO_CLOSE_METERS = 1.0  # tags closer than 1 meter won't cause bigger impact on (X, Y) update

    TRUST_GYRO_COMPLETELY = True  # if you set it =True, odometry heading (North) will never be modified
    MAX_ANGULAR_DEVIATION_DEGREES = 45  # if a tag appears to be more than 45 degrees away, ignore it (something wrong)
    INTERSECTION_WEIGHT_FACTOR = 0.005

    REDRAW_DASHBOARD_FREQUENCY = 5  # how often to redraw the tags in shuffleboard
    MAX_LOCATION_HISTORY_SIZE = 50  # that's worth 1 second of updates, at 50 updates/sec

    def __init__(
        self,
        drivetrain,
        fieldLayoutFile: str,
        ignoreTagIDs=(),
        importantTagIDs=(),
        flippedFromAllianceColor: typing.Callable[[DriverStation.Alliance], bool] | None = None
    ):
        super().__init__()

        self.cameras = {}  # empty list of cameras, until cameras are added using addPhotonCamera
        self.drivetrain = drivetrain
        self.fieldDrawing: Field2d = drivetrain.field

        # enabled or not? (depends on how we convert alliance color to flipped)
        self.enabled: SendableChooser | None = None
        self.flippedFromAllianceColor = flippedFromAllianceColor
        self.allowed = True

        from os.path import isfile, join
        from getpass import getuser

        self.username = getuser()
        self.fieldLayout = None
        self.fieldLength = None
        self.fieldWidth = None
        for prefix in ['/home/lvuser/py/', '/home/lvuser/py_new/', '']:
            candidate = join(prefix, fieldLayoutFile)
            print(f"Localizer: trying field layout from {candidate}")
            if isfile(candidate):
                self.fieldLayout = AprilTagFieldLayout(candidate)
                self.fieldLength = self.fieldLayout.getFieldLength()
                self.fieldWidth = self.fieldLayout.getFieldWidth()
                break
        assert self.fieldLayout is not None, f"file with field layout {fieldLayoutFile} does not exist"
        print(f"Localizer: loaded field layout with {len(self.fieldLayout.getTags())} tags, from {fieldLayoutFile}")

        self.learningRateMult = SendableChooser()
        self.learningRateMult.addOption("100%", 1.0)
        self.learningRateMult.addOption("30%", 0.3)
        self.learningRateMult.setDefaultOption("10%", 0.1)
        self.learningRateMult.addOption("3%", 0.03)
        self.learningRateMult.addOption("1%", 0.01)
        self.learningRateMult.addOption("0.1%", 0.001)
        SmartDashboard.putData("LoclzLearnRate", self.learningRateMult)

        self.ignoreTagIDs = set(ignoreTagIDs)
        self.importantTagIDs = set(importantTagIDs)
        self.recentlySeenTags = deque()
        self.skippedTags = set()


    def setAllowed(self, allowed: bool):
        self.allowed = allowed


    def initEnabledChooser(self):
        flipped = None
        if self.username == "lvuser" and self.flippedFromAllianceColor is not None:
            # if we are running on RoboRIO, wait until driver station gives us alliance color
            color = DriverStation.getAlliance()
            if color is None:
                return  # we cannot yet decide on whether the field should be flipped
            flipped = self.flippedFromAllianceColor(color)
            print("Localizer: color={}, flippedFromAllianceColor={}".format(color, flipped))
        print("Localizer will assume flipped={} (username={}, rule={})".format(flipped, self.username, self.flippedFromAllianceColor))

        self.enabled = SendableChooser()
        self.enabled.addOption("off", (None, False))
        if flipped in (None, False):
            self.enabled.addOption("demo", (False, False))
            self.enabled.setDefaultOption("on", (True, False))
        if flipped in (None, True):
            self.enabled.addOption("demo-red", (False, True))
            self.enabled.setDefaultOption("on-red", (True, True))
        SmartDashboard.putData("Localizer", self.enabled)


    def addPhotonCamera(self, name, directionDegrees, positionFromRobotCenter=Translation2d(0, 0), fov=70):
        """
        :param name: name of the camera in PhotonVision
        :param directionDegrees: how many degrees to the left (front camera = 0, left camera = 90, right cam = -90 etc.)
        :param fov: field-of-view in degrees (70 is default)
        :return: None
        """
        assert name not in self.cameras, f"camera '{name}' was already added"
        photonCamera = PhotonCamera(name)
        fovRatio = fov / 70  # 70 is the value for Arducam OV9281
        cameraPose = Pose2d(positionFromRobotCenter, Rotation2d.fromDegrees(directionDegrees))
        self.cameras[name] = CameraState(name, cameraPose, fovRatio, photonCamera, 0, [], 0, deque())


    def recentlySawMoreThanOneTag(self, horizonTime):
        lastTagSeen = None
        for t, tagId in reversed(self.recentlySeenTags):
            if t < horizonTime:
                break
            if lastTagSeen is None:
                lastTagSeen = tagId
            elif tagId != lastTagSeen:
                return True  # saw lastTagSeen and this other tagId => more than one seen recently


    def periodic(self):
        enabled, flipped = None, False
        if self.enabled is None:
            self.initEnabledChooser()
        if self.enabled is not None:
            enabled, flipped = self.enabled.getSelected()
        if not self.allowed:
            enabled = False

        self.skippedTags.clear()
        now = Timer.getFPGATimestamp()
        robotPose = self.drivetrain.getPose()
        learningRate = Localizer.LEARNING_RATE * self.learningRateMult.getSelected()
        intersectionReady = True

        # if saw more multiple tags in the last 0.25s, then apply adjustments (if enabled)
        while len(self.recentlySeenTags) > 4 * Localizer.MAX_LOCATION_HISTORY_SIZE:
            self.recentlySeenTags.popleft()
        if Localizer.ONLY_WORK_IF_SEEING_MULTIPLE_TAGS:
            intersectionReady = self.recentlySawMoreThanOneTag(now - 0.25)

        for name, camera in self.cameras.items():
            if enabled is None:
                self.eraseUnusedLastDrawnTags(camera, [])
                camera.lastDrawnTags = []
                continue

            recentRobotLocations: deque = camera.recentRobotLocations

            # 1. should we skip the results from this camera?
            if not camera.photonCamera.isConnected():
                continue  # skip! this camera is not connected
            detections = camera.photonCamera.getLatestResult()
            cameraFrameTime = detections.getTimestampSeconds()
            if camera.lastProcessedCameraTime == cameraFrameTime:
                continue  # skip! we already saw this camera frame result
            camera.lastProcessedCameraTime = cameraFrameTime

            # 2. should we redraw the detected tags from on Shuffleboard/Elastic?
            redrawing = False
            if (now - camera.lastRedrawTime) * Localizer.REDRAW_DASHBOARD_FREQUENCY > 1:
                camera.lastRedrawTime = now
                redrawing = True

            # 3. find where the robot was at the time of this camera's last frame
            recentRobotPose: Pose2d = robotPose
            recentRobotLocations.append((now, robotPose))
            while len(recentRobotLocations) > Localizer.MAX_LOCATION_HISTORY_SIZE:
                recentRobotLocations.popleft()
            while len(recentRobotLocations) > 0:
                t, recentRobotPose = recentRobotLocations.popleft()
                if t > cameraFrameTime:
                    break  # found the robot pose at the time of the camera frame
            robotLocation2d = recentRobotPose.translation()
            robotDirection2d = recentRobotPose.rotation()

            # 4. go through the detected tags and make adjustments to robot (X, Y)
            tagsSeen = []
            for tag in detections.getTargets():
                tagId = tag.getFiducialId()
                if tagId in self.ignoreTagIDs:
                    continue
                tagsSeen.append(tagId)

                tagFieldPosition = self.fieldLayout.getTagPose(tagId)
                if tagFieldPosition is None:
                    self.skippedTags.add(tagId)
                    continue  # our field map does not know this tag
                self.recentlySeenTags.append((now, tagId))

                odometryAdjustment = self.calculateProximityOdometryAdjustment(
                    camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition, flipped, learningRate)

                if odometryAdjustment is None and intersectionReady:
                    odometryAdjustment = self.calculateIntersectionOdometryAdjustment(
                        camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition, flipped, learningRate)

                if enabled and (odometryAdjustment is not None):
                    shift, turn = odometryAdjustment
                    self.drivetrain.adjustOdometry(shift, turn)

            # 5. finally, finish the drawing of detected tags on the dashboard
            if redrawing:
                self.eraseUnusedLastDrawnTags(camera, tagsSeen)
                camera.lastDrawnTags = tagsSeen

        # if waiting for alliance color, then all tags are skipped
        if self.enabled is None:
            SmartDashboard.putString("LocalizerSkipped", "waiting for alliance color")
        elif not self.allowed:
            SmartDashboard.putString("LocalizerSkipped", "not allowed at the moment")
        else:
            skipped = ",".join(str(t) for t in self.skippedTags) if self.skippedTags else ""
            SmartDashboard.putString("LocalizerSkipped", skipped)

    def eraseUnusedLastDrawnTags(self, camera, tagsSeen):
        for tagId in camera.lastDrawnTags:
            if tagId not in tagsSeen:
                lineToTag, lineFromTag = self.tagLineNames(camera.name, tagId)
                self.fieldDrawing.getObject(lineToTag).setPoses([])
                self.fieldDrawing.getObject(lineFromTag).setPoses([])


    def calculateProximityOdometryAdjustment(
        self, camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition, flipped, learningRate
    ):
        ta = tag.getArea()
        if ta < 0.5:  # tag not in proximity
            return None

        tagId = tag.getFiducialId()
        tagPose = tagFieldPosition.toPose2d()
        tagLocation2d, tagDirection = tagPose.translation(), tagPose.rotation()
        if flipped:
            tagLocation2d = Translation2d(x=self.fieldLength, y=self.fieldWidth) - tagLocation2d
        else:
            tagDirection = tagDirection.rotateBy(Rotation2d.fromDegrees(180))

        # camera pose
        cameraPoseOnRobot: Pose2d = camera.cameraPoseOnRobot
        fovRatio = camera.fovRatio

        # localize robot camera in the frame of tag
        cameraDirection2d: Rotation2d = robotDirection2d.rotateBy(cameraPoseOnRobot.rotation())

        # cosine between camera and tag
        tagSeenSideways = (tagDirection - cameraDirection2d).cos()
        if tagSeenSideways > 0.67:
            #print(f"tag {tagId} is supposed to be facing camera {camera.name} with cos={tagSeenSideways}")
            pass
        else:
            #print(f"WARNING: tag {tagId} is not supposed to be facing camera well {camera.name}: cos={tagSeenSideways} ( tagDir={tagDirection.degrees()}, cameraDir={cameraDirection2d.degrees()} )")
            return None

        # now localize
        objectArea = ta / tagSeenSideways
        distance = math.sqrt(0.2 * 0.2 / (1.70 * 0.01 * objectArea * fovRatio * fovRatio))
        # ^^ Arducam OV9281 is 1.70 square radians
        angle = Rotation2d.fromDegrees(-tag.getYaw())

        # X and Y of tag in camera's frame
        tagInCameraFrame = Translation2d(distance * angle.cos(), distance * angle.sin())
        tagInRobotFrame = tagInCameraFrame.rotateBy(cameraPoseOnRobot.rotation()) + cameraPoseOnRobot.translation()
        tagLocationIfOdometryCorrect = robotLocation2d + tagInRobotFrame.rotateBy(robotDirection2d)

        # debug
        #print(f"tag {tagId}: yaw={tag.getYaw()}, distanceInch={distance/0.0254}, posInCamFrame={tagInCameraFrame.x},{tagInCameraFrame.y}, posInRobotFrame={tagInRobotFrame.x},{tagInRobotFrame.y}")
        #self.fieldDrawing.getObject("tag-seen").setPose(Pose2d(tagLocationIfOdometryCorrect, tagDirection))

        shiftMeters = tagLocation2d - tagLocationIfOdometryCorrect
        if redrawing:
            lineToTag, lineFromTag = self.tagLineNames(camera.name, tagId)

            #  - a line going from odometry position to the tag
            lineFromOdometryPositionToTag = drawLine(40, robotLocation2d, tagLocation2d)
            self.fieldDrawing.getObject(lineToTag).setPoses(lineFromOdometryPositionToTag)
            #  - and a line going back from the tag to robot's real position, using observed tag direction from camera
            lineFromTagToRealPosition = drawLine(40, tagLocation2d, robotLocation2d + shiftMeters)
            self.fieldDrawing.getObject(lineFromTag).setPoses(lineFromTagToRealPosition)

        # if too far, trust less (maybe not necessary, but let's keep for now)
        if distance > Localizer.TAG_TOO_CLOSE_METERS:
            factor = Localizer.TAG_TOO_CLOSE_METERS / distance
            learningRate *= factor
            # ^^ power 1, because the biggest cause will be systematic errors (uncalibrated camera angle or location)

        shift = shiftMeters * learningRate
        return shift, Rotation2d()


    def calculateIntersectionOdometryAdjustment(
        self, camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition, flipped, learningRate
    ):
        tagId = tag.getFiducialId()
        tagPose = tagFieldPosition.toPose2d()

        tagLocation2d, tagDirection = tagPose.translation(), tagPose.rotation()
        if flipped:
            tagLocation2d = Translation2d(x=self.fieldLength, y=self.fieldWidth) - tagLocation2d
            tagDirection = tagDirection.rotateBy(Rotation2d.fromDegrees(180))

        cameraPoseOnRobot: Pose2d = camera.cameraPoseOnRobot
        cameraLocation2d = robotLocation2d + cameraPoseOnRobot.translation().rotateBy(robotDirection2d)
        cameraDirectionDegrees = cameraPoseOnRobot.rotation().degrees()
        distanceToTag = tagLocation2d.distance(cameraLocation2d)

        # actual math, part 1:

        # say robot is already facing 30 degrees left and left camera is pointing 90 degrees further left
        # , but tag X is actually seen 10 degrees to the right of that (on that camera frame)
        #
        # ...then, of course, the tag is really 90 + 30 - 10 = 110 degrees to the left, correct?
        observedTagDirectionDegrees = cameraDirectionDegrees + robotDirection2d.degrees() - tag.getYaw()

        observedDirectionToTag = Rotation2d.fromDegrees(observedTagDirectionDegrees)
        observedVectorToTag = Translation2d(distanceToTag, 0).rotateBy(observedDirectionToTag)

        # but if odometry says that this vector below is our direction to tag
        odometryVectorToTag = tagLocation2d - cameraLocation2d

        # actual math, part 2:
        #  - say we see that the tag is at a slightly different angle than our (X,Y) predicted,
        #  - how much do we then shift our (X,Y), in meters?
        # answer: just subtract these two vectors -- actual direction to tag and previously assumed tag direction
        shiftMeters = odometryVectorToTag - observedVectorToTag

        # also, imagine we trust the (X, Y) but don't trust the heading angle: how much would that angle need to change?
        turnDegrees = odometryVectorToTag.angle().degrees() - observedVectorToTag.angle().degrees()
        turnDegrees = Rotation2d.fromDegrees((turnDegrees + 180) % 360 - 180)  # avoid dRot=+350 degrees if it's -10 deg

        skip = False
        if abs(turnDegrees.degrees()) > Localizer.MAX_ANGULAR_DEVIATION_DEGREES:
            skip = True

        # do we need to redraw the lines on the screen?
        if redrawing:
            lineToTag, lineFromTag = self.tagLineNames(camera.name, tagId)

            #  - if we have zero trust in this tag, draw a line that says we ignore it
            if skip:
                self.fieldDrawing.getObject(lineToTag).setPoses(drawLine(10, cameraLocation2d, tagLocation2d))
                self.fieldDrawing.getObject(lineFromTag).setPoses(drawLine(10, tagLocation2d, tagLocation2d - observedVectorToTag, fraction=0.5))
            else:
                #  - a line going from odometry position to the tag
                lineFromOdometryPositionToTag = drawLine(40, cameraLocation2d, tagLocation2d)
                self.fieldDrawing.getObject(lineToTag).setPoses(lineFromOdometryPositionToTag)
                #  - and line going back from the tag to robot's real position, using real actual tag direction from camera
                lineFromTagToRealPosition = drawLine(40, tagLocation2d, tagLocation2d - observedVectorToTag)
                self.fieldDrawing.getObject(lineFromTag).setPoses(lineFromTagToRealPosition)

        # if skipping, do not compute the odometry adjustment
        if skip:
            self.skippedTags.add(tagId)
            return None

        # if tag is too far from us, make it have smaller impact
        if distanceToTag > Localizer.TAG_TOO_CLOSE_METERS:
            factor = Localizer.TAG_TOO_CLOSE_METERS / distanceToTag
            learningRate *= factor
            # ^^ power 1, because the biggest cause will be systematic errors (uncalibrated camera angle or location)

        # for intersection localization, use lower learning rate
        learningRate *= self.INTERSECTION_WEIGHT

        # adjust the (X, Y) in the drivetrain odometry
        shift = shiftMeters * learningRate
        turn = Rotation2d(0.0) if Localizer.TRUST_GYRO_COMPLETELY else turnDegrees * learningRate * 0.25  # 0.25 because we still kind of trust the gyro
        return shift, turn


    def tagLineNames(self, cameraName, tagId):
        toTag = f"{cameraName}-fromtag{tagId}"
        fromTag = f"{cameraName}-totag{tagId}"
        return toTag, fromTag

def drawLine(pointsPerMeter: int, start: Translation2d, end: Translation2d, fraction=1.0):
    result = []
    direction = end - start
    angle = direction.angle()
    length = direction.norm()
    nPoints = max([3, int(length * pointsPerMeter)])
    for point in range(nPoints + 1):
        percentage = point / nPoints
        result.append(Pose2d(start + direction * percentage, angle))
        if percentage >= fraction:
            break
    return result
