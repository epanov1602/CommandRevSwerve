#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from wpilib import Field2d
from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from os.path import isfile, join


# if you are using Limelight, please set up the pipelines to match this
# (or you can provide your own `pipeline2TagGroup` that matches your Limelight, when constructing these command)
DEFAULT_PIPELINE_TO_TAGS = {
    0: (),  # all tags
    6: (6, 19),
    7: (7, 18),
    8: (8, 17),
    9: (9, 22),
    4: (10, 21),
    5: (11, 20),
}


class AimLike1811(commands2.Command):
    """
    Simply find the AprilTag that's facing the camera most directly
    (really amounts to taking robot heading and finding the nearest multiple of 60 degrees)
    """

    _fields = dict()

    def __init__(self, fieldLayoutFile, camera, drivetrain, cameraLocationOnRobot: Pose2d, reverse=False, pipeline2TagGroup=None, forceBlue=False):
        """
        :param fieldLayoutFile: something like "2025-reefscape.json" or "2024-crescendo.json", you can download them at
         https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/
        """
        super().__init__()
        assert camera is not None

        self.camera = camera
        self.addRequirements(camera)

        self.drivetrain = drivetrain
        self.field2d: Field2d = getattr(self.drivetrain, "field", None)

        self.reverse = reverse
        if cameraLocationOnRobot is None:
            cameraLocationOnRobot = Pose2d(0, 0, 0)
        self.toCameraLocation = Transform2d(Pose2d(0, 0, 0), cameraLocationOnRobot)
        self.forceBlue = forceBlue

        if fieldLayoutFile not in self._fields:
            self._fields[fieldLayoutFile] = loadFieldLayout(fieldLayoutFile)
        self.fieldLayout = self._fields[fieldLayoutFile]

        self.fieldLength = self.fieldLayout.getFieldLength()
        self.fieldWidth = self.fieldLayout.getFieldWidth()
        print(f"Field layout with {len(self.fieldLayout.getTags())} tags, from {fieldLayoutFile}")

        # which tag is in which pipeline?
        if pipeline2TagGroup is None:
            pipeline2TagGroup = DEFAULT_PIPELINE_TO_TAGS

        self.tagToPipeline = {}
        self.pipelineToTags = pipeline2TagGroup
        for pipeline, tags in pipeline2TagGroup.items():
            for tag in tags:
                pose = self.fieldLayout.getTagPose(tag)
                if pose is not None:
                    self.tagToPipeline[tag] = pipeline
        self.tagPoses = [(tagId, self.fieldLayout.getTagPose(tagId)) for tagId in self.tagToPipeline.keys()]
        self.tagPoses = [(tagId, pose.toPose2d()) for tagId, pose in self.tagPoses if pose is not None]

        # state
        self.chosenTagId = None
        self.chosenTagPose: Pose2d = None
        self.chosenPipeline = 0
        self.chosenTagIds = None


    def getChosenHeadingDegrees(self):
        if self.chosenTagPose is None:
            return None
        direction = self.chosenTagPose.rotation()
        if not self.reverse:  # robot direction should be opposite to tag's (we are supposed to face it)
            direction = direction.rotateBy(Rotation2d.fromDegrees(180))
        return direction.degrees()


    def recomputeChosenHeadingDegrees(self):
        self.pickChosenTagId()
        return self.getChosenHeadingDegrees()


    def initialize(self):
        self.pickChosenTagId()

        if hasattr(self.camera, "setOnlyTagIds"):
            self.camera.setOnlyTagIds(self.chosenTagIds)

        # if camera has "setPipeline", set it
        if hasattr(self.camera, "setPipeline"):
            self.camera.setPipeline(self.chosenPipeline)


    def isFinished(self) -> bool:
        # if camera has no "setPipeline", we have nothing to wait for
        if not hasattr(self.camera, "setPipeline"):
            return True
        # we are finished when the camera has responded that pipeline index is now set
        if self.camera.getPipeline() == self.chosenPipeline:
            return True
        # we are in sim, and camera doesn't respond
        if commands2.TimedCommandRobot.isSimulation():
            return True
        # otherwise, print that we aren't finished
        print("SetCameraPipeline: not yet finished, because camera pipeline is {} and we want {}".format(
            self.camera.getPipeline(), self.chosenPipeline)
        )


    def execute(self):
        pass


    def end(self, interrupted: bool):
        pass


    def reset(self):
        self.chosenTagId = None
        self.chosenTagPose = None
        self.chosenPipeline = 0
        self.chosenTagIds = None


    def pickChosenTagId(self):
        self.reset()

        # 0. where is the front of the robot
        front: Pose2d = self.drivetrain.getPose().transformBy(self.toCameraLocation)
        self.drawArrow("front", front, flip=False)

        # 1. which of the tags is in front of us?
        self.chosenTagId = self.findNearestVisibleTagInFront(front, fieldOfViewDegrees=61)

        # 2. decide on chosen pipeline and tags
        if self.chosenTagId is not None:
            self.chosenPipeline = self.tagToPipeline.get(self.chosenTagId)
            self.chosenTagIds = self.pipelineToTags.get(self.chosenPipeline)
            pose = self.fieldLayout.getTagPose(self.chosenTagId)
            self.chosenTagPose = pose.toPose2d() if pose is not None else None

        if self.chosenPipeline is None:
            self.chosenPipeline = 0  # this is the default pipeline, if nothing is chosen (the camera better have it)
            print(f"goal: no pipeline chosen => using 0, chosenTagIds={self.chosenTagIds}")

        # 3. display the result
        if self.field2d is not None:
            self.drawArrow("tag", self.chosenTagPose, flip=True)
        print(f"{self.__class__.__name__}: tag {self.chosenTagId} @ {self.getChosenHeadingDegrees()} degrees (tags: {self.chosenTagIds})")


    def findNearestVisibleTagInFront(self, front, fieldOfViewDegrees):
        bestTagId, bestDistance = None, None
        flippedFront = front.rotation().rotateBy(Rotation2d.fromDegrees(180))
        pickBlue = self.forceBlue or self.drivetrain.getPose().x < self.fieldLength / 2
        for tagId, tagPose in self.tagPoses:
            isBlue = tagPose.x < self.fieldLength / 2
            if isBlue != pickBlue:
                continue
            tagDirection: Rotation2d = tagPose.rotation()
            distance = abs((tagDirection - flippedFront).degrees())
            if distance > fieldOfViewDegrees / 2:
                continue
            if bestDistance is None or distance < bestDistance:
                bestTagId, bestDistance = tagId, distance
        if bestTagId is not None:
            print(f"goal: nearest tag in front is {bestTagId}, only {bestDistance} degrees away")
        else:
            print(f"goal: nearest tag in front is not found")
        return bestTagId


    def drawArrow(self, name: str, pose: Pose2d, flip):
        if self.field2d is None:
            return
        if pose is None:
            poses = []
        else:
            poses = [
                pose,
                pose.transformBy(Transform2d(0.03, 0, 0)),
                pose.transformBy(Transform2d(0.06, 0, 0)),
                pose.transformBy(Transform2d(0.09, 0, 0)),
                pose.transformBy(Transform2d(0.12, 0, 0)),
                pose.transformBy(Transform2d(0.15, 0, 0)),
                pose.transformBy(Transform2d(0.18, 0, 0)),
                pose.transformBy(Transform2d(0.21, 0, 0)),
                pose.transformBy(Transform2d(0.24, 0, 0)),
                pose.transformBy(Transform2d(0.27, 0, 0)),
                pose.transformBy(Transform2d(0.30, 0, 0)),
            ]
            if flip:
                poses += [
                    pose,
                    pose.transformBy(Transform2d(0.1, 0.1, 0)),
                    pose,
                    pose.transformBy(Transform2d(0.1, -0.1, 0)),
                ]
            else:
                poses += [
                    pose.transformBy(Transform2d(0.3, 0, 0)),
                    pose.transformBy(Transform2d(0.2, 0.1, 0)),
                    pose.transformBy(Transform2d(0.3, 0, 0)),
                    pose.transformBy(Transform2d(0.2, -0.1, 0)),
                ]
        self.field2d.getObject(name).setPoses(poses)



class AimLike5895(AimLike1811):
    """
    Picks an AprilTag that we were most likely meaning to approach with this camera
    (command named this way since we must give credit where it is well deserved).
    """
    MAX_DISTANCE_TO_TAG = 3  # meters (10 feet)


    def findNearestVisibleTagInFront(self, front, fieldOfViewDegrees):
        bestTagId, bestDistance = None, None
        for tagId, tagPose in self.tagPoses:
            visible, distance = self.isTagVisibleFromHere(front, tagPose, fieldOfViewDegrees)
            if visible and (bestDistance is None or distance < bestDistance):
                bestDistance = distance
                bestTagId = tagId
        return bestTagId


    def isTagVisibleFromHere(self, front, tagPose, fieldOfViewDegrees):
        directionToTag = Transform2d(front, tagPose)
        vectorToTag = directionToTag.translation()

        distance = vectorToTag.norm()
        if distance > self.MAX_DISTANCE_TO_TAG:
            return False, None  # too far

        direction = vectorToTag.angle().degrees()
        if abs(direction) > fieldOfViewDegrees / 2:
            return False, None  # won't fit into our imaginary 50 degree field of view

        orientation = directionToTag.rotation().rotateBy(Rotation2d.fromDegrees(180) - vectorToTag.angle())
        if abs(orientation.degrees()) > 45:
            return False, None  # tag tilted too far to the side, not oriented towards us

        return True, distance / orientation.cos()



def loadFieldLayout(fieldLayoutFile):
    from robotpy_apriltag import AprilTagFieldLayout

    for prefix in ['/home/lvuser/py/', '/home/lvuser/py_new/', '']:
        candidate = join(prefix, fieldLayoutFile)
        print(f"trying field layout from {candidate}")
        if isfile(candidate):
            return AprilTagFieldLayout(candidate)

    assert False, f"file with field layout {fieldLayoutFile} does not exist"