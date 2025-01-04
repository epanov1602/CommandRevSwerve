# Adding a Localizer (automatic odometry X,Y adjustments from cameras looking at tags)

## 0. Installing dependencies
In `requirements.txt` we need to add two more lines (dependencies):
```python
photonlibpy==2024.3.1
robotpy-apriltag==2024.3.2.1
```
, and these will need to be installed.

In `pyproject.toml` we need to change `requires` line to:
```python
# Other pip packages to install
requires = [
    "pyntcore",
    "photonlibpy",
    "robotpy-apriltag",
]
```

## 1. Adding the map of the field
In the root directory of the project, we can add a JSON file with map of the field.


For example, for 2024 it will be `2024-crescendo.json` as pasted below:
<details>
<summary>(click to expand)</summary>

```json
{
  "tags": [
    {
      "ID": 1,
      "pose": {
        "translation": {
          "x": 15.079471999999997,
          "y": 0.24587199999999998,
          "z": 1.355852
        },
        "rotation": {
          "quaternion": {
            "W": 0.5000000000000001,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.8660254037844386
          }
        }
      }
    },
    {
      "ID": 2,
      "pose": {
        "translation": {
          "x": 16.185134,
          "y": 0.883666,
          "z": 1.355852
        },
        "rotation": {
          "quaternion": {
            "W": 0.5000000000000001,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.8660254037844386
          }
        }
      }
    },
    {
      "ID": 3,
      "pose": {
        "translation": {
          "x": 16.579342,
          "y": 4.982717999999999,
          "z": 1.4511020000000001
        },
        "rotation": {
          "quaternion": {
            "W": 6.123233995736766e-17,
            "X": 0.0,
            "Y": 0.0,
            "Z": 1.0
          }
        }
      }
    },
    {
      "ID": 4,
      "pose": {
        "translation": {
          "x": 16.579342,
          "y": 5.547867999999999,
          "z": 1.4511020000000001
        },
        "rotation": {
          "quaternion": {
            "W": 6.123233995736766e-17,
            "X": 0.0,
            "Y": 0.0,
            "Z": 1.0
          }
        }
      }
    },
    {
      "ID": 5,
      "pose": {
        "translation": {
          "x": 14.700757999999999,
          "y": 8.2042,
          "z": 1.355852
        },
        "rotation": {
          "quaternion": {
            "W": -0.7071067811865475,
            "X": -0.0,
            "Y": 0.0,
            "Z": 0.7071067811865476
          }
        }
      }
    },
    {
      "ID": 6,
      "pose": {
        "translation": {
          "x": 1.8415,
          "y": 8.2042,
          "z": 1.355852
        },
        "rotation": {
          "quaternion": {
            "W": -0.7071067811865475,
            "X": -0.0,
            "Y": 0.0,
            "Z": 0.7071067811865476
          }
        }
      }
    },
    {
      "ID": 7,
      "pose": {
        "translation": {
          "x": -0.038099999999999995,
          "y": 5.547867999999999,
          "z": 1.4511020000000001
        },
        "rotation": {
          "quaternion": {
            "W": 1.0,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
          }
        }
      }
    },
    {
      "ID": 8,
      "pose": {
        "translation": {
          "x": -0.038099999999999995,
          "y": 4.982717999999999,
          "z": 1.4511020000000001
        },
        "rotation": {
          "quaternion": {
            "W": 1.0,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
          }
        }
      }
    },
    {
      "ID": 9,
      "pose": {
        "translation": {
          "x": 0.356108,
          "y": 0.883666,
          "z": 1.355852
        },
        "rotation": {
          "quaternion": {
            "W": 0.8660254037844387,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.49999999999999994
          }
        }
      }
    },
    {
      "ID": 10,
      "pose": {
        "translation": {
          "x": 1.4615159999999998,
          "y": 0.24587199999999998,
          "z": 1.355852
        },
        "rotation": {
          "quaternion": {
            "W": 0.8660254037844387,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.49999999999999994
          }
        }
      }
    },
    {
      "ID": 11,
      "pose": {
        "translation": {
          "x": 11.904726,
          "y": 3.7132259999999997,
          "z": 1.3208
        },
        "rotation": {
          "quaternion": {
            "W": -0.8660254037844387,
            "X": -0.0,
            "Y": 0.0,
            "Z": 0.49999999999999994
          }
        }
      }
    },
    {
      "ID": 12,
      "pose": {
        "translation": {
          "x": 11.904726,
          "y": 4.49834,
          "z": 1.3208
        },
        "rotation": {
          "quaternion": {
            "W": 0.8660254037844387,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.49999999999999994
          }
        }
      }
    },
    {
      "ID": 13,
      "pose": {
        "translation": {
          "x": 11.220196,
          "y": 4.105148,
          "z": 1.3208
        },
        "rotation": {
          "quaternion": {
            "W": 6.123233995736766e-17,
            "X": 0.0,
            "Y": 0.0,
            "Z": 1.0
          }
        }
      }
    },
    {
      "ID": 14,
      "pose": {
        "translation": {
          "x": 5.320792,
          "y": 4.105148,
          "z": 1.3208
        },
        "rotation": {
          "quaternion": {
            "W": 1.0,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
          }
        }
      }
    },
    {
      "ID": 15,
      "pose": {
        "translation": {
          "x": 4.641342,
          "y": 4.49834,
          "z": 1.3208
        },
        "rotation": {
          "quaternion": {
            "W": 0.5000000000000001,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.8660254037844386
          }
        }
      }
    },
    {
      "ID": 16,
      "pose": {
        "translation": {
          "x": 4.641342,
          "y": 3.7132259999999997,
          "z": 1.3208
        },
        "rotation": {
          "quaternion": {
            "W": -0.4999999999999998,
            "X": -0.0,
            "Y": 0.0,
            "Z": 0.8660254037844387
          }
        }
      }
    }
  ],
  "field": {
    "length": 16.541,
    "width": 8.211
  }
}
```
</details>

## 2. Adding Localizer subsystem
We need to add the code for the new subsystem (Localizer) into `subsystem/localizer.py`
<details>
<summary>(click to expand)</summary>

```python
import commands2
from wpilib import Timer, Field2d, SmartDashboard, SendableChooser

# new dependencies!
from robotpy_apriltag import AprilTagFieldLayout
from photonlibpy.photonCamera import PhotonCamera

from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from dataclasses import dataclass
from collections import deque

@dataclass
class CameraState:
    name: str
    directionDegrees: float
    photonCamera: PhotonCamera
    lastProcessedCameraTime: float
    lastDrawnTags: list
    lastRedrawTime: float
    recentRobotLocations: deque  # (time, pose) tuples


class Localizer(commands2.Subsystem):
    LEARNING_RATE = 0.4  # to converge faster you may want to increase this, but location can become more unstable

    TRUST_GYRO_COMPLETELY = False  # if you set it =True, odometry heading (North) will never be modified
    MAX_ANGULAR_DEVIATION_DEGREES = 45  # if a tag appears to be more than 45 degrees away, ignore it (something wrong)
    IMPORTANT_TAG_WEIGHT_FACTOR = 2.0  # if a tag is important (for example, located on a shooting target)

    REDRAW_DASHBOARD_FREQUENCY = 5  # how often to redraw the tags in shuffleboard
    MAX_LOCATION_HISTORY_SIZE = 50  # that's worth 1 second of updates, at 50 updates/sec

    def __init__(self, drivetrain, fieldLayoutFile: str, ignoreTagIDs=(), importantTagIDs=()):
        super().__init__()

        self.cameras = {}  # empty list of cameras, until cameras are added using addPhotonCamera
        self.drivetrain = drivetrain
        self.fieldDrawing: Field2d = drivetrain.field

        # enabled or not?
        self.enabled = SendableChooser()
        self.enabled.setDefaultOption("off", None)
        self.enabled.addOption("demo", False)
        self.enabled.addOption("on", True)
        SmartDashboard.putData("Localizer", self.enabled)

        from os.path import isfile
        assert isfile(fieldLayoutFile), f"file with field layout {fieldLayoutFile} does not exist"
        self.fieldLayout = AprilTagFieldLayout(fieldLayoutFile)
        print(f"Localizer: loaded field layout with {len(self.fieldLayout.getTags())} tags, from {fieldLayoutFile}")

        self.ignoreTagIDs = set(ignoreTagIDs)
        self.importantTagIDs = set(importantTagIDs)
        self.skippedTags = set()

    def addPhotonCamera(self, name, directionDegrees):
        """
        :param name: name of the camera in PhotonVision
        :param directionDegrees: how many degrees to the left (front camera = 0, left camera = 90, right cam = -90 etc.)
        :return: None
        """
        assert name not in self.cameras, f"camera '{name}' was already added"
        photonCamera = PhotonCamera(name)
        self.cameras[name] = CameraState(name, directionDegrees, photonCamera, 0, [], 0, deque())

    def periodic(self):
        now = Timer.getFPGATimestamp()
        robotPose = self.drivetrain.getPose()
        enabled = self.enabled.getSelected()

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
            cameraFrameTime = detections.getTimestamp()
            if camera.lastProcessedCameraTime == cameraFrameTime:
                continue  # skip! we already saw this camera frame result
            camera.lastProcessedCameraTime = cameraFrameTime

            # 2. should we redraw the detected tags from on Shuffleboard/Elastic?
            redrawing = False
            if (now - camera.lastRedrawTime) * Localizer.REDRAW_DASHBOARD_FREQUENCY > 1:
                camera.lastRedrawTime = now
                redrawing = True

            # 3. find where the robot was at the time of this camera's last frame
            recentRobotPose = robotPose
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

                odometryAdjustment = self.calculateOdometryAdjustment(
                    camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition)

                if enabled and (odometryAdjustment is not None):
                    shift, turn = odometryAdjustment
                    self.drivetrain.adjustOdometry(shift, turn)

            # 5. finally, finish the drawing of detected tags on the graphical interface
            if redrawing:
                self.eraseUnusedLastDrawnTags(camera, tagsSeen)
                camera.lastDrawnTags = tagsSeen

        skipped = ",".join(str(t) for t in self.skippedTags) if self.skippedTags else ""
        SmartDashboard.putString("LocalizerSkipped", skipped)

    def eraseUnusedLastDrawnTags(self, camera, tagsSeen):
        for tagId in camera.lastDrawnTags:
            if tagId not in tagsSeen:
                lineToTag, lineFromTag = self.tagLineNames(camera.name, tagId)
                self.fieldDrawing.getObject(lineToTag).setPoses([])
                self.fieldDrawing.getObject(lineFromTag).setPoses([])

    def calculateOdometryAdjustment(self, camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition):
        """
        Some geometry work to convert the observed tag yaw angle into the adjustments to robot (X, Y)
        """
        tagId = tag.getFiducialId()
        tagLocation2d = tagFieldPosition.toPose2d().translation()
        distanceToTag = tagLocation2d.distance(robotLocation2d)

        # now let's do the math:
        #  - say robot is already facing 30 degrees left and left camera is pointing 90 degrees further left
        #  - but tag X is actually seen 10 degrees to the right of that (on that camera frame)
        #
        #  ...then, of course, the tag is really 90 + 30 - 10 = 110 degrees to the left, correct?
        observedTagDirectionDegrees = camera.directionDegrees + robotDirection2d.degrees() - tag.getYaw()

        observedDirectionToTag = Rotation2d.fromDegrees(observedTagDirectionDegrees)
        observedVectorToTag = Translation2d(distanceToTag, 0).rotateBy(observedDirectionToTag)

        # but if odometry says that this vector below is our direction to tag
        odometryVectorToTag = tagLocation2d - robotLocation2d

        # so, how much do we need to adjust the odometry (X, Y) to match the direction that we see?
        shiftMeters = odometryVectorToTag - observedVectorToTag

        # or, if we are allowed to turn the robot heading, how much would we need to turn it?
        turnDegrees = odometryVectorToTag.angle().degrees() - observedVectorToTag.angle().degrees()
        turnDegrees = Rotation2d.fromDegrees((turnDegrees + 180) % 360 - 180)  # avoid +350 degrees if it's really -10

        # use "learning rate" (for example, 0.15) to apply this X, Y adjustment only partially (+do it again next time)
        learningRate = Localizer.LEARNING_RATE
        if tagId in self.importantTagIDs:
            learningRate = learningRate * Localizer.IMPORTANT_TAG_WEIGHT_FACTOR

        skipping = False
        if abs(turnDegrees.degrees()) > Localizer.MAX_ANGULAR_DEVIATION_DEGREES:
            skipping = True

        # do we need to redraw the lines on the screen?
        if redrawing:
            lineToTag, lineFromTag = self.tagLineNames(camera.name, tagId)

            #  - if we have zero trust in this tag, draw a line that says we ignore it
            if skipping:
                self.fieldDrawing.getObject(lineToTag).setPoses(drawLine(1.5, robotLocation2d, tagLocation2d))
                self.fieldDrawing.getObject(lineFromTag).setPoses([])
            else:
                #  - a line going from odometry position to the tag
                lineFromOdometryPositionToTag = drawLine(5, robotLocation2d, tagLocation2d)
                self.fieldDrawing.getObject(lineToTag).setPoses(lineFromOdometryPositionToTag)
                #  - and line going back from the tag to robot's real position, using real actual tag direction from camera
                lineFromTagToRealPosition = drawLine(5, tagLocation2d, tagLocation2d - observedVectorToTag)
                self.fieldDrawing.getObject(lineFromTag).setPoses(lineFromTagToRealPosition)

        # update skipped tags
        if skipping:
            self.skippedTags.add(tagId)
            return
        if tagId in self.skippedTags:
            self.skippedTags.remove(tagId)

        # if everything looks good, this is how to adjust the (X, Y) in the drivetrain odometry
        shift = shiftMeters * learningRate
        turn = Rotation2d() if Localizer.TRUST_GYRO_COMPLETELY else turnDegrees * learningRate * 0.25
        # ^^ if we completely trust the gyro, we don't need any turn
        return shift, turn

    def tagLineNames(self, cameraName, tagId):
        toTag = f"{cameraName}-fromtag{tagId}"
        fromTag = f"{cameraName}-totag{tagId}"
        return toTag, fromTag

def drawLine(pointsPerMeter: int, start: Translation2d, end: Translation2d):
    result = []
    direction = end - start
    angle = direction.angle()
    length = direction.norm()
    nPoints = max([3, int(length * pointsPerMeter)])
    for point in range(nPoints + 1):
        percentage = point / nPoints
        result.append(Pose2d(start + direction * percentage, angle))
    return result

```
</details>


## 3. Adding one localizer to actual robot subsystems

Inside of `robotcontainer.py` **__init__** function we need to add something along these lines
(after drivetrain is created):
```python
    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        from subsystems.localizer import Localizer
        self.localizer = Localizer(drivetrain=self.robotDrive, fieldLayoutFile="2024-crescendo.json")

        self.localizer.addPhotonCamera("front_camera", directionDegrees=0)
        self.localizer.addPhotonCamera("right_camera", directionDegrees=-90)  # right = -90 degrees to the left
        # ^^^ here we must add the cameras exactly the way they are called in PhotonVision
...
```