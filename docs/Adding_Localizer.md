# Adding a Localizer (automatic odometry X,Y adjustments from cameras looking at tags)

## 0. Installing dependencies
In `requirements.txt` we need to add three more lines (dependencies):
```python
pyntcore
photonlibpy
robotpy-apriltag
```
, so these two libraries will need to be installed.

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

And for 2025 it will be this `2025-reefscape.json`:
<details>
    <summary>(click to expand)</summary>

```json
    
{
  "tags": [
    {
      "ID": 1,
      "pose": {
        "translation": {
          "x": 16.697198,
          "y": 0.65532,
          "z": 1.4859
        },
        "rotation": {
          "quaternion": {
            "W": 0.4539904997395468,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.8910065241883678
          }
        }
      }
    },
    {
      "ID": 2,
      "pose": {
        "translation": {
          "x": 16.697198,
          "y": 7.3964799999999995,
          "z": 1.4859
        },
        "rotation": {
          "quaternion": {
            "W": -0.45399049973954675,
            "X": -0.0,
            "Y": 0.0,
            "Z": 0.8910065241883679
          }
        }
      }
    },
    {
      "ID": 3,
      "pose": {
        "translation": {
          "x": 11.560809999999998,
          "y": 8.05561,
          "z": 1.30175
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
      "ID": 4,
      "pose": {
        "translation": {
          "x": 9.276079999999999,
          "y": 6.137656,
          "z": 1.8679160000000001
        },
        "rotation": {
          "quaternion": {
            "W": 0.9659258262890683,
            "X": 0.0,
            "Y": 0.25881904510252074,
            "Z": 0.0
          }
        }
      }
    },
    {
      "ID": 5,
      "pose": {
        "translation": {
          "x": 9.276079999999999,
          "y": 1.914906,
          "z": 1.8679160000000001
        },
        "rotation": {
          "quaternion": {
            "W": 0.9659258262890683,
            "X": 0.0,
            "Y": 0.25881904510252074,
            "Z": 0.0
          }
        }
      }
    },
    {
      "ID": 6,
      "pose": {
        "translation": {
          "x": 13.474446,
          "y": 3.3063179999999996,
          "z": 0.308102
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
      "ID": 7,
      "pose": {
        "translation": {
          "x": 13.890498,
          "y": 4.0259,
          "z": 0.308102
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
          "x": 13.474446,
          "y": 4.745482,
          "z": 0.308102
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
      "ID": 9,
      "pose": {
        "translation": {
          "x": 12.643358,
          "y": 4.745482,
          "z": 0.308102
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
      "ID": 10,
      "pose": {
        "translation": {
          "x": 12.227305999999999,
          "y": 4.0259,
          "z": 0.308102
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
      "ID": 11,
      "pose": {
        "translation": {
          "x": 12.643358,
          "y": 3.3063179999999996,
          "z": 0.308102
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
    },
    {
      "ID": 12,
      "pose": {
        "translation": {
          "x": 0.851154,
          "y": 0.65532,
          "z": 1.4859
        },
        "rotation": {
          "quaternion": {
            "W": 0.8910065241883679,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.45399049973954675
          }
        }
      }
    },
    {
      "ID": 13,
      "pose": {
        "translation": {
          "x": 0.851154,
          "y": 7.3964799999999995,
          "z": 1.4859
        },
        "rotation": {
          "quaternion": {
            "W": -0.8910065241883678,
            "X": -0.0,
            "Y": 0.0,
            "Z": 0.45399049973954686
          }
        }
      }
    },
    {
      "ID": 14,
      "pose": {
        "translation": {
          "x": 8.272272,
          "y": 6.137656,
          "z": 1.8679160000000001
        },
        "rotation": {
          "quaternion": {
            "W": 5.914589856893349e-17,
            "X": -0.25881904510252074,
            "Y": 1.5848095757158825e-17,
            "Z": 0.9659258262890683
          }
        }
      }
    },
    {
      "ID": 15,
      "pose": {
        "translation": {
          "x": 8.272272,
          "y": 1.914906,
          "z": 1.8679160000000001
        },
        "rotation": {
          "quaternion": {
            "W": 5.914589856893349e-17,
            "X": -0.25881904510252074,
            "Y": 1.5848095757158825e-17,
            "Z": 0.9659258262890683
          }
        }
      }
    },
    {
      "ID": 16,
      "pose": {
        "translation": {
          "x": 5.9875419999999995,
          "y": -0.0038099999999999996,
          "z": 1.30175
        },
        "rotation": {
          "quaternion": {
            "W": 0.7071067811865476,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.7071067811865476
          }
        }
      }
    },
    {
      "ID": 17,
      "pose": {
        "translation": {
          "x": 4.073905999999999,
          "y": 3.3063179999999996,
          "z": 0.308102
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
    },
    {
      "ID": 18,
      "pose": {
        "translation": {
          "x": 3.6576,
          "y": 4.0259,
          "z": 0.308102
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
      "ID": 19,
      "pose": {
        "translation": {
          "x": 4.073905999999999,
          "y": 4.745482,
          "z": 0.308102
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
      "ID": 20,
      "pose": {
        "translation": {
          "x": 4.904739999999999,
          "y": 4.745482,
          "z": 0.308102
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
      "ID": 21,
      "pose": {
        "translation": {
          "x": 5.321046,
          "y": 4.0259,
          "z": 0.308102
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
      "ID": 22,
      "pose": {
        "translation": {
          "x": 4.904739999999999,
          "y": 3.3063179999999996,
          "z": 0.308102
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
    }
  ],
  "field": {
    "length": 17.548,
    "width": 8.052
  }
}

```

</details>

* For other years, go to: https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/


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
    cameraPoseOnRobot: Pose2d
    photonCamera: PhotonCamera
    lastProcessedCameraTime: float
    lastDrawnTags: list
    lastRedrawTime: float
    recentRobotLocations: deque  # (time, pose) tuples


class Localizer(commands2.Subsystem):
    LEARNING_RATE = 0.1  # to converge faster you may want to increase this, but location can become more unstable
    ONLY_WORK_IF_SEEING_MULTIPLE_TAGS = False  # avoid making adjustments to odometry if only one tag is seen

    TRUST_GYRO_COMPLETELY = True  # if you set it =True, odometry heading (North) will never be modified
    MAX_ANGULAR_DEVIATION_DEGREES = 45  # if a tag appears to be more than 45 degrees away, ignore it (something wrong)
    IMPORTANT_TAG_WEIGHT_FACTOR = 2.0  # if a tag is important (for example, located on a shooting target)

    REDRAW_DASHBOARD_FREQUENCY = 5  # how often to redraw the tags in Elastic/Shuffleboard
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

        from os.path import isfile, join

        self.fieldLayout = None
        for prefix in ['/home/lvuser/py/', '/home/lvuser/py_new/', '']:
            candidate = join(prefix, fieldLayoutFile)
            print(f"Localizer: trying field layout from {candidate}")
            if isfile(candidate):
                self.fieldLayout = AprilTagFieldLayout(candidate)
                break
        assert self.fieldLayout is not None, f"file with field layout {fieldLayoutFile} does not exist"
        print(f"Localizer: loaded field layout with {len(self.fieldLayout.getTags())} tags, from {fieldLayoutFile}")

        self.ignoreTagIDs = set(ignoreTagIDs)
        self.importantTagIDs = set(importantTagIDs)
        self.recentlySeenTags = deque()
        self.skippedTags = set()

    def addPhotonCamera(self, name, directionDegrees, positionFromRobotCenter=Translation2d(0, 0)):
        """
        :param name: name of the camera in PhotonVision
        :param directionDegrees: how many degrees to the left (front camera = 0, left camera = 90, right cam = -90 etc.)
        :return: None
        """
        assert name not in self.cameras, f"camera '{name}' was already added"
        photonCamera = PhotonCamera(name)
        cameraPose = Pose2d(positionFromRobotCenter, Rotation2d.fromDegrees(directionDegrees))
        self.cameras[name] = CameraState(name, cameraPose, photonCamera, 0, [], 0, deque())


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
        self.skippedTags.clear()
        now = Timer.getFPGATimestamp()
        robotPose = self.drivetrain.getPose()
        enabled = self.enabled.getSelected()

        while len(self.recentlySeenTags) > 4 * Localizer.MAX_LOCATION_HISTORY_SIZE:
            self.recentlySeenTags.popleft()

        # if saw more multiple tags in the last 0.25s, then apply adjustments (if enabled)
        ready = True
        if Localizer.ONLY_WORK_IF_SEEING_MULTIPLE_TAGS:
            ready = self.recentlySawMoreThanOneTag(now - 0.25)

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

                self.recentlySeenTags.append((now, tagId))

                odometryAdjustment = self.calculateOdometryAdjustment(
                    camera, redrawing, robotDirection2d, robotLocation2d, tag, tagFieldPosition)

                if enabled and ready and (odometryAdjustment is not None):
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
        tagId = tag.getFiducialId()
        tagLocation2d = tagFieldPosition.toPose2d().translation()

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

        # use "learning rate" (for example, 0.05) to partly apply this (X, Y) shift and partly the shift in heading
        learningRate = Localizer.LEARNING_RATE
        if tagId in self.importantTagIDs:
            learningRate = learningRate * Localizer.IMPORTANT_TAG_WEIGHT_FACTOR

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
        # ^^field layouts like "2024-crescendo.json" and "2025-reefscape.json" can be downloaded from
        #  https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/

        self.localizer.addPhotonCamera("front_camera", directionDegrees=0)
        self.localizer.addPhotonCamera("left_camera", directionDegrees=+90)  # left = +90 degrees to the left (right would have been -90)
        # ^^^ here we must add the cameras exactly the way they are called in PhotonVision
...
```


## 4. In Elastic/SmartDashboard

Once added as above, you should be able to add Localizer widget in Elastic, near your field map:
![image](https://github.com/user-attachments/assets/ffecdad0-dd1e-4b08-8d6e-b9442d322778)

, resulting in something like this:
![image](https://github.com/user-attachments/assets/3d6d31e6-ab2c-4b4e-af92-ce3522107204)
