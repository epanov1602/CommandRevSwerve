# Manual Driving Command Examples

## 1. resetting robot's (X, Y) coordinates to a known location with a single button click
If you want to reset odometry (position on the field) to some known location, something like this goes to `configureButtonBindings(...)`:

```python3
        from wpimath.geometry import Pose2d, Rotation2d, Translation2d

        # if "start" button pressed, reset X,Y position to the **upper** feeding station (x=1.30, y=6.90, 54 degrees **east**)
        startButton = self.drivingController.button(XboxController.Button.kStart)
        startButton.onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(Pose2d(Translation2d(1.30, 6.90), Rotation2d.fromDegrees(-54)))
            )
        )

        # if "end" button pressed, reset X,Y position to the **lower** feeding station (x=1.30, y=1.15, 54 degrees **west**)
        backButton = self.drivingController.button(XboxController.Button.kBack)
        backButton.onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(Pose2d(Translation2d(1.30, 1.15), Rotation2d.fromDegrees(54)))
            )
        )

        # coordinates above assume robot bumper length=0.9 meters (width does not matter), but if you need to recompute then:
        #  - center of feeding station is x=0.84, y=0.65 (lower) and x=0.84, y=7.40 (upper), heading=+-54 degrees

```

## 2. Aiming using robot video feed? Switch to FPV driving ("first-person view") if left-trigger is pushed more than 50%
Look how the stick input is multiplied by 0.3 everywhere below. Is 0.3 the right percentage for your case?

 * This goes into `configureButtonBindings(...)` in `robotcontainer.py` :

```python3
...
        from commands.holonomicdrive import HolonomicDrive

        # if someone pushes left trigger of driving controller more than 50%
        leftTriggerAsButton = self.drivingController.axisGreaterThan(XboxController.Axis.kLeftTrigger, threshold=0.50)
        # ... then the sticks of the driving controller start driving the robot FPV-style (not field-relative)
        leftTriggerAsButton.whileTrue(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -0.3 * self.drivingController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -0.3 * self.drivingController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -0.3 * self.drivingController.getRawAxis(XboxController.Axis.kRightX),
                deadband=0,
                fieldRelative=False,  # driving FPV (first person view), not field-relative (install an FPV camera on robot?)
                rateLimit=False,
                square=False,
            )
        )
...
```

 * If you want the sticks of the secondary joystick (`self.scoringController`) to be used for this instead, then it should be:
```python3
...
        from commands.holonomicdrive import HolonomicDrive

        # if someone pushes left trigger of scoring controller more than 50%
        leftTriggerAsButton = self.scoringController.axisGreaterThan(XboxController.Axis.kLeftTrigger, threshold=0.50)
        # ... then the sticks of the scoring controller start driving the robot FPV-style (not field-relative)
        leftTriggerAsButton.whileTrue(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -0.3 * self.scoringController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -0.3 * self.scoringController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -0.3 * self.scoringController.getRawAxis(XboxController.Axis.kRightX),
                deadband=0,
                fieldRelative=False,  # driving FPV (first person view), not field-relative (install an FPV camera on robot?)
                rateLimit=False,
                square=False,
            )
        )
...
```
