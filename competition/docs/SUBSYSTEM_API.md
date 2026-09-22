# Subsystem Command API

One line per `Command` factory in [src/main/java/frc/robot/subsystems](../src/main/java/frc/robot/subsystems).
Examples use the instance names from [RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java)
(`robot.intake`, `robot.hood`, ...). Every command listed as "sets a value" is a `runOnce` — it finishes
immediately and the subsystem's `periodic()` keeps applying the value until another command changes it.

## IntakeSubsystem — [IntakeSubsystem.java](../src/main/java/frc/robot/subsystems/IntakeSubsystem.java) (`robot.intake`)

| Command | What it does | Example |
| --- | --- | --- |
| `intake()` | Sets rollers to 100% (output is scaled by 0.75 in `periodic`) | `robot.intake.intake()` |
| `intakeSlow()` | Sets rollers to 50% | `robot.intake.intakeSlow()` |
| `outtake()` | Runs rollers backward at 100% | `robot.intake.outtake()` |
| `outtakeDown()` | Runs rollers backward gently (-20%) | `robot.intake.outtakeDown()` |
| `stop()` | Sets rollers to a 10% idle speed (not fully off — holds game pieces) | `robot.intake.stop()` |

## ArmSubsystem — [ArmSubsystem.java](../src/main/java/frc/robot/subsystems/ArmSubsystem.java) (`robot.arm`)

| Command | What it does | Example |
| --- | --- | --- |
| `armUp()` | Holds the arm at 80° | `robot.arm.armUp()` |
| `setAngle(Angle)` | Holds the arm at the given angle using feedforward | `robot.arm.setAngle(Degrees.of(15))` |
| `wiggleArm(Angle, Angle, Time)` | Repeats forever: go to angle 1, wait, go to angle 2, wait | `robot.arm.wiggleArm(Degrees.of(20), Degrees.of(60), Seconds.of(0.5))` |
| `set(double)` | Open-loop duty cycle (-1..1, clamped to ≥ -0.4); cancels the angle setpoint | `robot.arm.set(-0.2)` |

## HoodSubsystem — [HoodSubsystem.java](../src/main/java/frc/robot/subsystems/HoodSubsystem.java) (`robot.hood`)

The hood forces itself back to 0° whenever `isHoodSafeToDeploy()` is false (robot moving and feeder not running).

| Command | What it does | Example |
| --- | --- | --- |
| `hoodDown()` | Sets the hood setpoint to 0° | `robot.hood.hoodDown()` |
| `setAngle(Angle)` | Sets the hood setpoint once | `robot.hood.setAngle(Degrees.of(25))` |
| `setAngle(Supplier<Angle>)` | Continuously tracks a changing angle until interrupted | `robot.hood.setAngle(() -> Degrees.of(getAimAngle()))` |
| `set(double)` | Open-loop duty cycle; cancels the angle setpoint | `robot.hood.set(-0.1)` |
| `bumpUp5Degrees()` | Raises the setpoint 5° above the current angle | `robot.hood.bumpUp5Degrees()` |
| `bumpDown5Degrees()` | Lowers the setpoint 5° below the current angle | `robot.hood.bumpDown5Degrees()` |
| `hoodDownandReset()` | Drives down slowly until stalled (1.5 s timeout), then zeroes the encoder | `robot.hood.hoodDownandReset()` |
| `setEncoderPosition(Angle)` | Declares the current position to be the given angle and holds it | `robot.hood.setEncoderPosition(Degrees.of(0))` |

## ShooterSubsystem — [ShooterSubsystem.java](../src/main/java/frc/robot/subsystems/ShooterSubsystem.java) (`robot.shooterLeft`, `robot.shooterRight`)

Velocity is in RPM; `isAtSetpoint()` is true within ±50.

| Command | What it does | Example |
| --- | --- | --- |
| `stop()` | Sets the velocity setpoint to 0 (motor coasts) | `robot.shooterLeft.stop()` |
| `spinWithSetpoint(double)` | Sets the velocity setpoint once | `robot.shooterLeft.spinWithSetpoint(2800)` |
| `spinWithSetpoint(Supplier<Double>)` | Continuously tracks a changing setpoint until interrupted | `robot.shooterLeft.spinWithSetpoint(() -> NTHelper.getDouble("/shooter/speed", 2800))` |

## FeederSubsystem — [FeederSubsystem.java](../src/main/java/frc/robot/subsystems/FeederSubsystem.java) (`robot.feederLeft`, `robot.feederRight`)

| Command | What it does | Example |
| --- | --- | --- |
| `spin()` | Runs the feeder at 100% | `robot.feederLeft.spin()` |
| `stop()` | Stops the feeder | `robot.feederLeft.stop()` |

## TwindexerSubsystem — [TwindexerSubsystem.java](../src/main/java/frc/robot/subsystems/TwindexerSubsystem.java) (`robot.twindexer`)

| Command | What it does | Example |
| --- | --- | --- |
| `spindex()` | Runs the indexer forward at 100% | `robot.twindexer.spindex()` |
| `spindexBack()` | Runs the indexer backward at 100% (unjam) | `robot.twindexer.spindexBack()` |
| `stop()` | Stops the indexer | `robot.twindexer.stop()` |

## CommandSwerveDrivetrain — [CommandSwerveDrivetrain.java](../src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) (`robot.drivetrain`)

| Command | What it does | Example |
| --- | --- | --- |
| `applyRequest(Supplier<SwerveRequest>)` | Applies a CTRE swerve request every loop until interrupted | `robot.drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())` |
| `driveToDistanceCommand(meters, speed)` | Drives straight (robot-centric) until the pose has moved `meters`; negative meters = backward; brakes at the end | `robot.drivetrain.driveToDistanceCommand(2.0, 1.5)` |
| `sysIdQuasistatic(Direction)` | SysId quasistatic characterization run | `robot.drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)` |
| `sysIdDynamic(Direction)` | SysId dynamic characterization run | `robot.drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)` |

## BLine (path following) — [BLine.java](../src/main/java/frc/robot/subsystems/BLine.java) (`robot.bline`)

| Command | What it does | Example |
| --- | --- | --- |
| `goToPose(Pose2d)` | Follows a one-waypoint BLine path to the pose (0.1 m end tolerance) | `robot.bline.goToPose(new Pose2d(3, 2, Rotation2d.kZero))` |
| `goToNearestPose(Pose2d[])` | Picks the closest pose when the command starts, then goes there | `robot.bline.goToNearestPose(new Pose2d[] { poseA, poseB })` |

## DriveShortestPath — [DriveShortestPath.java](../src/main/java/frc/robot/subsystems/DriveShortestPath.java) (`robot.driveShortestPath`)

| Command | What it does | Example |
| --- | --- | --- |
| `driveShortestPath(Pose2d)` | Drives to the pose routing through the trenches only (assumes not currently on a bump or in a trench) | `robot.driveShortestPath.driveShortestPath(pose)` |
| `driveShortestPath(Pose2d, goesOverBumps, flipForRedAlliance)` | Same, optionally routing over bumps. The pose is flipped automatically when on the red alliance (`flipForRedAlliance` is currently unused) | `robot.driveShortestPath.driveShortestPath(pose, true, false)` |

## KwarqsLed — [LEDS/KwarqsLed.java](../src/main/java/frc/robot/subsystems/LEDS/KwarqsLed.java) (`robot.kwarqsLed`)

All LED commands run until interrupted and work while the robot is disabled. `disable()` is the default command.

| Command | What it does | Example |
| --- | --- | --- |
| `setLeds(String)` | Shows a named pattern (`yellow`, `orange`, `purple`, `green`, `rainbow`, `dark`, `GreenCycle`, `RedCycle`, `BlueCycle`, `POOP`, `AutoDown`, `YellowAndGreenCycle`) | `robot.kwarqsLed.setLeds("rainbow")` |
| `setLeds(Supplier<String>)` | Same, but the name is read from a supplier (note: the command name is fixed at creation) | `robot.kwarqsLed.setLeds(() -> hasPiece ? "green" : "dark")` |
| `disable()` | LEDs off (`dark`) | `robot.kwarqsLed.disable()` |
| `setYellow()` | Solid yellow | `robot.kwarqsLed.setYellow()` |
| `setOrange()` | Solid orange | `robot.kwarqsLed.setOrange()` |
| `setPurple()` | Solid purple | `robot.kwarqsLed.setPurple()` |
| `setGreen()` | Solid green | `robot.kwarqsLed.setGreen()` |
| `setRainbow()` | Rainbow pattern | `robot.kwarqsLed.setRainbow()` |
| `setRedCycle()` | Red cycle pattern | `robot.kwarqsLed.setRedCycle()` |
| `setBlueCycle()` | Blue cycle pattern | `robot.kwarqsLed.setBlueCycle()` |
| `setGreenCycle()` | Green cycle pattern | `robot.kwarqsLed.setGreenCycle()` |
| `YellowAndGreenCycle()` | Yellow-and-green cycle pattern | `robot.kwarqsLed.YellowAndGreenCycle()` |
| `setAutoDown()` | "AutoDown" pattern | `robot.kwarqsLed.setAutoDown()` |

## DAS (distance / angle / speed table) — [DAS.java](../src/main/java/frc/robot/subsystems/DAS.java)

Not a `Subsystem`; the shared instance is `ShooterCommands.das`.

| Command | What it does | Example |
| --- | --- | --- |
| `increaseVelocityOffset()` | Adds 50 RPM to the shot-speed offset and publishes it to `/tuning/velocityOffset` | `ShooterCommands.das.increaseVelocityOffset()` |
| `decreaseVelocityOffset()` | Subtracts 50 RPM from the shot-speed offset | `ShooterCommands.das.decreaseVelocityOffset()` |
