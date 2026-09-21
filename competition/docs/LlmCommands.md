# Using `LlmCommands` in Your Robot Code

`LlmCommands` lets an AI assistant (a large language model, or "LLM") drive your robot by
calling the **commands you already wrote**. You tell it which commands exist, what
parameters they take and what they are supposed to do. The AI reads that list, picks a
command, fills in the parameters, and the robot runs it.

You never write any AI code on the robot. Everything travels over NetworkTables, the same
system Shuffleboard and Glass use.

```
  You type: "drive forward two meters"
        │
        ▼
  ┌──────────────┐   NetworkTables   ┌──────────────────┐
  │  LLM client  │ ───────────────▶  │  LlmCommands     │
  │  (laptop)    │ ◀───────────────  │  (robot code)    │
  └──────────────┘   status/state    └────────┬─────────┘
                                              │ schedules
                                              ▼
                                     drivetrain.driveDistance(2.0)
```

---

## 1. Setup (two lines)

Call `periodic()` once per loop in `Robot.java`. This is what checks NetworkTables for new
requests.

```java
@Override
public void robotPeriodic() {
  CommandScheduler.getInstance().run();
  LlmCommands.getInstance().periodic();   // <-- add this
}
```

That's the only wiring. Everything else is registering commands.

---

## 2. Registering a command

Registration happens in `RobotContainer`, usually in a method called from the constructor.
The simplest possible registration looks like this:

```java
LlmCommands.register("stop_driving")
    .description("Immediately stop all drivetrain motion.")
    .command(m_drivetrain::stopCommand);
```

Three parts, always in this order:

| Part | What it does |
|------|--------------|
| `register("stop_driving")` | The **name** the AI will use. Use `snake_case`. Must be unique. |
| `.description("...")` | Plain English explanation. **The AI reads this**, so be clear about units and directions. |
| `.command(...)` | How to build the WPILib `Command` to run. This must come **last**. |

> **Why `m_drivetrain::stopCommand` and not `m_drivetrain.stopCommand()`?**
> Each run needs a *fresh* command object. The scheduler won't let the same command instance
> be reused in a new composition, so you hand in a way to *make* the command, not the command
> itself.

---

## 3. Adding parameters

Most commands need input, like "how far?" or "which preset?". Add parameters between
`description` and `command`, then read them with the `p` object inside `command`.

```java
LlmCommands.register("arm_to_angle")
    .description("Move the arm to an absolute angle and hold it there. 0 is stowed flat.")
    .doubleParam("degrees", "Target arm angle in degrees.", Arm.kMinAngleDegrees, Arm.kMaxAngleDegrees)
    .command(p -> m_arm.goToAngle(p.getDouble("degrees")));
```

### Parameter types

| Builder method | Reading it back | Notes |
|----------------|-----------------|-------|
| `.doubleParam(name, desc)` | `p.getDouble(name)` | Decimal number |
| `.doubleParam(name, desc, min, max)` | `p.getDouble(name)` | Same, with limits the AI must respect |
| `.integerParam(name, desc)` | `p.getInteger(name)` | Whole number (also has a min/max version) |
| `.booleanParam(name, desc)` | `p.getBoolean(name)` | `true` / `false` |
| `.stringParam(name, desc)` | `p.getString(name)` | Any text |
| `.choiceParam(name, desc, "low", "high")` | `p.getString(name)` | Text limited to the choices you list |

Giving `min`/`max` limits and using `choiceParam` for fixed options is a big help: the AI
sees the limits and won't ask for an arm angle of 9000 degrees.

### Example with a choice

```java
LlmCommands.register("score")
    .description("Score the held game piece at the chosen level, then stow the arm.")
    .choiceParam("level", "Scoring level to use.", "low", "high")
    .command(p ->
        m_arm.goToPreset(p.getString("level"))
            .andThen(m_intake.eject())
            .andThen(m_arm.goToPreset("stowed")));
```

Any command composition you already know (`andThen`, `alongWith`, `Commands.either`, etc.)
works inside `.command(...)`.

---

## 4. Publishing robot state

The AI can only make good decisions if it can *see* the robot. Publish sensor values from your
subsystems' `periodic()` methods with `LlmCommands.publishState`:

```java
@Override
public void periodic() {
  LlmCommands.publishState("arm/angle_degrees", m_angleDegrees);   // double
  LlmCommands.publishState("arm/at_setpoint", atSetpoint());       // boolean
}
```

Use a `subsystem/thing_units` naming pattern like `drivetrain/x_meters` or
`intake/has_game_piece`. The AI can read every published key, and the monitoring features
below use them too.

---

## 5. Safety: timeouts, stall detection and watchdogs

The AI is not always right. These optional settings let the **robot** decide when a command
has gone wrong and stop it, without waiting for the laptop to notice.

```java
LlmCommands.register("turn_by")
    .description("Rotate in place by a relative angle. Positive turns left (counter-clockwise).")
    .doubleParam("degrees", "Relative angle to rotate in degrees.", -360.0, 360.0)
    .timeout(10.0)                                            // (a)
    .track("drivetrain/heading_degrees", "drivetrain/x_meters", "drivetrain/y_meters")  // (b)
    .expected(p -> Map.of("drivetrain/heading_degrees",
                          m_drivetrain.getHeadingDegrees() + p.getDouble("degrees")))    // (c)
    .stallTimeout(1.0)                                        // (d)
    .watchdog(this::turnInPlaceWatchdog)                      // (e)
    .command(p -> ...);
```

| | Method | What it does |
|-|--------|--------------|
| (a) | `.timeout(seconds)` | Cancel the command if it runs longer than this. **Always set one** on anything that moves. |
| (b) | `.track(keys...)` | Which `publishState` keys show this command's progress. The client records them while the command runs. |
| (c) | `.expected(p -> Map.of(...))` | What the tracked values *should* be when the command finishes. Lets both the robot and the AI check "did it actually work?" |
| (d) | `.stallTimeout(seconds)` | Abort if **none** of the tracked numeric keys changes for this long. Only use on commands that should be moving continuously. |
| (e) | `.watchdog(fn)` | Your own custom check, run every loop. Return `null` if all is well, or a short reason string to abort. |
| | `.checkIn(seconds)` | How often the AI should pause and look at progress before deciding to keep waiting or cancel. Useful for long commands. |

### Writing a watchdog

A watchdog is a method that takes an `LlmRun` and returns a `String` (or `null`). `LlmRun`
gives you the parameters, the expected end values, a snapshot of the tracked state from when
the command started, and the live state table:

```java
private String turnInPlaceWatchdog(LlmRun run) {
  double startX = run.startDouble("drivetrain/x_meters");
  double startY = run.startDouble("drivetrain/y_meters");
  double nowX   = run.stateDouble("drivetrain/x_meters");
  double nowY   = run.stateDouble("drivetrain/y_meters");

  double drift = Math.hypot(nowX - startX, nowY - startY);
  if (drift > 0.1) {
    return String.format("robot moved %.2f m while it should be turning in place", drift);
  }
  return null;   // all good
}
```

Handy `LlmRun` methods:

- `run.params()` – the parameters the AI sent
- `run.elapsedSeconds()` – how long the command has been running
- `run.startDouble(key)` – tracked value when the command started
- `run.expectedDouble(key)` – the value you declared in `.expected(...)`
- `run.stateDouble(key)` / `run.stateBoolean(key)` – the value right now

### Rejecting a command before it starts

If the request doesn't make sense (for example it would drive into a wall), throw an exception
inside `.command(...)`. The run is reported to the AI as **rejected** with your message, and
nothing moves:

```java
.command(p -> {
  if (!m_intake.hasGamePiece()) {
    throw new IllegalStateException("no game piece held, nothing to score");
  }
  return scoreSequence(p.getString("level"));
})
```

---

## 6. Avoidance zones (keeping the robot out of places)

You can mark rectangles on the field the robot must not enter. The AI sees them in the robot
state and plans around them. Coordinates are field meters (WPILib convention: x along the
field length, y along the width, heading 0 pointing +x).

```java
// Fence the whole field. Pad by the robot's half-width since the pose is the robot's centre.
LlmCommands.setFieldBounds(0.5, 0.5, fieldLength - 0.5, fieldWidth - 0.5);

// Add a no-go rectangle by giving any two opposite corners.
LlmCommands.addAvoidanceZone("charging_station", 4.0, 1.3, 5.2, 6.7);
```

Zones are only *published* by default. To actually refuse a motion, check before you build
the command:

```java
.command(p -> {
  Translation2d start = m_drivetrain.getPose().getTranslation();
  Translation2d end   = new Translation2d(p.getDouble("x"), p.getDouble("y"));
  AvoidanceZone blocked = LlmCommands.findZoneCrossedBy(start, end);
  if (blocked != null) {
    throw new IllegalStateException("path would enter zone '" + blocked.name() + "'");
  }
  return m_drivetrain.driveToPoint(end);
})
```

Other helpers: `clearAvoidanceZones()` (keeps the field fence), `clearFieldBounds()`,
`getAvoidanceZones()`.

---

## 7. What the AI sees when a command runs

You don't need to handle any of this, but it helps to know. After a run ends the command's
status is one of:

| Status | Meaning |
|--------|---------|
| `finished` | The command ended on its own. Success. |
| `interrupted` | Cancelled: timed out, the AI cancelled it, or the robot was disabled. |
| `aborted` | The robot stopped it because of the stall check or your watchdog. The reason string is passed along. |
| `rejected` | Never started: the robot was disabled, or `.command(...)` / `.expected(...)` threw. |

Commands are **never** run while the robot is disabled. If the AI asks for the same command
again while it is still running, the running one is cancelled and restarted with the new
parameters.

---

## 8. Checklist for a good registration

- [ ] Name is `snake_case` and unique
- [ ] Description says what happens, in what **units**, and which direction is positive
- [ ] Numeric parameters have `min`/`max`; fixed options use `choiceParam`
- [ ] `.timeout(...)` is set on anything that moves
- [ ] `.track(...)` and `.expected(...)` are set so the AI can tell if it worked
- [ ] `.command(...)` is last and builds a *new* command each time
- [ ] Bad requests throw inside `.command(...)` so they are rejected cleanly

---

## Full example

Everything above, together, from `RobotContainer`:

```java
private void configureLlmCommands() {
  LlmCommands.register("arm_to_preset")
      .description("Move the arm to one of its named preset positions.")
      .choiceParam("preset", "Which preset position to move to.", Arm.Preset.names())
      .timeout(5.0)
      .track("arm/angle_degrees", "arm/at_setpoint")
      .expected(p -> Map.of("arm/angle_degrees", Arm.Preset.parse(p.getString("preset")).degrees))
      .stallTimeout(1.0)
      .command(p -> m_arm.goToPreset(p.getString("preset")));

  LlmCommands.register("say")
      .description("Print a message to the robot console.")
      .stringParam("message", "The text to print.")
      .command(p -> Commands.print("[robot says] " + p.getString("message")));
}
```

See [RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java) for the complete
set of example registrations, including drivetrain commands with watchdogs.
