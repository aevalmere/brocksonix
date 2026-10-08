# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An FTC (FIRST Tech Challenge) robot controller Android project for the 2026-2027 season, forked from the Pedro Pathing Quickstart, which itself tracks the official FTC SDK. Remotes: `origin` = `aevalmere/brocksonix` (the team repo), `upstream` = `Pedro-Pathing/Quickstart`. Builds a single APK that is deployed to the robot's Control Hub.

## Build

Uses the Gradle wrapper (Android Gradle Plugin 8.13, compileSdk 34, Java 8 source level, so no newer Java language features or APIs beyond what Android desugaring provides).

```
./gradlew :TeamCode:assembleDebug        # build the APK
./gradlew :TeamCode:installDebug         # build and install to a connected Control Hub (adb)
./gradlew :TeamCode:testDebugUnitTest    # JUnit tests on the desktop (add --offline when the libraries are cached)
```

JUnit 4 tests live in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/agateflow/` (same package as AgateFlow, so they reach its package-private parts). They plan and drive on `TestRobot`, Pedro's real `Follower` and `Foresight` on a simulated drivetrain with a virtual clock, and check results with their own `TrueRule`. Results go to `TeamCode/build/test-results/`, and pictures of plans to `TeamCode/build/agateflow-svg/`. No linters. Everything else is tested on the robot: the Pedro tuners and the `Tests` procedure (Hold, Line, Curve, Localization, Odometry, etc.).

## Layout

- `FtcRobotController/`: SDK app module. Treat as upstream; don't modify. Its `external/samples` folder holds example OpModes to copy from.
- `TeamCode/`: all team code. `TeamCode/build.gradle` pulls in `build.common.gradle` (shared Android config, don't edit) and `build.dependencies.gradle` (where libraries are added).
- `build.dependencies.gradle`: FTC SDK 12.0.0 artifacts plus Pedro libraries from `https://repo.dairy.foundation/releases/`: `com.pedropathing:revhub`, `com.pedropathing:tuning`, and `com.pedropathing.ivy:pedro` (Ivy command scheduler, 1.1.1); Panels (`com.bylazar:fullpanels`) from `https://mymaven.bylazar.com/releases`.
- `ROBOT.md` (hardware), `TELEOP_PLAN.md` (controls, warnings, field coordinates), `TUNING_GUIDE.md` (how to measure/tune, by step), `TODO_LIST.md` (generated, see below), `SOFTWARE_INNOVATIONS.md` (what is new in our software and how it works, for judges). Design notes live in the comment at the top of each class, not in a separate doc.

## Robot code (`teamcode/`)

`teamcode/` is `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Every folder, sub-folders included, is its own Java package:

- `opmodes/`: only OpModes (what the Driver Station lists): `teleop/` (`MainTeleOp`), `auto/` (`AutoTemplate`), `test/`. Test OpModes build single subsystems instead of `Robot`.
- `driver/`: what the drivers see and feel, used by the match OpModes. `AllianceSelector` picks the alliance in init. `Feedback` runs the gamepad rumble and light. It has only two rumbles. Warning: 3 strong pulses when a warning appears (`OUT_OF_RANGE` is screen-only). Full: a soft constant rumble while storage is full.
- `robot/`: the robot itself. `Robot` owns all subsystems and holds the per-loop decision logic in `update()`; OpModes only map buttons (TeleOp) or schedule commands (auto) onto it. Nothing fires without an alliance and a trusted pose (`Robot.canFire()`).
  - `FireControl` decides when each ball fires.
  - `Warning` enum plus `Robot.warnings()` is the single source for driver warnings. Telemetry prints them and `Feedback` buzzes. `Warning` stays next to `Robot`, not in `driver/`, so a new warning is added in one folder and `robot/` never depends on `driver/`.
  - The `Shooter` and `Turret` watchdogs latch off until the OpMode restarts (`SHOOTER_OFF` / `TURRET_OFF` warnings). The turret's stuck check first tests (pushes harder to rule out friction), then rests and retries, and latches only after 3 failed tests in a row, so a turret that is held for a few seconds comes back. Under `KS` it cuts the push to 0 once the angle stands still, so an encoder that freezes mid-move can't leave the turret creeping. Wrong `REVERSED` flags are caught by the wrong-way check (the turret turns against its own push), within about 1.5 s (about a third of a second when the target is near) even while the robot turns; the full-power check stays as a backup.
  - A NaN pose blocks aiming, but a burst that is already firing keeps the last good RPM, turret target and feed power, so the ball being fed isn't ruined.
  - `RobotState` carries the alliance, target cell and pose from one OpMode to the next. `Robot.update()` saves the pose only when it is trusted and not NaN. Switching alliance in init drops the saved pose, since it is in the other alliance's frame.
  - `subsystems/`: one class per mechanism, each with `addTelemetry()`. Most take a `HardwareMap`, work on their own, and have `update()` (reads/writes hardware). Exceptions: `Drive` takes a Pedro `Follower`; `Rail` has no `update()` and writes inside `collect`, `feed`, `reverse` and `stop`; `Shooter` and `Turret` also take a `VoltageCache` (`Robot.voltage`).
  - `commands/`: Ivy commands for autonomous, `RobotCommands` and `Retry` (see Architecture). AgateFlow's commands are in `agateflow/`.
  - `shot/`: `ShotSolver` (aim + RPM, layered SOTM with on/off switches) and `ShotTables` (distance lookup tables).
  - `hardware/`: hardware access. `HardwareNames` (configuration names), `BulkReads`, `CachedMotor` / `CachedServo` / `CachedCRServo` (write only on change), `VoltageCache`. Keep writes cached, call `BulkReads.clear()` once per loop, and never read motor current in the match loop.
- `agateflow/`: AgateFlow, the on-the-fly path planner, built but not used by any match OpMode yet. `Route` (chain of `to`/`through` waypoints, written for red) goes into `AgateFlow.plan`, which returns a `Plan` of `Leg`s, one Pedro `Path` per stop.
  - How it works: visibility graph around `Zone`s grown by the robot radius, then `CornerCurve` G2 corners, then `MotionModel` simulates Foresight (its look-ahead cuts inside curves, gains read from the follower) on the `DriveModel` physics in 10 ms steps, with battery sag. Speed-limit Modifiers (`maxVelocityConstraint`) go where the predicted path would get too close. It picks the fastest safe candidate. If none keeps the whole `SAFETY_MARGIN`, it takes the quickest one that misses by less than `NEAR_MISS_FRACTION` of it and still clears `ROBOT_RADIUS`, with a note in the `Plan`. Each class's top comment has the details.
  - The robot is its outline (`ROBOT_FRONT`, `ROBOT_BACK`, `ROBOT_HALF_WIDTH` around the turret center), checked on both the planned and the predicted path. Any part may touch the 4 field walls (`WALL_GAP`, 0), but never moves into one faster than `WALL_CONTACT_SPEED`. Along a path every other zone keeps `SAFETY_MARGIN` from the whole outline; near the start and end the robot may come into the margin, but its outline never goes into a zone or a wall (except the flower intake reaching into a FLOWER's opening on purpose, `Route.allowContact()` stretches, which use the `CONTACT_RADIUS` circle, and a start that already overlaps). Where the robot doesn't fit, AgateFlow says so in the `Plan` notes: it moves a waypoint with its own heading until the outline is 0.5 in. clear, leaves out a pickUp target it can't reach (`Plan.skipped()`), and refuses a turn in place that would swing a corner into something. A waypoint without a heading (`to(x, y)`) may arrive side-on or backward if that fits. Next to a wall it may keep its start heading for up to `MAX_START_HOLD` in., or finish a turn up to `MAX_END_HOLD` in. before a waypoint. AgateFlow never changes a heading the route asks for, and a fixed or linear heading crosses gaps at an angle (wider across the path), so tight places want `Heading.tangent()`, no heading, or a through-waypoint before the gap.
  - `Route.pickUp(poses)` (`.most(n)`, `.inOrder()`, `pickUpLive`) drives over floor targets intake first, in the fastest order (every order up to 7 groups; beyond that Held-Karp gives a near-best one, usually the best, under 10% off at worst; see `Pickup`). `Plan.pickedUp()` / `skipped()` say which it gets.
  - `Route.flowerIntakeDown()` plans that stretch with the front at `IntakePoses.FLOWER_INTAKE_REACH`. `RobotCommands.withFlowerIntake(drive)` lowers and raises the intake to match (`DriveRoute.flowerIntakeDown()`). Every other stretch is planned with it up, except the first `FLOWER_INTAKE_RAISE_RUN` in. after a down stretch, while it comes up (the robot backs straight out and keeps its heading there). Between drives, `RobotCommands.flowerUp()` raises it and waits `FLOWER_RAISE_MS`.
  - `FieldMap` holds zones by source: walls always, `FieldZones` (the HIVE frame ends and the 4 FLOWERS), and vision via `setZones("vision", ...)`, which is safe from a camera thread and drops null or NaN zones.
  - `AgateFlowCommands.driveTo(route)` gives an Ivy `DriveRoute`. It plans on a background thread and replans when pushed, or when new zones take room from the path ahead (not near stops, where it's meant to get close). Every loop while the zones differ from the plan's, it checks the predicted path 30 in. ahead (along the path): a new zone there makes it brake straight against its motion (`STOP_BRAKING_POWER`) until nearly still, hold, and replan from there. A replan that comes back after the robot moved on to the next leg is dropped. It gives up `TIMEOUT_SCALE` x its first estimate + `TIMEOUT_EXTRA_SECONDS` after the first plan; replans can add at most `REPLAN_EXTRA_SECONDS` more, except a detour around a new zone, which gets its own plan's time (at most `MAX_DETOURS` times). A robot still getting closer at the deadline gets up to `MAX_OVERTIME_SECONDS` more. `EtaCalibration` learns real/predicted time.
  - `Route.toLive` suppliers are asked on the main thread (in `AgateFlowCommands.planLater`) at the start, every replan and every estimate, never on the planning thread. Live poses are field coordinates, but a `Heading` (like `facing`) is always written for red.
  - `AgateFlowCommands.estimate(route)` times a route in the background without driving it. `driveTo(estimate)` starts with that plan, without waiting, only while it still fits: the robot is within 1 in. and 5° of where the estimate was made, or the estimate is under 0.5 s old and the robot is on its predicted path. Otherwise it plans again. It replans while driving like any `DriveRoute`.
  - `ParkGuard`: call `update()` each auto loop before `Scheduler.execute()`. When the time left reaches the current park estimate plus a margin, it schedules the park drive at high priority, requiring every subsystem passed in, which interrupts whatever auto command is running. Build it with `RobotCommands.parkGuard(...)`, which passes every subsystem and holds the flower intake up for the whole park drive (AgateFlow sizes the robot with it up).
  - It never produces hairpins (Foresight can stick on them): a through-waypoint that needs one becomes a stop, with a note in the `Plan`.
  - `PlanDrawing.toPanels(plan, map, pose)` draws zones, the planned path, the predicted path, robot outlines and pickups on the Panels field page (PEDRO_PATHING preset). It sends by itself, at most every 100 ms. Its `TO_PANELS` (144 / `Field.SIZE`) is for drawing only; planning never uses 144. `toSvg` is for the tests.
  - `DriverAssist`: one-button TeleOp drive along a `Route`; the sticks take over at once. Not in `MainTeleOp`.
  - The `AgateFlow Benchmark` test OpMode (`opmodes/test/`) times planning on the Control Hub without moving, then shows each plan on Panels (D-pad picks it).
  - Validation is desktop-only so far: the JUnit tests, plus random fields against Pedro's real `Foresight` in closed loop on a simulated drivetrain. It has not run on the robot.
  - One package on purpose: Java's package-private access only works inside one package, and a sub-folder is a separate package. `VisibilityGraph`, `ClearanceRule`, `MotionModel`, `Track` and `Geometry` are internal, and `Route`, `Waypoint`, `Heading`, `Zone`, `Leg` and `Estimate` have package-private parts. `AgateFlowCommands`, `DriveRoute` and `ParkGuard` use those parts, so they stay here, not in `robot/commands/`. Don't make internals public just to split the folder.
- `field/`: field coordinates in inches, Pedro frame: origin at the red-wall/audience-wall corner, +x toward blue, +y away from the audience. Write poses for red; `Alliance.fromRed` flips them for blue (180° spin about the center). `FieldZones` (AgateFlow's keep-out zones) and `IntakePoses` (auto pickup spots) are here too.
- `pedro/`: Pedro Pathing setup and tuners (see Architecture).
- `util/`: plain helpers with no hardware: `Angles`, `LookupTable`, `Debouncer`, `DriveCurve` (stick shaping).
- Panels: every tunable is a non-final `public static` field in a `@Configurable` class and must take effect live. Never read one only in a constructor, and apply direction flags when writing the power or position, not with `setDirection`. The shot tables (Panels edits values, not row counts; distances must stay increasing), `MainTeleOp` and the auto poses are in Panels too. Copy final values back into the code, since edits are lost on app restart. A Panels page is named after its class (not its folder), so renaming a `@Configurable` class renames its page.
- Anything that needs measuring or tuning gets a `// TODO(step): what to do` comment right above it, in exactly that form (anything else, like `// TODO: x`, is left out of the list with a build warning). The step is the `TUNING_GUIDE.md` section number, 1 to 12 (12 is Autonomous). The `todoList` Gradle task (runs before every build) regenerates `TODO_LIST.md` from these. Don't edit `TODO_LIST.md` by hand.
- Code style: written for students to read. Plain names, short comments only where the why isn't obvious, no clever abstractions.

## Architecture

**OpModes** are discovered by annotation (`@TeleOp` / `@Autonomous`) via the SDK's annotation processor; no registration needed. Use `LinearOpMode` with a `while (opModeIsActive())` loop. Match OpModes use `driver/AllianceSelector` in init.

**Pedro Pathing 3.x (`teamcode/pedro/`)**
- Upstream's files: `Constants.java`, `Tuning.java` (an empty class; leave it alone) and every tuner in `procedures/` except `DriveModelTuner`. Keep their paths and names, or every upstream merge conflicts. `TeamTuning` and `DriveModelTuner` are ours.
- `Constants.create(HardwareMap)` is the single factory that builds the `Follower` from three parts: a `Drivetrain`, a `Localizer`, and the path-following `Algorithm` (Foresight). It still returns `null` and must be filled in with config objects (e.g. `MecanumConfig`, `PinpointConfig`, `ForesightConfig`) produced by the tuners. Until then `Robot` throws a clear error at init.
  - The ForesightTuner output goes in a `foresightConfig()` method that builds a new `ForesightConfig` on every call, not in a `static` field (the comment in `Constants` shows how). A path's speed-limit `Modifier` writes into the config and is only put back when the follower lets go of the path, so a shared config could keep a limit after an OpMode stops mid-path or crashes.
- Tuners are registered in `TeamTuning.java`: the tuning menu scans every class under `org.firstinspires.ftc.teamcode` for static no-arg `@Tuner` methods returning `Procedure`. `ForesightTuner` gets added there after the Mecanum and Pinpoint output is pasted into `Constants`, and `Tests` after the Foresight output is too (the code to add is in a comment).
- `procedures/` contains the tuners. Each is a `com.pedropathing.tuning.autotune.Procedure` whose `run()` drives an interactive flow (`inputs(...)`/`awaitInputs`, `confirmation`, `runOpMode(new SomeTuningOpMode(...))`, `result`, `abort`) and ends by emitting a Java snippet via `code(Language.JAVA, ...)` to paste into `Constants`. Individual robot motions are package-private `TuningOpMode<T>` subclasses in the same file that return the measured value.
  - Hardware setup tuners (`MecanumTuner`, `PinpointTuner`, `OTOSTuner`, `OctoQuadTuner`, `TwoWheelTuner`, `ThreeWheelTuner`, `ThreeWheelIMUTuner`) take no constructor args.
  - `ForesightTuner`, `DriveModelTuner` and `Tests` take `Function<HardwareMap, Localizer>` / `Function<HardwareMap, Drivetrain>` (and `Tests` a `Supplier<Algorithm>`) so they can run before a full `Follower` exists; `Tests` builds a `Follower` only when all three are non-null. `DriveModelTuner` (ours) measures the numbers `agateflow/DriveModel` needs; it emits code for `DriveModel`, not `Constants`.
  - Tuners must keep commanding drive powers every loop iteration (not set once) so they work with swerve drivetrains, and use `Thread.sleep` pauses between phases to let the robot settle.

**Ivy (`robot/commands/`)**: used for autonomous only.
- `RobotCommands` builds commands that set what `Robot` should do (`shootHeld`, `unjamHeld`, `flowerIntake.setDown`, `singleBallBursts`) and wait on what it reports. They never drive hardware directly. `withFlowerIntake(drive)` runs an AgateFlow drive with the flower intake following its plan.
- `Retry` reruns a command while a check fails, up to a limit (`shootAllWithRetry` uses it). It ends each try exactly once, so use it instead of building retries out of Ivy groups.
- The auto loop is `Scheduler.execute()`, then `robot.update()`. `Scheduler` is static, so call `Scheduler.reset()` first. After the loop, call `robot.stop()`: it stops the follower, so Pedro puts back any path speed limit.
- `AutoTemplate` (`@Disabled`) is the skeleton to copy. Its poses are written for red; `Robot.setStartPose` and `driveTo` flip them for blue.
- TeleOp does not use commands. Its loop still runs buttons, then `Scheduler.execute()`, then `robot.update()`, as in auto, so one-button macros can be added later. `driverOne` sets `shootHeld`, `passHeld` and `unjamHeld` from the buttons every loop, so a macro that sets one of them must be OR-ed in there.
- Ivy 1.1.1 gotchas:
  - Groups can call `end()` on a child that never started, or twice, so cleanup in `setEnd` must be safe to repeat (see `RobotCommands.stopShooting`).
  - Scheduling a second top-level command that shares a requirement cancels the one already running, so keep a routine in one `sequential(...)`.
  - `Command.unless()` never finishes (it uses `NOOP`). Avoid it.
  - `Scheduler.reset()` does not call `end()`.

## Upstream merges

History includes merges from the official FTC SDK and the Pedro Quickstart (`upstream`). Keep team changes inside `TeamCode/` and `build.dependencies.gradle` to keep merges clean.
