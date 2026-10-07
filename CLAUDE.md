# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An FTC (FIRST Tech Challenge) robot controller Android project for the 2026-2027 season, forked from the Pedro Pathing Quickstart, which itself tracks the official FTC SDK. Remotes: `origin` = `aevalmere/brocksonix` (the team repo), `upstream` = `Pedro-Pathing/Quickstart`. Builds a single APK that is deployed to the robot's Control Hub.

## Build

Uses the Gradle wrapper (Android Gradle Plugin 8.13, compileSdk 34, Java 8 source level, so no newer Java language features or APIs beyond what Android desugaring provides).

```
./gradlew :TeamCode:assembleDebug        # build the APK
./gradlew :TeamCode:installDebug         # build and install to a connected Control Hub (adb)
```

There are no unit tests or linters configured. "Testing" happens on the robot: the Pedro tuners and the `Tests` procedure (Hold, Line, Curve, Localization, Odometry, etc.) are the verification tools.

## Layout

- `FtcRobotController/`: SDK app module. Treat as upstream; don't modify. Its `external/samples` folder holds example OpModes to copy from.
- `TeamCode/`: all team code. `TeamCode/build.gradle` pulls in `build.common.gradle` (shared Android config, don't edit) and `build.dependencies.gradle` (where libraries are added).
- `build.dependencies.gradle`: FTC SDK 12.0.0 artifacts plus Pedro libraries from `https://repo.dairy.foundation/releases/`: `com.pedropathing:revhub`, `com.pedropathing:tuning`, and `com.pedropathing.ivy:pedro` (Ivy command scheduler, 1.1.1); Panels (`com.bylazar:fullpanels`) from `https://mymaven.bylazar.com/releases`.
- `ROBOT.md` (hardware), `TELEOP_PLAN.md` (design, controls, field coordinates), `TUNING_GUIDE.md` (how to measure/tune, by step), `TODO_LIST.md` (generated, see below).

## Robot code (`teamcode/`)

- `field/`: field coordinates in inches, Pedro frame: origin at the red-wall/audience-wall corner, +x toward blue, +y away from the audience. Write poses for red; `Alliance.fromRed` flips them for blue (180° spin about the center).
- `subsystems/`: one class per mechanism, each with `addTelemetry()`. Most take a `HardwareMap`, work on their own, and have `update()` (reads/writes hardware). Exceptions: `Drive` takes a Pedro `Follower`; `Rail` has no `update()` and writes inside `collect`, `feed`, `reverse` and `stop`; `Shooter` and `Turret` also take a `VoltageCache`.
- Panels: every tunable is a non-final `public static` field in a `@Configurable` class and must take effect live. Never read one only in a constructor, and apply direction flags when writing the power or position, not with `setDirection`. The shot tables (Panels edits values, not row counts; distances must stay increasing), `MainTeleOp` and the auto poses are in Panels too. Copy final values back into the code, since edits are lost on app restart.
- `robot/Robot` owns all subsystems and holds the per-loop decision logic in `update()`; OpModes only map buttons (TeleOp) or schedule commands (auto) onto it. Test OpModes in `opmodes/test/` build single subsystems instead of `Robot`. Nothing fires without an alliance and a trusted pose (`Robot.canFire()`).
  - `Warning` enum plus `Robot.warnings()` is the single source for driver warnings. Telemetry prints them and `Feedback` buzzes.
  - `Feedback` has only two rumbles. Warning: 3 strong pulses when a warning appears (`OUT_OF_RANGE` is screen-only). Full: a soft constant rumble while storage is full.
  - The `Shooter` and `Turret` watchdogs latch off until the OpMode restarts (`SHOOTER_OFF` / `TURRET_OFF` warnings).
  - `Robot.update()` saves the pose to `RobotState` only when it is trusted and not NaN. Switching alliance in init drops the saved pose, since it is in the other alliance's frame.
- `shot/`: `ShotSolver` (aim + RPM, layered SOTM with on/off switches) and `ShotTables` (distance lookup tables).
- Keep writes cached (`util/Cached*`), call `BulkReads.clear()` once per loop, and never read motor current in the match loop.
- Anything that needs measuring or tuning gets a `// TODO(step): what to do` comment right above it, in exactly that form (anything else, like `// TODO: x`, is left out of the list with a build warning). The step is the `TUNING_GUIDE.md` section number, 1 to 12 (12 is Autonomous). The `todoList` Gradle task (runs before every build) regenerates `TODO_LIST.md` from these. Don't edit `TODO_LIST.md` by hand.
- Code style: written for students to read. Plain names, short comments only where the why isn't obvious, no clever abstractions.

## Architecture

**OpModes** are discovered by annotation (`@TeleOp` / `@Autonomous`) via the SDK's annotation processor; no registration needed. Use `LinearOpMode` with a `while (opModeIsActive())` loop. Match OpModes use `AllianceSelector` in init.

**Pedro Pathing 3.x (`teamcode/pedro/`)**
- `Constants.create(HardwareMap)` is the single factory that builds the `Follower` from three parts: a `Drivetrain`, a `Localizer`, and the path-following `Algorithm` (Foresight). It still returns `null` and must be filled in with config objects (e.g. `MecanumConfig`, `PinpointConfig`, `ForesightConfig`) produced by the tuners. Until then `Robot` throws a clear error at init.
- Tuners are registered in `TeamTuning.java`: the tuning menu scans every class under `org.firstinspires.ftc.teamcode` for static no-arg `@Tuner` methods returning `Procedure`. `ForesightTuner` gets added there after the Mecanum and Pinpoint output is pasted into `Constants`, and `Tests` after the Foresight output is too (the code to add is in a comment). `Tuning.java` is upstream's empty class; leave it alone.
- `procedures/` contains the tuners. Each is a `com.pedropathing.tuning.autotune.Procedure` whose `run()` drives an interactive flow (`inputs(...)`/`awaitInputs`, `confirmation`, `runOpMode(new SomeTuningOpMode(...))`, `result`, `abort`) and ends by emitting a Java snippet via `code(Language.JAVA, ...)` to paste into `Constants`. Individual robot motions are package-private `TuningOpMode<T>` subclasses in the same file that return the measured value.
  - Hardware setup tuners (`MecanumTuner`, `PinpointTuner`, `OTOSTuner`, `OctoQuadTuner`, `TwoWheelTuner`, `ThreeWheelTuner`, `ThreeWheelIMUTuner`) take no constructor args.
  - `ForesightTuner` and `Tests` take `Function<HardwareMap, Localizer>` / `Function<HardwareMap, Drivetrain>` (and `Tests` a `Supplier<Algorithm>`) so they can run before a full `Follower` exists; `Tests` builds a `Follower` only when all three are non-null.
  - Tuners must keep commanding drive powers every loop iteration (not set once) so they work with swerve drivetrains, and use `Thread.sleep` pauses between phases to let the robot settle.

**Ivy (`robot/RobotCommands.java`)**: used for autonomous only.
- `RobotCommands` builds commands that set what `Robot` should do (`shootHeld`, `unjamHeld`, `flowerIntake.setDown`, `singleBallBursts`) and wait on what it reports. They never drive hardware directly.
- The auto loop is `Scheduler.execute()`, then `robot.update()`. `Scheduler` is static, so call `Scheduler.reset()` first.
- `AutoTemplate` (`@Disabled`) is the skeleton to copy. Its poses are written for red; `Robot.setStartPose` and `driveTo` flip them for blue.
- TeleOp does not use commands. Its `Scheduler` calls stay so one-button macros can be added later (put `Scheduler.execute()` before `robot.update()`, as in auto).
- Ivy 1.1.1 gotchas:
  - Groups can call `end()` on a child that never started, or twice, so cleanup in `setEnd` must be safe to repeat (see `RobotCommands.stopShooting`).
  - Scheduling a second top-level command that shares a requirement cancels the one already running, so keep a routine in one `sequential(...)`.
  - `Command.unless()` never finishes (it uses `NOOP`). Avoid it.
  - `Scheduler.reset()` does not call `end()`.

## Upstream merges

History includes merges from the official FTC SDK and the Pedro Quickstart (`upstream`). Keep team changes inside `TeamCode/` and `build.dependencies.gradle` to keep merges clean.
