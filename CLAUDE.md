# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An FTC (FIRST Tech Challenge) robot controller Android project for the 2026-2027 season, forked from the Pedro Pathing Quickstart (`origin` = `Pedro-Pathing/Quickstart`), which itself tracks the official FTC SDK. Builds a single APK that is deployed to the robot's Control Hub.

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
- `build.dependencies.gradle`: FTC SDK 12.0.0 artifacts plus Pedro libraries from `https://repo.dairy.foundation/releases/`: `com.pedropathing:revhub`, `com.pedropathing:tuning`, and `com.pedropathing.ivy:pedro` (Ivy command scheduler); Panels (`com.bylazar:fullpanels`) from `https://mymaven.bylazar.com/releases`.
- `ROBOT.md` (hardware), `TELEOP_PLAN.md` (design, controls, field coordinates), `TUNING_GUIDE.md` (how to measure/tune, by step), `TODO_LIST.md` (generated, see below).

## Robot code (`teamcode/`)

- `field/`: field coordinates in inches, Pedro frame: origin at the red-wall/audience-wall corner, +x toward blue, +y away from the audience. Write poses for red; `Alliance.fromRed` flips them for blue (180° spin about the center).
- `subsystems/`: one class per mechanism. Each takes a `HardwareMap`, works on its own, has `update()` (reads/writes hardware) and `addTelemetry()`. Tunables are `public static` fields at the top of `@Configurable` classes (live-editable in Panels).
- `robot/Robot` owns all subsystems and holds the per-loop decision logic in `update()`; OpModes only map buttons to `Robot`. Test OpModes in `opmodes/test/` build single subsystems instead of `Robot`.
- `shot/`: `ShotSolver` (aim + RPM, layered SOTM with on/off switches) and `ShotTables` (distance lookup tables).
- Keep writes cached (`util/Cached*`), call `BulkReads.clear()` once per loop, and never read motor current in the match loop.
- Anything that needs measuring or tuning gets a `// TODO(step): what to do` comment right above it; the step is the `TUNING_GUIDE.md` section number. The `todoList` Gradle task (runs before every build) regenerates `TODO_LIST.md` from these. Don't edit `TODO_LIST.md` by hand.
- Code style: written for students to read. Plain names, short comments only where the why isn't obvious, no clever abstractions.

## Architecture

**OpModes** are discovered by annotation (`@TeleOp` / `@Autonomous`) via the SDK's annotation processor; no registration needed. Use `LinearOpMode` with a `while (opModeIsActive())` loop.

**Pedro Pathing 3.x (`teamcode/pedro/`)**
- `Constants.create(HardwareMap)` is the single factory that builds the `Follower` from three parts: a `Drivetrain`, a `Localizer`, and the path-following `Algorithm` (Foresight). It currently returns `null` and must be filled in with config objects (e.g. `MecanumConfig`, `PinpointConfig`, `ForesightConfig`) produced by the tuners.
- `Tuning.java` is the placeholder where tuning procedures get registered.
- `procedures/` contains the tuners. Each is a `com.pedropathing.tuning.autotune.Procedure` whose `run()` drives an interactive flow (`inputs(...)`/`awaitInputs`, `confirmation`, `runOpMode(new SomeTuningOpMode(...))`, `result`, `abort`) and ends by emitting a Java snippet via `code(Language.JAVA, ...)` to paste into `Constants`. Individual robot motions are package-private `TuningOpMode<T>` subclasses in the same file that return the measured value.
  - Hardware setup tuners (`MecanumTuner`, `PinpointTuner`, `OTOSTuner`, `OctoQuadTuner`, `TwoWheelTuner`, `ThreeWheelTuner`, `ThreeWheelIMUTuner`) take no constructor args.
  - `ForesightTuner` and `Tests` take `Function<HardwareMap, Localizer>` / `Function<HardwareMap, Drivetrain>` (and `Tests` a `Supplier<Algorithm>`) so they can run before a full `Follower` exists; `Tests` builds a `Follower` only when all three are non-null.
  - Tuners must keep commanding drive powers every loop iteration (not set once) so they work with swerve drivetrains, and use `Thread.sleep` pauses between phases to let the robot settle.

**Ivy (`teamcode/ivyexample/`)**: command-based pattern. Subsystems expose methods returning `Command`s (e.g. `instant(...).requiring(motor)`); the OpMode calls `Scheduler.reset()` before init, `schedule(cmd)` on input edges (`gamepad1.aWasPressed()`), and `Scheduler.execute()` once per loop.

## Upstream merges

History includes merges from the official FTC SDK and the Pedro Quickstart. Keep team changes inside `TeamCode/` and `build.dependencies.gradle` to keep merges clean.
