# Repo health report

## Fixes applied (2026-10-10)

Later the same day, the code was changed to fix most of what the review below found. A Gradle run after the changes was green: `:TeamCode:testDebugUnitTest` ran **181 tests in 27 classes, 0 failures** (up from 119 in 17), `assembleDebug` built `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk` at **81,799,948 bytes** (about 78 MB; it was 77,688,943), and the regenerated `TODO_LIST.md` has 162 items (61 / 12 / 23 / 36 / 30 by phase). Panels is now `com.bylazar:fullpanels` **1.0.18** (`build.dependencies.gradle:22`). The docs were brought in line with the code the same evening. None of it is committed yet (see "Still open").

Everything after this section is the original review, kept as the record. Only its scorecard was updated. Its file:line references point at the code before these fixes.

### Act on first

| # | Item | Status | Where |
|---|---|---|---|
| 1 | PinpointTuner behind upstream | Done | Merge commit 0481b65 (upstream 3494d7a and its merge 41158ec). `pedro/procedures/PinpointTuner.java` now matches `upstream/master`, and master is 0 commits behind it. |
| 2 | DriveRoute's detour allowance never runs | Done (uncommitted) | `agateflow/DriveRoute.java:471` reads the flag into `detour` before `:482` clears it, and `:491-495` gives a detour its own deadline, at most `MAX_DETOURS` times. Tested by `DriveRouteTest.aNewZoneAheadMakesItBrakeHoldAndDetourWithTheDetoursOwnTime` and `withoutDetourTimeTheSameDriveTimesOut`. |
| 3 | FireControl misses shots when the wheel starts above target | Done by a redesign (uncommitted) | `robot/FireControl.java` now runs continuous fire: the rail runs at the feed power for the whole burst and pauses only when not aimed or below `BURST_RPM_FLOOR`. The shot counter measures the dip from the highest reading of the last `DIP_WINDOW_MS` (`:215`, `:235-244`), the same for both hoods, and only reports shots, except with `stopAfterOneShot` (auto's `shoot()`). `FireControlTest` (14 tests), including `aSharpDipCountsWhereverTheWheelStarts`. `FEED_TIMEOUT_MS` and `RECOVER_MS` are gone. |
| 4 | INIT button presses fire on the first loop | Done (uncommitted) | `opmodes/teleop/MainTeleOp.java:77-78`, and `resetEdgeDetection()` right after init in every test OpMode (`ShooterTest.java:63`, `CoaxHoodTest.java:108`, `TurretTest.java:35`, `StorageTest.java:34`, `ServoPositionFinder.java:37`, `AgateFlowBenchmark.java:89`). |
| 5 | DriveRoute can't see a zone that lands on a stop | Done (uncommitted) | `agateflow/DriveRoute.java:586-588`: near a stop, a point counts as blocked when a new zone leaves less than `ROBOT_RADIUS`. The replan moves the stop. `DriveRouteTest.aPartnerParkedOnTheStopMovesTheStop`. |
| 6 | Shooter Test doesn't hold a fitted coax hood | Done (uncommitted) | `opmodes/test/ShooterTest.java:57` builds the hood when it is configured, and `:94-98` hold it at `CoaxHood.STATIC_LAUNCH_DEG` every loop. |
| 7 | Commit the uncommitted work | Open | See "Still open". |

### Other findings fixed the same day

- Match code: a clipped coax hood servo is now the `HOOD_AT_END` warning (`robot/Warning.java:22`, `robot/Robot.java:343`, `CoaxHood.servoAtEnd()` at `robot/subsystems/CoaxHood.java:187`), so it buzzes and shows in competition mode. `Shooter` has a NaN guard on its power (`robot/subsystems/Shooter.java:235`). `PIDF.smooth` (`util/PIDF.java:264`) is the one smoothing step, used by `Shooter`, `MotionPredictor` and `EtaCalibration`.
- Stale comments: `FireControl`'s class comment describes continuous fire. The shooter coast-down TODO is now `TODO(6)` and names driving toward the HIVE as well as trimming (`Shooter.java:138`). The `COAX_RPM` note lists the physics guesses its seeds depend on (`robot/shot/ShotTables.java:75-77`). `DriveRoute.java:184` names `RobotCommands.withFlowerIntake`.
- Dead code removed: `FireControl.burst()`, `Turret.stop()`, `Storage.slot2Occupied()`, `CoaxHood.trimDeg()`, `IntakePoses.gardenTravelDegrees()`; in AgateFlow `Leg.predictedSpeedAt`, `Leg.predictedPath()`, `Leg.speedLimitedSegments`, `ClearanceRule.reach(x, y, h)`, `ClearanceRule.circleNeed(int)`, `EtaCalibration.legsLearned()`, `Estimate.from()`, `FieldMap.clearZones`, `ParkGuard.disarm`, and the 5-argument `AgateFlow.plan` with its `follower` field (`AgateFlow(FieldMap)` is the only constructor). `FlowerIntake.isDown()` stays: `RobotCommands.parkGuard` now uses it.
- TODO hygiene: the missing TODOs were added: `TODO(16)` on `Field.PASS_TARGET_RED_X/Y`, `ShotTables.PASS_RPM` and `PASS_FEED_POWER`, `HoodSolver.SEED_RPM_WINDOW` (with "keep it equal to `Robot.COAX_RPM_TOLERANCE`"), `HardwareNames.HOOD_SERVO`, `CoaxHood.IDLE_LAUNCH_DEG` and `ON_ANGLE_DEG`, and a `SHOT_DIP_RPM` recheck; `TODO(12)` on `MainTeleOp`'s trim steps and `TRIGGER_THRESHOLD`; `TODO(11)` on `ShotSolver.MAX_ANGULAR_ACCEL` and `Shooter.MAX_TARGET_RATE`; `TODO(3)` on `Rail.REVERSE_POWER`; `TODO(13)` on `AgateFlow.PLAN_TIME_LIMIT_MS` and `DriveModel.SETTLE_SECONDS`; `TODO(14)` on `PICKUP_RUN`, `PICKUP_EXIT`, `PICKUP_SIDE_MARGIN` and `INTAKE_HALF_WIDTH`; `TODO(15)` on `MAX_DETOURS`, `MAX_OVERTIME_SECONDS`, `REPLAN_OFF_PATH`, `ParkGuard.REFRESH_MS` and `FALLBACK_SECONDS`. The `TODO_LIST.md` intro names five phases (`TeamCode/build.gradle:117-118`).
- Test OpModes: Coax Hood Test's physics mode feeds only with a solved angle and the hood at it (`CoaxHoodTest.java:185-193`), leaving raw mode starts from the angle the raw position maps to (`:150-153`), and the init seeds no longer run the seed scan twice (`:242-243`). Turret Test's manual mode is capped at `Turret.MAX_POWER` (`TurretTest.java:49`).
- AgateFlow: progress tracking starts over on every new plan (`DriveRoute.java:484`). A replan within `FLOWER_INTAKE_RAISE_RUN` of a flower-down stop keeps the intake-coming-up start (`DriveRoute.java:446`, `:602-605`, `Route.startingFlowerDown`), and so does a park drive (`ParkGuard.flowerIntakeDown`, `ParkGuard.java:162-164`). `ParkGuard` starts the park drive again whenever it isn't scheduled and hasn't arrived (`ParkGuard.java:122-123`), and its estimates wait while a drive's plan is pending (`:132`). `EtaCalibration` learns only from legs driven start to finish on their own plan that reached their stop (`DriveRoute.java:340`, `:478`), and `LEARN_RATE` became `LEARN_FILTER` (0.8), which runs the same way as every other filter. `ROBOT_FRONT` and `ROBOT_HALF_WIDTH` take their values from `Field` (`AgateFlow.java:87`, `:89`).
- Tests: new `FireControlTest` (14), `RetryTest` (5), `ShotSolverCoaxTest` (6), `FieldTest` (7), `LookupTableTest` (5), `DriveCurveTest` (4), `DebouncerTest` (4), `DriveRouteTest` (8), `ParkGuardTest` (3) and `EtaCalibrationTest` (6). `DriveRoute`, `Estimate` and `ParkGuard` read their time from `DriveRoute.NOW`, which `TestRobot` swaps for its virtual clock, and `TestRobot` puts back the statics it changes (`TestRobot.java:54-98`), so `FlowerIntakeStretchTest` no longer sleeps on the real clock. `HoodSolverTest.everyCoaxRpmRowHasRoomBothWays` now says in its failure message that it is expected to fail once the rows are tuned (`HoodSolverTest.java:366`).
- Repo: `.gitattributes` added (`* text=auto eol=lf`, with images, sounds, jars, PDFs, AARs and APKs marked binary).

### Still open

- **Commit the work.** 59 modified files and 17 untracked entries: 7 source files (`CoaxHoodTest`, `HoodType`, `HoodSolver`, `MotionPredictor`, `CoaxHood`, `PIDF`, `PanelsSettings`), 3 AgateFlow test classes, the test folders `field/`, `robot/` and `util/`, `.gitattributes` and `REPO_HEALTH.md`. 124 of the 181 tests are in files git doesn't have yet. Review the diff, run `onyx-noslop` on the text as CLAUDE.md asks, and commit on a branch.
- **`research/` and `.claude/` are git-ignored (decided 2026-10-11).** They are local-only AI assistant files and are never committed. The `.gitignore` edit is uncommitted; CLAUDE.md's pointers to them will not work in a clone.
- **Renormalize line endings in one commit.** `.gitattributes` is in place, but the files in git were not renormalized, and git still warns that CRLF will become LF in several working-copy files. Run `git add --renormalize .` in a commit of its own, so real changes stay readable in the diff.
- **Gradle and Java warnings.** The build still prints "Java compiler version 25 has deprecated support for compiling with source/target version 8" (the JDK running Gradle is 25; `build.common.gradle` sets Java 8 and we don't edit it). Pick a Gradle Java toolchain on an older JDK, or set `android.javaCompile.suppressSourceTargetDeprecationWarning=true` in `gradle.properties`. Gradle also still warns about features that Gradle 10 drops; find them with `--warning-mode all` before moving past wrapper 9.1.0.
- **`AgateFlow.java` is 2065 lines.** The pickup planning could move into `Pickup`.
- **A `ParkGuard` retry can plan with the intake up.** `RobotCommands.parkGuard` passes `robot.flowerIntake::isDown`, which is the commanded state, and the park drive commands the intake up as it starts. So a retry within `FLOWER_RAISE_MS` (400 ms) of the first start plans without the back-out, though the intake may still be coming up.
- **The no-shot note is telemetry only.** When the rail runs `NO_SHOT_WARN_MS` with no shot counted, `FireControl.status()` adds a note to the "Fire" line (`FireControl.java:273-275`). That line only shows with `MainTeleOp.FULL_TELEMETRY` on, it isn't a `Warning`, and nothing stops the rail. In competition mode the drivers won't see a jam this way; unjam (LT) is up to them.
- Also still open from the review below: `Robot`'s per-loop decisions have no desktop test (they need a fake `HardwareMap`); `com.pedropathing:tuning` and `ivy:pedro` were not checked against their repo; nothing has run on the robot. The review's other Low and Nit items that aren't named in this section were not re-checked.

---

**2026-10-10.** Built from four reviews run today: build, tests and dependencies; the docs; the match code (`robot/`, `driver/`, `field/`, `util/`, `opmodes/`); and AgateFlow with `pedro/`. Every file:line under "Act on first" was opened and checked against the source. Paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` unless they start with `TeamCode/` or name a file at the repo root. Severity tags: Medium (can cost a match or a tuning step), Low (small bug or drift), Nit (style).

**Bottom line.** The build is green: `assembleDebug` passes and so do the desktop tests, 119 of 119. Dependencies are current except Panels (one patch behind) and the Gradle wrapper; two Pedro artifacts could not be checked. Four things need action before the robot is used: merge upstream's PinpointTuner fix before step 2, fix FireControl's dip test for a wheel that starts above its target, clear button presses made during INIT in MainTeleOp, and make Shooter Test hold a fitted coax hood. AgateFlow has two runtime bugs in `DriveRoute`, but no match OpMode uses it yet. The build also depends on work that isn't committed: 7 untracked source files (the coax hood, `HoodSolver`, `MotionPredictor`, `PIDF`, `PanelsSettings`) plus two untracked test folders, on top of 24 modified files. Until that is committed it exists only on this computer.

## Scorecard

Updated after the fixes above (2026-10-10, evening). The status the review first gave is in brackets.

| Area | Status | Why |
|---|---|---|
| Build | Good [Good] | `assembleDebug` passes; the APK is 81,799,948 bytes. The `todoList` task prints no warnings. The other warnings are the JDK's Java 8 notice and Gradle's own; none points at team code. |
| Tests | Good [Good] | 181/181 pass in 27 classes (was 119 in 17). `FireControl`, `DriveRoute`, `ParkGuard`, `EtaCalibration`, `Retry`, the coax `ShotSolver` paths and the `field/` and `util/` helpers are now tested. `Robot`'s decisions still aren't. |
| Dependencies | Good [Good] | FTC SDK 12.0.0, Pedro 3.0.1 and Panels 1.0.18 are current. Two Pedro artifacts unchecked. The Gradle wrapper (9.1.0) is behind. |
| Match code (`robot/`) | Good [Needs attention] | Continuous fire replaced the dip-gated feed, so a missed dip can't hold up a burst. INIT presses are cleared at Start. A clipped hood servo is a warning. Nothing has run on the robot yet. |
| AgateFlow | Good [Needs attention] | The detour allowance and the zone-on-a-stop check work and are tested. One known gap: a `ParkGuard` retry within `FLOWER_RAISE_MS` plans with the intake up. Not in a match OpMode yet. |
| Pedro setup | Good [Needs attention] | `PinpointTuner` matches upstream. `Constants.create` still returns null (expected before tuning). |
| Docs | Current [Needs attention] | CLAUDE, ROBOT, TELEOP_PLAN, TUNING_GUIDE and SOFTWARE_INNOVATIONS were brought in line with the code on 2026-10-10. `CODE_REVIEW.md` (local only) is marked as a historical record. |
| Repo hygiene | Stale [Stale] | 59 modified files and 17 untracked entries are uncommitted, and the build needs them. Line endings aren't renormalized yet. Master is level with upstream. |

## Act on first

1. **PinpointTuner is behind upstream.** `pedro/procedures/PinpointTuner.java:146-150` and `:190-194`. Both direction checks read the pose after Stop is pressed. The Pinpoint resets on Stop, so "forward reversed" and "strafe reversed" always come back false, and step 2 can paste wrong pod directions into `Constants`. Fix: `git merge upstream/master` (2 commits, 3494d7a and its merge 41158ec, +14/-2 in this one file, no conflict with team edits).

2. **DriveRoute's detour allowance never runs.** `agateflow/DriveRoute.java:418` sets `replanForBlock = false`, and `:425` then tests `replanForBlock && detours < MAX_DETOURS`, which is always false. `detours` never counts up, `MAX_DETOURS` does nothing, and a detour around a new zone is still capped at the first deadline plus `REPLAN_EXTRA_SECONDS`. The class comment (`:34-41`), CLAUDE.md:52 and SOFTWARE_INNOVATIONS.md:193 describe time the code doesn't give. Fix: copy the flag into a local before line 418 and test the local at line 425.

3. **FireControl misses shots when the wheel starts above target.** `robot/FireControl.java:117`. The dip is measured from `min(peak, latched target)`, so a wheel that starts 300 RPM above target (allowed in COAX by `Robot.COAX_RPM_TOLERANCE`, `robot/Robot.java:54` and `:288-291`) has to drop `SHOT_DIP_RPM` + 300 = 450 RPM to count. That is common: the target falls as the robot drives toward the HIVE, and the Shooter can't brake (FLOAT, power clipped 0 to 1). A missed dip keeps the rail feeding for `FEED_TIMEOUT_MS` (400 ms), which can send a second ball, and counts as a misfeed, which throws off `RobotCommands.shoot()`. STATIC has the same problem, bounded by its 90 RPM window. Fix: measure the dip from where the wheel really was when the feed started, keeping a guard so a wheel coasting back down to target isn't counted (the reason the cap exists), and write the `FireControl` test first.

4. **INIT button presses fire on the first loop.** `opmodes/teleop/MainTeleOp.java:117-135`. `...WasPressed()` is true for any press since the last call, and during INIT only `AllianceSelector` asks, for D-pad up. So a D-pad left/right, Circle or Square on gamepad1 during INIT relocalizes to a corner on the first loop after Start (once an alliance is picked) and marks the pose trusted, which defeats `canFire()`. LB toggles the flower intake, and gamepad2 presses add trims. The test OpModes do the same (`opmodes/test/ShooterTest.java:61`: Cross in INIT turns the wheel on at Start). Fix: call `gamepad1.resetEdgeDetection()` and `gamepad2.resetEdgeDetection()` right after the init loop.

5. **DriveRoute can't see a zone that lands on a stop.** `agateflow/DriveRoute.java:515-516` skips every predicted point within `ROBOT_RADIUS + SAFETY_MARGIN` (12 in.) of a leg's end or a later leg's start, however much room a new zone took there. A partner robot parked on our stop never triggers a replan, and our robot drives into it. Fix: still count near-stop points whose new clearance is under `ROBOT_RADIUS`.

6. **Shooter Test doesn't hold a fitted coax hood.** `opmodes/test/ShooterTest.java:48-84` never builds `CoaxHood`. With the hood on the robot, step 7 measures `SHOT_RPM` in Shooter Test (the TODO at `robot/shot/ShotTables.java:26`) while the hood servo gets no command, so the rows are measured at whatever angle the hood sits at, not `CoaxHood.STATIC_LAUNCH_DEG`. TUNING_GUIDE.md:221 says steps 6 and 7 still hold with the hood fitted; that is only true inside `Robot`. Fix: build `CoaxHood` when `CoaxHood.isConfigured(hardwareMap)` and hold it at `STATIC_LAUNCH_DEG` every loop.

7. **Commit the uncommitted work.** 7 untracked source files and 6 untracked test classes (62 of the 119 tests), plus 24 modified files (list under Repo hygiene). The build and those tests need them. Fix: review the diff, run `onyx-noslop` on the text as CLAUDE.md asks, and commit it on a branch.

## Build and tests

Gradle ran from Git Bash with `--offline`. Every build passed.

- `:TeamCode:testDebugUnitTest`: BUILD SUCCESSFUL in 30 s wall time, about 7.7 s of test time. 17 classes, 119 tests, 0 failures, 0 errors, 0 skipped.
- About the count: 119 was confirmed by a Gradle run on 2026-10-10 (summing the `tests` attribute of the 17 result XML files in `TeamCode/build/test-results/testDebugUnitTest/`), and it matches a hand count of `@Test` annotations. An earlier draft of this report said 120; that was an addition error.
- Every test is deterministic: fixed random seeds, and every planning test sets `PLAN_TIME_LIMIT_MS` to 1e9.
- The run wrote 5 plan pictures to `TeamCode/build/agateflow-svg/`.

| Package | Class | Tests | In git |
|---|---|---|---|
| agateflow | AgateFlowBenchmarkTest | 1 | yes |
| agateflow | AgateFlowTest | 8 | yes |
| agateflow | DriveModelTest | 4 | yes |
| agateflow | FlowerIntakeStretchTest | 2 | yes |
| agateflow | MotionModelTest | 3 | yes |
| agateflow | OutlineFitTest | 6 | yes |
| agateflow | PickupTest | 10 | yes |
| agateflow | PlanDrawingTest | 5 | yes |
| agateflow | RouteTest | 8 | yes |
| agateflow | SafetyFuzzTest | 1 | yes |
| agateflow | ZoneTest | 9 | yes |
| robot.hardware | CachedHardwareTest | 4 | no |
| robot.shot | HoodSolverTest | 24 | no |
| robot.shot | MotionPredictorTest | 10 | no |
| robot.shot | ShotSolverTest | 4 | no |
| util | AnglesTest | 3 | no |
| util | PIDFTest | 17 | no |

Slowest: SafetyFuzzTest 4.711 s, PickupTest 1.264 s, AgateFlowBenchmarkTest 0.486 s, PlanDrawingTest 0.395 s, AgateFlowTest 0.288 s.

**APK.** `clean assembleDebug --offline --rerun-tasks` passed in 1 m 6.8 s. `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk` is 77,688,943 bytes (about 74 MB).

**Warnings.** None points at a file:line in team code. No unchecked or deprecation warnings in TeamCode source, and the `todoList` task printed none (no malformed TODOs, no unknown steps).

- "Java compiler version 25 has deprecated support for compiling with source/target version 8", plus "source value 8 is obsolete" and "target value 8 is obsolete". The JDK running Gradle is 25, and the project compiles at Java 8 (`build.common.gradle:103-104`, which we don't edit). Harmless today, but a later JDK may drop Java 8. Two ways out: a Gradle Java toolchain on an older JDK, or hide it with `android.javaCompile.suppressSourceTargetDeprecationWarning=true` in `gradle.properties`.
- `FtcRobotControllerActivity.java` uses a deprecated API. That is the upstream SDK module, not ours.
- A native-access "restricted method" warning from Gradle 9.1.0's own native-platform library. Not ours.
- "Deprecated Gradle features were used in this build, making it incompatible with Gradle 10." The wrapper is 9.1.0, so nothing breaks now. Find the feature (`--warning-mode all`) before any move to Gradle 10.

## Dependencies

From `debugRuntimeClasspath`. Pinned versions are in `build.dependencies.gradle:9-25` and the root `build.gradle:16-17`.

| Artifact | Pinned | Latest | Status | Link |
|---|---|---|---|---|
| `org.firstinspires.ftc:*` (8 artifacts) | 12.0.0 | 12.0 (12 Sep 2026) | Current. Needs Android Studio Narwhal 3 Feature Drop or later. | https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases |
| `com.pedropathing:revhub` (brings core) | 3.0.1 | 3.0.1 (18 Sep) | Current | https://github.com/Pedro-Pathing/PedroPathing/releases |
| `com.pedropathing:tuning` | 1.0.1 | unknown | Could not be checked | https://repo.dairy.foundation/#/releases/com/pedropathing |
| `com.pedropathing.ivy:pedro` (with ivy:core 1.1.1) | 1.1.1 | unknown | Could not be checked | https://repo.dairy.foundation/#/releases/com/pedropathing |
| `com.bylazar:fullpanels` (with panels 1.0.7) | 1.0.17 | 1.0.18 | One patch behind. No changelog found. | https://mymaven.bylazar.com/releases/com/bylazar/fullpanels/maven-metadata.xml |
| `androidx.appcompat:appcompat` | 1.2.0 | not checked | Held at 1.2.0 strictly by Panels | |
| `junit:junit` (tests only) | 4.13.2 | not checked | Not checked | |
| Gradle wrapper | 9.1.0 | 9.8.1 | Behind. Already warns about Gradle 10. | https://gradle.org/releases/ |
| Android Gradle Plugin | 8.13.2 | 8.13.2 on 8.x; 9.4.1 | Newest 8.x. 9.x is a major migration: check what the FTC SDK supports first. | https://developer.android.com/build/releases/gradle-plugin |

- The two Pedro artifacts couldn't be checked because repo.dairy.foundation is a single-page web app and its Maven metadata returned 404. Check them in a browser.
- `tuning` 1.0.1 asks for revhub 3.0.0 and gets 3.0.1. Pedro 3.0.1 fixed the path completion calculation and added a 500 ms Pinpoint IMU calibration sleep, PoseFactory angle units and `driveOrHold()`. Nothing changed in `Follower` or `Modifier` beyond that.
- Version conflicts Gradle resolved, all harmless: kotlin-stdlib (2.3.21, 2.0.21, 1.3.41) raised to 2.4.10; annotations 13.0 to 23.0.0; androidx annotation 1.1.0 to 1.2.0, core 1.1.0 to 1.3.0, lifecycle 2.0.0 to 2.1.0, collection 1.0.0 to 1.1.0.

## Code quality by area

The reviewers found the code in good shape for a student codebase: thorough, mostly accurate comments, a compliant TODO system, and NaN guards, watchdogs and write caching applied with care. Checked clean: no hardware writes during INIT, no motor current reads, every motor and servo write cached (except `ServoPositionFinder`), direction flags applied at write, units consistent, Ivy end handlers safe to call twice, live suppliers asked only on the main thread, the Modifier leak covered, and no Java 9+ APIs in our code.

### robot/ (match code)

Correctness
- [Medium] `robot/FireControl.java:117`: the dip test from above target. Act on first #3.
- [Low] `robot/subsystems/CoaxHood.java:204-209` with `:167-170`: when the mapping asks for a servo position past 0 to 1, the servo is clipped but the estimate still "arrives", so `atTarget()` turns true and shots fire at the wrong angle. The only notice is a telemetry line (`:238-240`) that shows only with `FULL_TELEMETRY` on. Fix: make it a `Warning`, or keep `atTarget()` false.
- [Low] `robot/subsystems/Shooter.java:231`: no NaN guard on the motor power (`Turret.java:345` and `CoaxHood` have one). A NaN Panels gain passes `Range.clip` and is sent every loop. One line, like the Turret's.
- [Low] Other risks to check during step 16, not one-line fixes: `HoodSolver.java:89-94` assumes the CELL opening faces the shooter, but the opening is at the cell's outer end along y, so most shots cross it at an angle; in COAX, boost plus the 300 RPM window means many balls leave while the wheel speeds up, and `averageRpm()` (3 samples, about a loop late) reads low, which `RPM_MARGIN` 50 may not cover (`Robot.java:220`, `:233`, `Shooter.java:227`); `CachedServo.java:15` caches on exact equality, so the hood servo is written nearly every loop in COAX; switching burst mode mid-feed keeps the old latched target (`FireControl.java:97`, `:109`), unreachable while `PASS_ENABLED` is false.

Stale comments
- [Low] `robot/FireControl.java:13` and `:75` say ready means "turret on target AND flywheel at speed". Wrong for COAX (it is within `COAX_RPM_TOLERANCE` with the hood at a scoring angle).
- [Low] `robot/subsystems/Shooter.java:136` (TODO(12)) blames only trim-down for the coast that blocks shots. Driving toward the HIVE lowers the target the same way.
- [Low] `robot/shot/ShotTables.java:65-72`: the `COAX_RPM` seed note lists only the seedRpm inputs, window and margin. The seeds also depend on `LAUNCH_HEIGHT`, `COAX_EXIT_SPEED` and `TARGET_BAND`.
- [Nit] Six small comment drifts: `Robot.java:56-59` (leaves out the hood target), `Storage.java:78` ("at least 2 balls", auto allows 1), `ShotSolver.java:33`, `CoaxHood.java:38`, `ShotSolution.java:13-14`, `ShotSolverTest.java:86`.

Dead code and duplication
- [Low] Unused, safe to delete: `FireControl.burst()` (`:138`), `Turret.stop()` (`:543`), `Storage.slot2Occupied()` (`:66`), `FlowerIntake.isDown()` (`:41`), `CoaxHood.trimDeg()` (`:147`), `IntakePoses.gardenTravelDegrees()` (`:106`). `MotionPredictor.compensated()` and `alpha()` are public but only the tests use them.
- [Low] `Shooter.java:248-261` repeats `PIDF`'s target-rate logic (jump rejection, cap, smoothing), and the filter clamp `min(max(f, 0), 0.99)` is copied in `PIDF`, `Shooter` and `MotionPredictor`. NaN checks come in three styles.
- [Nit] "KV" means target-rate feedforward in `Turret` but power per RPM in `Shooter`; `FireControl.update`'s `coaxHood` parameter shares a name with `Robot.coaxHood`; `RobotCommands.java:173` and `:90` hold on interrupt differently; `HoodSolver` (518 lines) and `Turret` (560 lines, three safety state machines) are at the limit of "written for students".

TODO hygiene
- All 107 TODOs in this area are well formed.
- [Low] Missing TODO(16): `Field.PASS_TARGET_RED_X/Y` (`field/Field.java:57-58`, marked "Placeholder") and `ShotTables.PASS_RPM` / `PASS_FEED_POWER` (`:54-62`); their distance range gates coax passes (G405). `HoodSolver.SEED_RPM_WINDOW` (`:107`) has no TODO and no note that it must match `Robot.COAX_RPM_TOLERANCE` (both 300).
- [Nit] No TODO on `MainTeleOp`'s trim steps and `TRIGGER_THRESHOLD`, `CoaxHood.IDLE_LAUNCH_DEG` / `ON_ANGLE_DEG`, `ShotSolver.MAX_ANGULAR_ACCEL`, `Shooter.MAX_TARGET_RATE`, `Rail.REVERSE_POWER`. Questionable steps: `Shooter.java:136` TODO(12) matters at step 6 or 7, `Turret.java:123` TODO(11) also smooths D (step 5), and `SHOT_DIP_RPM` has no TODO(16) recheck for the 300 RPM coax window.

Test coverage
- [Medium] `FireControl` has no test: the coax dip rule, release mid-feed, `READY_HOLD_MS`, the misfeed timeout, the `enoughBalls` gate. It is pure logic apart from `ElapsedTime`.
- [Medium] `Robot`'s decisions are untested: `shotReady`, `aimHood`, the hood switch only between bursts with trim reset, keeping the last aim when the pose goes NaN mid-burst, `hoodOutOfReach`. Needs a fake `HardwareMap`; `CachedHardwareTest` already builds Proxy fakes.
- [Medium] `ShotSolver`'s coax paths are untested: `solveCoaxPass`, coax `LEAD` with physics flight time and the `SHOT_FLIGHT_TIME` fallback. One coax assertion exists (`ShotSolverTest.java:101`).
- [Low] `HoodSolverTest.java:356` (`everyCoaxRpmRowHasRoomBothWays`) pins the placeholder `COAX_RPM` rows to the physics model. It will fail once the rows are tuned from real shots or `LAUNCH_HEIGHT` / `COAX_EXIT_SPEED` are measured.
- [Low] Also untested: `CoaxHood` mapping and timing, `Shooter` averaging, watchdog and boost, `Turret`'s stuck, wrong-way and full-power checks, `Retry`, and the static half of `ShotTables.orderProblem`.

### agateflow/

Correctness
- [Medium] `agateflow/DriveRoute.java:418` / `:425`: the detour allowance never runs. Act on first #2.
- [Medium] `agateflow/DriveRoute.java:515-516`: a zone on a stop is never seen. Act on first #5.
- [Low] `DriveRoute.takePlan` (`:409-435`) keeps `progressLeg` and `closestToStop`, and `trackProgress` (`:309`) resets them only when the leg changes. After a replan, a detour that first moves away counts as "not getting there" and loses the `MAX_OVERTIME_SECONDS` grace. Fix: set `progressLeg = -1` in `takePlan`.
- [Low] Flower raise-run protection is lost on a replan: `AgateFlow.java:341` sets `startFlowerDown` only from the previous target in the same `Route`. A replan within 12 in. of a flower-down stop, or a `ParkGuard` interrupt at a FLOWER, plans with the intake up and no back-out, so the intake can sweep through the FLOWER.
- [Low] `ParkGuard.java:101-102` retries only after `NO_PLAN` or `TIMED_OUT`. A park drive that was blocked when scheduled (`WAITING`) or interrupted (`STOPPED`) leaves it in `PARKING` for good. Fix: retry on those too.
- [Low] `AgateFlow.plan` with 5 arguments (`:226-228`) is unused and calls `DriveModel.capture` on the caller's thread, writing main-thread-only statics. Fix: delete it and the `follower` field. The 6-argument `plan` javadoc (`:230-234`) should warn that `toLive` suppliers run on the calling thread.
- [Low] Other: `EtaCalibration` learns from wrong data (`legReplanned` at `:414` never cleared in `startLeg`; settle-timeout legs at `:467`, `:470` recorded as normal; a `driveTo(estimate)` joined partway records against the full-leg estimate); the single planning thread (`AgateFlowCommands.java:85`) gets a park estimate every 250 ms from `ParkGuard` (`:107-111`), so replans queue behind it.
- [Nit] `Leg.secondsLeft` (`Leg.java:116-117`) relies on `follower.isBusy()`, which in Pedro 3.0.1 doesn't fall after a bare `hold()`. Telemetry only.

Stale comments
- [Low] `DriveRoute.java:34-41`, CLAUDE.md:52 and SOFTWARE_INNOVATIONS.md:193 promise per-detour time. Fixed by Act on first #2.
- [Low] `DriveRoute.java:156-157` says `RobotCommands.driveTo` lowers and raises the intake. It is `RobotCommands.withFlowerIntake`.
- [Nit] Wrong names in `EtaCalibration.java:7-8`, `Route.java:187`, `AgateFlow.java:945` and `:956`; waypoints numbered from 0 in a message at `AgateFlow.java:809`; garbled sentences at `ClearanceRule.java:17-19` and `DriveRoute.java:29-30`; the `PlanDrawing` example (`:20-24`) lacks a "Panels is practice only" note.

Dead code and duplication
- [Low] Unused: `Leg.predictedSpeedAt` (`:133`), `Leg.predictedPath()` (`:139`), `Leg.speedLimitedSegments` (set, never read), `ClearanceRule.reach(x, y, h)` (`:176`), `ClearanceRule.circleNeed(int)` (`:276`), `EtaCalibration.legsLearned()`, `Estimate.from()`, `FieldMap.clearZones`, `ParkGuard.disarm`, and the 5-argument `plan` with `AgateFlow.follower`.
- [Low] `ROBOT_FRONT` and `ROBOT_HALF_WIDTH` (`AgateFlow.java:84-86`) copy `Field.CENTER_TO_INTAKE_WALL` and `CENTER_TO_SIDE_WALL` (7.99, 5.87), and the TODO at `:73` asks for a hand copy, so they can drift. Fix: read the `Field` values.
- [Low] `EtaCalibration.LEARN_RATE` (`:22`) runs the opposite way from CLAUDE.md's smoothing rule (0 = none, closer to 1 = smoother).
- [Low] `AgateFlow.java` is 2063 lines. The pickup planning (`:362-637`, about 270 lines) could move into `Pickup`.
- [Nit] `Math.hypot` in hot paths though `Geometry` exists; fully qualified names despite imports; polyline distance written twice (`DriveRoute`, `MotionModel`); `CornerCurve` public but package-only; `VisibilityGraph.wallGrowth` builds a throwaway `Zone` per cache lookup.

TODO hygiene
- All 25 TODOs here are well formed (steps 2, 13, 14, 15) and match TODO_LIST.md.
- [Low] Tunables with no TODO: `AgateFlow.PLAN_TIME_LIMIT_MS`; `AgateFlowCommands.MAX_OVERTIME_SECONDS`, `MAX_DETOURS`, `REPLAN_OFF_PATH`; `DriveModel.SETTLE_SECONDS`; `PICKUP_RUN`, `PICKUP_EXIT`, `PICKUP_SIDE_MARGIN`, `INTAKE_HALF_WIDTH` (step 14); `ParkGuard.REFRESH_MS`, `FALLBACK_SECONDS`.

Test coverage
- 57 tests in 11 classes, deterministic, with an independent judge (`TrueRule`).
- [Medium] The runtime layer is untested: `DriveRoute` replan, watch-ahead, braking, pause, deadline, detour, overtime and stale-drop (the one `DriveRoute` test runs with `REPLAN = false`, which is how bug #2 got through); `ParkGuard`; `Estimate`, `driveTo(estimate)`, `canUse`, `planFrom`; `EtaCalibration`; `DriverAssist`; `FieldMap` threading. The command, park and fuzz harness that CODE_REVIEW.md cites lived in scratch folders and is not in the repo.
- [Low] `FlowerIntakeStretchTest` could flake: it uses the real planning thread with `Thread.sleep(1)`, and `DriveRoute`'s deadline and settle timers use `System.nanoTime` while `TestRobot` runs a virtual clock.
- [Low] Shared static state leaks between tests: `TestRobot` zeroes `DriveModel.*_KP_*` and never restores them, `EtaCalibration.TIME_SCALE` changes after `FlowerIntakeStretchTest`, and `DriveModel.lastGains` / `lastSpeedLimit` persist.

### pedro/

- [Medium] `pedro/procedures/PinpointTuner.java:146-150`, `:190-194`: behind upstream. Act on first #1.
- `Constants.create` still returns null (`Constants.java:40-43`). Expected until step 2; `Robot` throws a clear error at init.
- Against `upstream/master` only 4 files differ: `Constants` (comments and TODOs only; the stub's argument order is right), `TeamTuning` and `DriveModelTuner` (ours), and `PinpointTuner` (missing the fix). `Tuning.java` and the other 8 tuners are byte-identical to upstream. `TeamTuning`'s commented registrations match the constructor orders.
- [Nit] `DriveModelTuner.java:130` adds a package-private top-level class `FullPower` to the upstream package; a future upstream class with that name would clash. Its fit math is right but untested (`Run.fit`, `batteryOhms`).

### util/, field/, driver/

- [Low] `util/PanelsSettings.java:22`: the `EVENT_BUILD = true` path has never run. Every OpMode builds `JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), ...)`, so confirm on the robot that it does nothing harmful with Panels off, well before the first event.
- [Low] `field/Field.java:57-58`: the pass target has no TODO(16) (see robot/ TODO hygiene). `IntakePoses.gardenTravelDegrees()` is unused.
- `util/PIDF` is well covered (17 tests) and `Angles` has 3. Untested [Low]: `LookupTable`, `DriveCurve`, `Debouncer`, `Field.cellForSide` / `cornerPose` / `cellTarget`, `Alliance.fromRed`, `RobotState`.
- [Nit] The CAD-derived `Field.RED_HIVE_X` and `*_CELL_Y` have no TODO. `AllianceSelector` reads only D-pad up, which is why other INIT presses leak (Act on first #4). `Feedback` runs on driver 1 only (`MainTeleOp.java:55`); the docs don't say so.

### Test OpModes (opmodes/test/)

- [Medium] `ShooterTest.java:48-84` never holds a fitted coax hood. Act on first #6.
- [Low] `CoaxHoodTest.java:136`: in physics mode RB feeds as soon as the door is open, without waiting for `hood.atTarget()` or a solved angle. A NaN `solvedDeg` fires at the old target, which corrupts the `COAX_EXIT_SPEED` calibration. Fix: fire only when `hood.atTarget()` and `solvedDeg` is a number.
- [Low] `TurretTest.java:44` with `Turret.java:329`: manual mode sends the raw stick (up to 1.0) without clipping to `MAX_POWER` 0.8.
- [Low] INIT presses carry into the first loop here too (`ShooterTest.java:61` and the others). Same fix as Act on first #4.
- [Nit] Leaving raw mode in Coax Hood Test (Circle) jumps the hood back to the old angle (`CoaxHoodTest.java:138-142`); `seedLaunchDeg` re-runs `seedRpm`, doubling the init seed scan (`:216-217`); the `ServoPositionFinder` header doesn't say the hood servo is found in Coax Hood Test.

## Documentation staleness

Method: every backticked name in CLAUDE, ROBOT, TELEOP_PLAN, TUNING_GUIDE and SOFTWARE_INNOVATIONS exists in the code, and TODO_LIST.md was regenerated with a copy of the `build.gradle` rules.

| Doc | Verdict | Stale lines |
|---|---|---|
| CLAUDE.md | Current | :106-108 points to `.claude/skills/` and `research/onyxskill-research/`, both git-ignored (local-only), so a clone won't have them. :25 lists libraries without versions and leaves out the JUnit 4.13.2 test dependency. :26 leaves out `research/README.md`. :52 promises per-detour time (Act on first #2). |
| ROBOT.md | Minor drift | :9 says TU03 but :19-20 cite TU04. :39 says the names match `HardwareNames.java`, but the table (:24-37) has no configuration-name column, so `frontLeft` ... `hood`, `beamSlot1`, `beamSlot2`, `beamFull` appear nowhere. :105 asks to check the CR pulse range 1050 to 1950 us, but no code sets a PWM range and there is no TODO(5). |
| TELEOP_PLAN.md | Minor drift | :38 ties the hood trim to `Robot.HOOD = COAX`; the code uses `robot.usingCoaxHood()` (`MainTeleOp.java:130`), which needs the servo and changes only between bursts, and with `HOOD_MISSING` the D-pad still trims RPM. :42-43 don't say rumble and light are gamepad1 only. :51 says `OUT_OF_RANGE` needs the wheel "at speed"; the code means within 300 RPM. :62 leaves the coax hood out of the first-loop list. No pass target in the field section. |
| TUNING_GUIDE.md | Minor drift | :23 "8 motors and 4 servos" (5 with the hood). :186 "No OpMode uses it yet": AgateFlow Benchmark does, so "no match OpMode". :221 says steps 6 and 7 hold with the hood fitted; not in Shooter Test (Act on first #6). :229, :257, :259, :260 have no TODO(16) in the code. Every quoted default checks out. |
| SOFTWARE_INNOVATIONS.md | Rewritten 2026-10-10 | Rewritten today after this audit, so these lines no longer apply. Before the rewrite: :5 "Status (2026-10-08)" and "57 JUnit tests". :35 "four phases" (now five). :148 warning list lacks `HOOD_MISSING`. :193 per-detour time. :214 "166 lines, 23 KB" isn't backed by code or a test. :218 "57 JUnit tests ... All pass as of 2026-10-08". Nothing on the coax hood, `MotionPredictor`, `PIDF` or `PanelsSettings`. |
| CODE_REVIEW.md | Stale, local only | Not in git (`.git/info/exclude:7`). Still names `planner/`; the newest round is AgateFlow phase 2, nothing on the coax work. Old TODO steps (F7 and F8 TODO(8) are now 14 and 13, V3 TODO(12) is now 15). "HEAD is still f0af773" (now da21fe1), 57/57 tests, "steps 1-12, 87/91 items" (now 1-16, 133). V3 is marked "Code fixed" though the detour allowance never runs. Most fix-status file:line refs point at old code. |
| TODO_LIST.md | Current | Identical to a regeneration: 133 items (57 / 11 / 19 / 25 / 21 by phase). Only the intro (:5, from `TeamCode/build.gradle:117-118`) still says four phases. |
| README.md | Current | Upstream's v12.0 readme, unchanged. |
| research/README.md | Current | The whole `research/` folder is git-ignored (local-only). |

`SOFTWARE_INNOVATIONS.md` was rewritten on 2026-10-10, after the audit above. The rewrite covers the sections the audit listed (coax hood aimed by physics, hood position without a sensor, the physics seed table, coax shot confirmation, motion prediction, flywheel anticipation, our turret controller, turret encoder calibration, Panels off at events, nothing moves in INIT, safe auto drives, the optional hood, desktop tests beyond AgateFlow).

Missing from the docs:
- Configuration names, including `hood`, are only in `HardwareNames.java:11-32`.
- The pass target (24, 106), `Field.java:57-58`, is not in TELEOP_PLAN's field tables.
- Rumble and the ball-count light are on driver 1 only (`MainTeleOp.java:55`).
- The coax hood's power-on wait: `atTarget()` stays false for (MAX - MIN) / SPEED, 35 / 200 = about 0.18 s, after start (`CoaxHood.java:166-169`). `ON_ANGLE_DEG` 0.5 (`:90`) isn't mentioned.
- The hood telemetry lines "Hood servo at its end" and "HOOD SERVO NOT SENT" (`CoaxHood.java:236-240`). Step 16 doesn't say to watch for them.
- Knobs no doc names: `ShotSolver.MAX_ANGULAR_ACCEL` (`:72`), `LEAD_ITERATIONS` (`:73`); `HoodSolver.TARGET_HEIGHT`, `TARGET_SAMPLES`, `PASS_LANDING_HEIGHT`, `PASS_LANDING_TOLERANCE` (`:81-111`); `Shooter.MAX_TARGET_RATE` (`:59`); `Turret.CLOSE_ZONE_BAND_DEG` (`:101`); `CoaxHoodTest.SEED_MAX_RPM` (`:58`).
- Seed rows don't line up: Coax Hood Test prints seeds every 10 in. (`CoaxHoodTest.java:215`) but `COAX_RPM` rows are every 15 in. (`ShotTables.java:73-81`), while TUNING_GUIDE.md:247 says the seeds can be copied straight in.
- Shooter Test doesn't command the hood (Act on first #6).
- The uncommitted work adds Panels pages Robot, CoaxHood, HoodSolver and CoaxHoodTest. Only "Robot" is named in the docs.

## Repo hygiene

- **Modified, not committed (24):** `.gitignore`, `CLAUDE.md`, `ROBOT.md`, `SOFTWARE_INNOVATIONS.md`, `TELEOP_PLAN.md`, `TODO_LIST.md`, `TUNING_GUIDE.md`, `TeamCode/build.gradle`, and 16 source files: `field/Field.java`, `opmodes/auto/AutoTemplate.java`, `opmodes/teleop/MainTeleOp.java`, `robot/FireControl.java`, `robot/Robot.java`, `robot/Warning.java`, `robot/commands/RobotCommands.java`, `robot/hardware/` `CachedCRServo`, `CachedMotor`, `CachedServo`, `HardwareNames`, `robot/shot/` `ShotSolution`, `ShotSolver`, `ShotTables`, `robot/subsystems/` `Shooter`, `Turret`.
- **Untracked (9 entries):** 7 source files: `opmodes/test/CoaxHoodTest.java`, `robot/HoodType.java`, `robot/shot/HoodSolver.java`, `robot/shot/MotionPredictor.java`, `robot/subsystems/CoaxHood.java`, `util/PIDF.java`, `util/PanelsSettings.java`. Two test folders: `TeamCode/src/test/java/.../robot/` (CachedHardwareTest, HoodSolverTest, MotionPredictorTest, ShotSolverTest) and `.../util/` (AnglesTest, PIDFTest). 
- **`.claude/` and `research/`:** git-ignored as local-only AI assistant files (decided 2026-10-11), by the working-tree `.gitignore` (uncommitted; the committed one has no entry). CLAUDE.md:109 still says only `.claude/onyx/` and `.claude/onyx-loop/` are ignored.
- **Line endings:** `core.autocrlf` is true and there is no `.gitattributes`, so git prints LF-to-CRLF warnings on diff for most modified files and line endings depend on each computer's setting.
- **Upstream:** master is 2 commits behind `upstream/master` and 10 ahead (merge base 2df9646). The 2 missing commits touch only `PinpointTuner.java` (Act on first #1). `git fetch upstream` works.
- **Upstream modules untouched:** `git diff --stat upstream/master -- FtcRobotController/ build.common.gradle` is empty, as CLAUDE.md asks.
- **Size:** no tracked file over 1 MB. The largest are `README.md` (140 KB), `AgateFlow.java` (112 KB), `gold.wav` (104 KB), `silver.wav` (100 KB).
- **Ignored as expected:** `local.properties`, `build/`, `*.apk`. `libs/` (2 files, kept by `!libs/*.aar`) and `doc/` (4 files) are tracked.
- **CODE_REVIEW.md** is local only through `.git/info/exclude:7`, and stale (see above).

## Suggested next steps

- [ ] Merge `upstream/master` (2 commits, `PinpointTuner.java` only) before tuning step 2.
- [ ] Fix the code bugs from Act on first: the `DriveRoute` detour flag (#2), the `FireControl` dip from above target (#3), `resetEdgeDetection()` after INIT in `MainTeleOp` and the test OpModes (#4), the `DriveRoute` zone on a stop (#5), and Shooter Test holding a fitted `CoaxHood` (#6).
- [ ] Add JUnit tests: `FireControl` (coax dip rule, mid-feed release, `READY_HOLD_MS`, misfeed timeout), `Robot` decisions on a fake `HardwareMap`, and `DriveRoute` (replan, detour, deadline) plus `ParkGuard` on `TestRobot`.
- [ ] Add the missing TODO(16) comments: `Field.PASS_TARGET_RED_X/Y`, `ShotTables.PASS_RPM` / `PASS_FEED_POWER`, `HardwareNames.HOOD_SERVO`, `MainTeleOp.HOOD_TRIM_STEP_DEG`, `CoaxHood.IDLE_LAUNCH_DEG`, and a note tying `HoodSolver.SEED_RPM_WINDOW` to `Robot.COAX_RPM_TOLERANCE`.
- [ ] Fix the TODO_LIST intro in `TeamCode/build.gradle:117-118` so it names phase 5 (the coax hood).
- [ ] Commit the uncommitted work on a branch. Commit the `.gitignore` change that ignores `.claude/` and `research/`.
- [ ] Refresh `CODE_REVIEW.md` (refs, counts, V3 status) or delete it.
- [ ] Bump `com.bylazar:fullpanels` to 1.0.18 (`build.dependencies.gradle:22`), and check `tuning` and `ivy:pedro` by hand.
- [ ] Decide on the Java 8 warning (a Gradle toolchain or the suppress flag), and find the Gradle 10 deprecation before moving past wrapper 9.1.0.
