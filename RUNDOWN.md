# Robot rundown: TeleOp and auto commands

How the robot code runs. Part 2 follows Main TeleOp one loop at a time. Part 3 lists every command an autonomous can use. File paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, and `Robot.java:180` means line 180 of that file.

## 1. Overview

OpModes don't decide anything. Main TeleOp turns buttons into requests on `Robot`, like `robot.shootHeld = pad.right_bumper`. An auto schedules Ivy commands that set the same requests and wait on what `Robot` reports. `Robot.update()` runs once per loop and decides the rest: where the turret points, how fast the wheel spins, when the door opens, what the rail does. TeleOp and auto share one brain, so a fix to aiming or firing helps both.

## 2. How TeleOp runs

### 2a. INIT

`MainTeleOp.runOpMode()` (`opmodes/teleop/MainTeleOp.java:55`):

1. `Scheduler.reset()` (line 57) clears Ivy commands left over from the last OpMode.
2. `new Robot(hardwareMap)` (line 58) builds the Pedro `Follower` with `Constants.create` (it throws a clear error while that still returns null), then every subsystem. Constructors only look up hardware and set motor modes. No motor power or servo position is sent in INIT (G304.H, G403). The coax hood is built only if the configuration has its servo (`CoaxHood.isConfigured`).
3. `new Feedback(gamepad1)` (line 59): rumble and light, driver 1 only.
4. `new AllianceSelector(gamepad1, gamepad2)` (line 61) calls `RobotState.forgetIfStale()`: with no pose saved in the last 60 s (`RobotState.MAX_SAVED_AGE_SEC`), the alliance from an old match is cleared.
5. The init loop (lines 65-74). D-pad up on either gamepad switches the alliance (the first press picks red). Both gamepads light red, blue, or white for none. Switching drops the saved pose, since it is in the other alliance's frame (line 68 and `RobotState.setAlliance`). The screen says whether a saved pose will be used.

At Start (lines 77-84):

- `gamepad1.resetEdgeDetection()` and `gamepad2.resetEdgeDetection()` throw away presses made in INIT. A `...WasPressed()` call remembers a press until it is read, and most buttons aren't read in INIT. Without the reset, a thumb resting on LB or D-pad right during INIT would toggle the intake or relocalize in the first loop.
- Saved pose from auto: `robot.setPose(savedPose, true)`. The pose is trusted.
- No saved pose, alliance picked: the pose is set to the right corner with the intake facing the driver (`Corner.RIGHT_INTAKE_TO_DRIVER`), untrusted. Field-centric driving uses that heading right away. Nothing fires until a relocalize.
- No alliance: no pose is set at all.

Panels: telemetry goes to the Driver Station and Panels at once (`JoinedTelemetry`, line 56). Every `public static` field in a `@Configurable` class can be changed live from Panels in practice. Panels is not allowed at events (R704.D): `PanelsSettings.EVENT_BUILD = true` keeps it off.

### 2b. The loop

`MainTeleOp.java:88-106`, in code order:

| # | Code | What it does |
|---|---|---|
| 1 | `driverOne(gamepad1)` (line 93, body 111-131) | Sticks go to `robot.drive.drive(...)`, field-centric (as red until an alliance is picked). Every loop: `robot.shootHeld` = RB, `robot.passHeld` = RT, `robot.unjamHeld` = LT (triggers count past `TRIGGER_THRESHOLD`, 0.5). LB press: `robot.flowerIntake.toggle()`, the only thing that moves the flower intake in TeleOp. D-pad right/left, Circle, Square: `robot.relocalize(...)` with the matching `Corner`. |
| 2 | `driverTwo(gamepad2)` (line 94, body 133-145) | D-pad left/right: `robot.turret.addTrim` by `TURRET_TRIM_STEP_DEG` (2°, left = counter-clockwise). D-pad up/down: `robot.shooter.addTrim` by `RPM_TRIM_STEP` (25), or `robot.coaxHood.addTrim` by `HOOD_TRIM_STEP_DEG` (0.5°) while `robot.usingCoaxHood()`. |
| 3 | `Scheduler.execute()` (line 95) | Runs Ivy commands. TeleOp schedules none yet; the call is there so one-button macros can be added. A macro that sets `shootHeld`, `passHeld` or `unjamHeld` must be OR-ed into `driverOne`, or the button overwrites it next loop. |
| 4 | `robot.update()` (line 96) | Reads sensors, decides, writes hardware. See 2c. |
| 5 | `feedback.update(robot.storage, robot.warnings())` (line 97) | Rumble for new warnings and full storage, light for the ball count. |
| 6 | Telemetry (lines 99-105) | Warnings and loop time, plus everything when `FULL_TELEMETRY` is true. With it false, only every `TELEMETRY_INTERVAL_MS` (200 ms). |

Not in this list: the bulk-read clear and the battery reading. `Robot.update()` clears the bulk cache at its top, and `VoltageCache` re-reads the battery at most every 250 ms, when the shooter or turret asks for `compensation()`.

Drive powers: `driverOne` only tells the `Follower` what to do (`follower.manual`, or hold). The `Follower` writes the drive motors in `drive.update()`, inside `robot.update()`. With the sticks let go, `Drive.drive` calls `ManualDrive.driveOrHold`, which holds position once the robot is under `HOLD_BELOW_SPEED` (2 in/s), so a defender can't push it off a shot.

### 2c. Inside `Robot.update()`

`robot/Robot.java:180-261`. Read first, decide, write last, so every decision uses this loop's readings.

1. **Mode** (182-184): `shootHeld` means SHOOT. Else `passHeld` with `FireControl.PASS_ENABLED` means PASS. Else NONE. Shoot wins if both are held.
2. **Read** (186-192): `bulkReads.clear()`, `drive.update()` (odometry in, drive motors out), `storage.update()` (beams), `shooter.readSensors()` (wheel RPM), `turret.readSensors()` (turret angle).
3. **Hood switch** (196-204): only while no burst is firing. If `Robot.HOOD` changed which hood shoots, the RPM and hood trims reset.
4. **Aim** (206-208): `solve()` (312-326) gives null with no alliance, or a NaN in the pose or velocity. Otherwise it picks the target cell by which half of the field the robot is on (`Field.cellForSide`, with a 6 in. band so it doesn't flicker), then solves the turret angle, RPM, feed power, in-range, and the coax launch angle.
5. **Last good aim** (210-218): no aim in the middle of a burst keeps the last RPM, turret angle and feed power, so a ball already fed isn't ruined. No aim and no burst: turret target 0, wheel 0.
6. **Targets** (221-230): turret to the aim angle. Wheel to the aim RPM while a burst is firing or any ball is stored, else 0. A fitted coax hood is aimed and written here, early, so `atTarget()` already counts this loop's move.
7. **Ready** (232-235):
   - `aimed()` (282): an aim that is in range, `canFire()` (alliance and trusted pose, line 162), `turret.onTarget()` (within `ON_TARGET_DEG`, 3°, and not off or being tested), and the hood. Static with a coax hood fitted: the hood is at `STATIC_LAUNCH_DEG`. Coax: some angle scores, and the hood is there.
   - `wheelReady()` (295): static, `shooter.atSpeed()` (within `RPM_TOLERANCE`, 90). Coax, within `COAX_RPM_TOLERANCE` (300).
   - Enough balls: 2 in TeleOp (`storage.hasAtLeastTwo()`), 1 when `singleBallBursts` is on (auto).
8. **Fire** (236-238): `fireControl.update(...)`, then `shooter.setBoost(isFiring())`.
9. **Door** (241-242): open while `FireControl` says so.
10. **Rail** (247-251), first match wins: `unjamHeld` runs it backwards. Feeding runs it at the aim's feed power. A burst going, or the door not fully closed: stopped (the rail holds balls back). Otherwise it collects.
11. **Write** (253-256): `door.update()`, `flowerIntake.update()`, `shooter.update()`, `turret.update()`. `Robot` doesn't pick the flower intake's position, even with storage full: it writes what LB or an auto command last set.
12. **Save** (260): the pose goes to `RobotState` only if trusted and not NaN.

| Guard | What happens |
|---|---|
| No alliance | No aim: turret to 0, wheel off, nothing fires. |
| Alliance, pose not trusted | Aims and spins up from the guessed pose, but `canFire()` is false, so no burst starts. |
| NaN pose or velocity | No aim. A burst already going keeps the last good targets, but the rail pauses (not aimed). The pose isn't saved. |
| Shooter watchdog | Power over 0.5 for `Shooter.WATCHDOG_MS` (500) while RPM stays under 100: shooter off until the OpMode restarts. |
| Turret watchdog | Turns against its own push (REVERSED flags disagree), or full power that doesn't close the error, or 3 failed stuck tests in a row: off until restart. A held turret rests and retries first. |

| Subsystem | Reads | Writes |
|---|---|---|
| `Drive` | Odometry, in `follower.update()` | Four drive motors |
| `Storage` | Three break beams, debounced | Nothing |
| `Shooter` | Wheel speed, in `readSensors()` | Both wheel motors, in `update()`: kS + kV + kP + kA, times voltage compensation. Boost: full power when 50+ RPM under target. |
| `Turret` | Absolute encoder, in `readSensors()` | CR servo power, in `update()` (PIDF plus KS) |
| `CoaxHood` (optional) | Nothing: no feedback, it estimates arrival from `SPEED_DEG_PER_SEC` | Servo position |
| `Door` | Nothing: timers say fully open or closed | Servo position |
| `Rail` | Nothing | Intake and transfer motors, right inside `collect`, `feed`, `reverse`, `stop` (no `update()`) |
| `FlowerIntake` | Nothing | Two servos |

### 2d. Shooting, as the driver feels it

`robot/FireControl.java` runs each burst in three steps: WAIT, FIRE, CLOSE. Its tests are in `FireControlTest`.

- **Hold RB.** A burst starts once 2 or more balls are stored (light yellow or green). The wheel is usually at speed already, since it spins whenever a ball is stored.
- **WAIT** (line 144): the shot must stay ready for `READY_HOLD_MS` (40 ms). Then the door opens, and the rail starts once it is fully open (`Door.OPEN_TIME_MS`, 150 ms).
- **FIRE** (line 159): the rail runs without stopping while RB is held. The feed power (`ShotTables.SHOT_FEED_POWER`, by distance) sets the gap between balls. The wheel sags as balls go through, and ready is not asked again. The rail pauses only while the aim is off or the wheel is more than `BURST_RPM_FLOOR` (600) under target, and goes again when both are back. Running out of balls doesn't end the burst.
- **CLOSE** (line 170): let go and the rail stops at once. The door stays open `DOOR_CLOSE_DELAY_MS` (100 ms) so it doesn't close on a ball already past it. Then it closes, and the rail stays stopped until it is fully closed (`Door.CLOSE_TIME_MS`, 150 ms), then collects.
- **Shot count**: a shot counts when the RPM falls `SHOT_DIP_RPM` (150) under the highest reading of the last `DIP_WINDOW_MS` (100 ms). It is for telemetry and auto's `shoot()`. If the rail runs `NO_SHOT_WARN_MS` (400) with no shot counted, the "Fire" line says "jam?".
- **Trims** (driver 2): turret ±2°, RPM ±25, or hood ±0.5° with the coax hood. A relocalize resets all of them.
- **Hood switch**: `Robot.HOOD` in Panels, applied only between bursts. It resets the RPM and hood trims.
- **Pass (RT)**: does nothing now. `FireControl.PASS_ENABLED` is false, so `Robot` treats RT as not held. Once on, switching between shoot and pass mid-burst ends the burst, and the new one waits for its own ready.
- **Unjam (LT)**: rail backwards. It beats everything, even a burst.
- **Relocalize**: push into a corner on your alliance wall and press D-pad right/left (intake facing you) or Circle/Square (intake facing the side wall). Needs an alliance. The pose becomes trusted and all trims reset (`Robot.java:144`).

Warnings come from `Robot.warnings()` (`Robot.java:329`), with text in `robot/Warning.java`. Telemetry always shows them. `Feedback.update` (`driver/Feedback.java:63`) buzzes 3 strong pulses when one appears, and the same one buzzes again only after `WARNING_COOLDOWN_MS` (3 s).

| Warning | Means | Buzzes |
|---|---|---|
| `NO_ALLIANCE` | No alliance picked in init. Restart and pick one. | Yes |
| `POSE_UNKNOWN` | Pose not trusted. Relocalize before shooting. | Yes |
| `POSE_NAN` | Odometry glitch. Not aiming, not saving the pose. | Yes |
| `OUT_OF_RANGE` | Target outside the shot table. Coax: wheel near target but no hood angle scores. | No, screen only |
| `SHOOTER_OFF` | Shooter watchdog tripped. Restart. | Yes |
| `TURRET_OFF` | Turret watchdog tripped. Restart. | Yes |
| `SHOT_TABLE_ORDER` | A Panels edit put a table's distances out of order. | Yes |
| `HOOD_MISSING` | `HOOD` is COAX but there is no hood servo, so it shoots with static numbers. | Yes |
| `HOOD_AT_END` | The hood servo is asked past its range, so shots miss. Check the `CoaxHood` mapping. | Yes |

Also: a soft constant rumble while storage is full. Light: red 0-1 balls, yellow 2-3, green 4.

### 2e. Stop

After the loop (lines 107-108): `feedback.stop()` ends the rumble, and `robot.stop()` calls `drive.stop()`, which stops the `Follower`. Letting go of a path makes Pedro put back any speed limit the path wrote into the Foresight settings. `robot.stop()` touches nothing else.

## 3. Commands for autonomous

### 3a. The auto loop

`opmodes/auto/AutoTemplate.java` is the skeleton to copy (`@Disabled`). Its shape:

```java
Scheduler.reset();                          // line 55: Scheduler is static
Robot robot = new Robot(hardwareMap);
RobotCommands commands = new RobotCommands(robot);
AllianceSelector allianceSelector = new AllianceSelector(gamepad1, gamepad2);
while (opModeInInit()) { allianceSelector.update(screen); screen.update(); }
if (RobotState.alliance == null) { /* line 70: do nothing all auto */ return; }
robot.setStartPose(start);                  // written for red, flipped for blue, trusted
Scheduler.schedule(sequential(              // line 86: one sequential, see section 4
        commands.driveTo(shootSpot, DRIVE_TIMEOUT_MS),
        commands.shootAllWithRetry()));
while (opModeIsActive()) {
    // park.update();                       // with a ParkGuard: before execute
    Scheduler.execute();                    // commands set what Robot should do
    robot.update();                         // Robot does it, this same loop
}
robot.stop();                               // line 108: puts back any path speed limit
```

With no alliance, auto sits still: shooting at the wrong goal is worse. `Robot.update()` saves the trusted pose each loop and `preselectTeleOp = "Main TeleOp"` queues TeleOp, so TeleOp starts with auto's pose and alliance.

### 3b. `RobotCommands`

`robot/commands/RobotCommands.java`. Make one with `new RobotCommands(robot)`. Commands never touch hardware. They set `Robot` fields (the same ones the gamepads set) and wait on what it reports. Poses are written for red.

| Command (line) | What it does | Ends when | On end or interrupt | Requires |
|---|---|---|---|---|
| `driveTo(Pose redPose)` (79) | Straight line from where the robot is at start to the flipped pose, heading turning evenly. Already there: just holds the pose. Throws if no alliance. | End of the line (the follower keeps holding it) | Interrupted: holds where it is | `robot.drive` |
| `driveTo(Pose redPose, double timeoutMs)` (85) | Same, raced with `Commands.waitMs(timeoutMs)` | Line done, or timeout | Timeout: holds where it is | `robot.drive` |
| `follow(Path path)` (93) | Any Pedro path, already in this alliance's coordinates (no flip). No time limit: add `.raceWith(Commands.waitMs(ms))`. | End of the path | Interrupted while following: holds | `robot.drive` |
| `withFlowerIntake(DriveRoute drive)` (123) | Runs an AgateFlow drive and moves the flower intake to match: down on `flowerIntakeDown()` stretches, up on the rest. Moves it only when the stretch changes. Full storage doesn't raise it; `flowerUp()` does. | The drive ends | Drive not ARRIVED: intake up. Arrived on a down stretch: stays down. | The drive's, plus `robot.flowerIntake` |
| `shoot()` (195) | Sets `singleBallBursts` and `stopAfterOneShot`, holds `shootHeld` until a new shot is counted. Needs `SHOT_DIP_RPM` tuned: an uncounted shot keeps feeding until the timeout. | Not firing, and a shot counted or storage empty (right away if empty), or `SHOOT_ONE_TIMEOUT_MS` (1500) | `stopShooting()` | `robot.shooter` |
| `shootAll()` (215) | Sets `singleBallBursts`. Holds `shootHeld` while a ball is stored, and `LAST_BALL_MS` (300) after storage reads empty mid-burst, so the last ball reaches the wheel. | Storage empty and burst over, or `SHOOT_ALL_TIMEOUT_MS` (3000) | `stopShooting()` | `robot.shooter` |
| `shootAllWithRetry()` (229) | A `Retry`: `shootAll()`. Balls left: `unjam(RETRY_UNJAM_MS)` (200), then `shootAll()` again, up to `SHOOT_RETRIES` (1) times. | Storage empty after a try, or out of retries | Ends the running try once | `robot.shooter`, `robot.rail` |
| `shootContinuous()` (242) | Shoots whenever a ball is stored, lets go once empty so the rail can collect | Never by itself | `stopShooting()` | `robot.shooter` |
| `sweepAndShoot(Command drive)` (250) | The drive, with `shootContinuous()` alongside | The drive ends | Shooting is let go | The drive's, plus `robot.shooter` |
| `flowerDown()` (293) | Flower intake down | At once | Nothing | `robot.flowerIntake` |
| `flowerUp()` (301) | Flower intake up, then waits `FLOWER_RAISE_MS` (400) | After the wait | Nothing | `robot.flowerIntake` |
| `waitUntilFull(double timeoutMs)` (308) | Waits. The rail collects on its own. | Storage full, or timeout | Nothing | None |
| `unjam(double ms)` (313) | Sets `unjamHeld` | After `ms` | Clears `unjamHeld`, even if cancelled | `robot.rail` |
| `parkGuard(AgateFlowCommands agateFlow, Route parkRoute, DoubleSupplier secondsLeft)` (283) | Not a command: returns a `ParkGuard` that stops the shooter, rail and flower intake commands too, and holds the flower intake up while parking. See 3d. | | | |

`stopShooting()` (266) is the private cleanup every shooting command ends with. It clears `shootHeld`, `singleBallBursts` and `stopAfterOneShot`, and is safe to run twice.

### 3c. `Retry`

`robot/commands/Retry.java`. `new Retry(first, retry, succeeded, maxRetries)` runs `first`. When it ends, even by its own timeout, it asks `succeeded`. False with retries left: it starts `retry`. After the last retry it ends anyway, so the auto moves on instead of hanging. It holds the requirements of both tries the whole time, so nothing grabs the shooter between tries. Each try is ended exactly once, even if Ivy ends the `Retry` twice. That is why it exists: retries built from Ivy groups can end a child twice or never (section 4). Give each try its own timeout and let the check decide. Tests: `RetryTest`.

### 3d. AgateFlow commands

AgateFlow plans paths around obstacles on the fly. **No match OpMode uses it yet**, and it has only been checked on the desktop (JUnit and a simulated drivetrain), not on the robot.

```java
AgateFlowCommands agateFlow = new AgateFlowCommands(robot.drive.follower(), robot.voltage::get, robot.drive);
```

| Call (`agateflow/AgateFlowCommands.java`) | What you get |
|---|---|
| `agateFlow.driveTo(route)` (126) | A `DriveRoute` command. It plans on a background thread from where the robot is when it starts, then drives each leg. Until the first plan is ready the robot stays as it was. |
| `agateFlow.driveTo(redPose)` (131) | Same, one stop. |
| `agateFlow.estimate(route)` (152) | An `Estimate`: times the route from here without driving. Check `ready()`, then `seconds()` or `plan()`. |
| `agateFlow.driveTo(estimate)` (142) | Starts with the estimate's plan if it still fits (robot within 1 in. and 5° of where it was made, or the estimate is under 0.5 s old and the robot is on its predicted path). Otherwise plans again. |
| `DriveRoute.planFrom(fieldStart)` (`DriveRoute.java:161`) | Plans in init, so the first drive doesn't wait. |
| `agateFlow.map().setZones("vision", zones)` | Obstacles from vision. Safe to call from the camera thread. |

What a `DriveRoute` does while driving (class comment, `DriveRoute.java:20-55`):

- **Replans** (checked every `REPLAN_EVERY_MS`, 250) when new zones block the path ahead, or the robot is more than `REPLAN_OFF_PATH` (8 in.) off its predicted path (pushed). It keeps driving the old path meanwhile.
- **Brakes for a new zone**: every loop while the zones differ from the plan's, it checks the path 30 in. ahead. A block there makes it brake straight against its motion at `STOP_BRAKING_POWER` (0.5) until nearly still, hold, and replan from there. Near a stop, only a zone the robot would hit counts (a partner parked on the stop).
- **Drops stale replans**: one that comes back after the robot moved on to the next leg, or one asked for before the robot stopped for a block when the robot is now more than 3 in. off that replan's predicted path.
- **Deadline**: `TIMEOUT_SCALE` (1.5) times the first estimate plus `TIMEOUT_EXTRA_SECONDS` (1), counted from the first plan. Replans for a push can move it at most `REPLAN_EXTRA_SECONDS` (5) later. A detour around a new zone gets its own plan's time, at most `MAX_DETOURS` (3) times. A robot still getting closer at the deadline gets up to `MAX_OVERTIME_SECONDS` (3) more.
- **Ends** with `status()` ARRIVED, NO_PLAN, TIMED_OUT or STOPPED. Anything but ARRIVED holds where it is, and `problem()` says why. Clean legs teach `EtaCalibration` how real times compare to predicted ones.

`ParkGuard` (`agateflow/ParkGuard.java`) makes sure auto ends parked:

- Build it with `commands.parkGuard(agateFlow, PARK_ROUTE, () -> 30 - matchTime.seconds())`. Call `park.update()` (line 118) every loop, before `Scheduler.execute()`.
- Every `REFRESH_MS` (250) it re-estimates the park drive from where the robot is. It waits while a drive's own plan is being made, since they share one planning thread.
- When the time left drops to the estimate plus `MARGIN_SECONDS` (0.75) and the planning time, it schedules the park drive at `PRIORITY` 1000, requiring the drive and every subsystem passed in. That interrupts every auto command using them, and their `end()` runs, so shoot is let go. With no good estimate yet it parks `FALLBACK_SECONDS` (5) before the end. Already within 2 in. of the park spot: it doesn't start.
- A park drive that ends without arriving (no plan, out of time, interrupted, never let start) is started again while more than 0.5 s is left.
- The flower intake is held up for the whole park drive. If it was down at a FLOWER, the plan backs out with it still reaching ahead while it comes up.

`Route` cheat sheet (`agateflow/Route.java`). Poses are written for red. Calls like `heading()` change the waypoint added just before them.

| Call | Meaning |
|---|---|
| `to(redPose)`, `to(x, y, headingDegrees)` | Stop and settle there. |
| `to(x, y)` | Stop there, no heading asked: may arrive side-on or backward if that fits. |
| `through(...)` | Pass it without slowing. |
| `toLive(fieldPose)`, `throughLive(fieldPose)` | Pose asked on the main thread at the start and every replan. Field coordinates. Null means no plan. |
| `pickUp(redTargets...)`, `most(count)`, `inOrder()`, `pickUpLive(fieldTargets)` | Drive over floor targets intake first, in the fastest order. `Plan.pickedUp()` and `Plan.skipped()` say which. |
| `heading(heading)` | How to turn on the way there. |
| `approach(travelDegrees, inches)` | Arrive driving straight this way for the last stretch. |
| `maxSpeed(inchesPerSecond)` | Speed limit on the way there. |
| `flowerIntakeDown()` | Plan this stretch with the flower intake reaching ahead. Pair with `withFlowerIntake`. |
| `allowContact()` | Let the robot hug walls and zones on this stretch. Add a `maxSpeed`. |

`Heading` options: `linear()` (default, turn evenly), `finishBy(fraction)`, `between(startFraction, endFraction)`, `tangent()`, `reverseTangent()`, `facing(x, y)` (written for red). Tight gaps want `tangent()`, no heading, or a through-waypoint before the gap.

```java
DriveRoute toFlower = agateFlow.driveTo(new Route().to(FLOWER).approach(FLOWER_DEGREES, 12).flowerIntakeDown());
sequential(commands.withFlowerIntake(toFlower), commands.waitUntilFull(1500), commands.flowerUp())
```

### 3e. `DriverAssist`

`agateflow/DriverAssist.java` is one-button driving for TeleOp along a `Route`. Call `update(button)` before `Scheduler.execute()`. A press schedules a `DriveRoute`. Touching the sticks cancels it at once. A second press starts again from where the robot is. `driving()` tells the OpMode not to drive with the sticks, and `failed()` is true after no plan or a timeout. It is **not** in `MainTeleOp`.

## 4. Ivy gotchas we code around

- Groups can call `end()` on a child that never started, or twice. Cleanup must be safe to repeat (`stopShooting()`, `Retry`, `DriveRoute`).
- Scheduling a second top-level command that shares a requirement cancels the first. Keep a routine in one `sequential(...)`.
- `Command.unless()` never finishes. Don't use it.
- `Scheduler.reset()` doesn't call `end()`. Call it at the top of every OpMode.
- `PedroCommands.follow` has no `end()`, so a cancelled follow keeps driving. Our `follow`, `driveTo` and `DriveRoute` hold on interrupt.
- Pedro never times out a follow. Every auto drive needs a limit: `driveTo(pose, ms)`, `.raceWith(Commands.waitMs(ms))`, or AgateFlow's deadline.
- `.requiring(...)` replaces a command's requirements, including a group's union.
- `waitMs` uses the wall clock.
- `hold(follower)` reads the pose when the command is built, not when it runs. Build it late (`driveTo` uses `Commands.lazy`).

## 5. Where to look

| File | What |
|---|---|
| `opmodes/teleop/MainTeleOp.java` | Button mapping and the TeleOp loop |
| `opmodes/auto/AutoTemplate.java` | Auto skeleton to copy |
| `robot/Robot.java` | `update()`, the per-loop decisions; `warnings()` |
| `robot/FireControl.java` | Burst steps and the shot count (`FireControlTest`) |
| `robot/subsystems/` | One class per mechanism |
| `robot/shot/ShotSolver.java`, `ShotTables.java` | Aim, RPM and feed power |
| `robot/RobotState.java` | Alliance and pose carried between OpModes |
| `robot/Warning.java`, `driver/Feedback.java` | Warning text, rumble and light |
| `driver/AllianceSelector.java` | Alliance picking in INIT |
| `robot/commands/RobotCommands.java`, `Retry.java` | Auto building blocks (`RetryTest`) |
| `agateflow/AgateFlowCommands.java`, `DriveRoute.java`, `ParkGuard.java`, `Route.java`, `Heading.java`, `DriverAssist.java` | AgateFlow drives (`DriveRouteTest`, `ParkGuardTest`) |
| `TELEOP_PLAN.md`, `TUNING_GUIDE.md` | Button map and field numbers; how to tune |
