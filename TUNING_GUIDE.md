# Tuning and measuring guide

How to measure and tune everything the code can't know until someone tries it on the robot or the field. The steps are in order of importance and in the order they depend on each other, so work top to bottom:

| Phase | Steps | Done when |
|---|---|---|
| 1. TeleOp, bare minimum | 1 to 8 | The robot plays a TeleOp match: it drives, intakes, counts balls, aims and scores standing still, and knows where it is after a relocalize. |
| 2. Autonomous and competition | 9 to 10 | A preloaded auto drives, scores and parks, and TeleOp starts with auto's pose. Ready to compete. |
| 3. Make it better | 11 to 12 | Shooting while driving, and the drivers like how it feels. |
| 4. AgateFlow and auto pickups | 13 to 15 | Autos plan their own paths, pick up POLLEN from FLOWERS, the GARDEN and the floor, and always park in time. |
| 5. Coax hood | 16 | Only once the coax hood is on the robot: the hood angle is solved from the wheel's real speed, so shots score from more places and bursts go faster. |

- **What's left:** `TODO_LIST.md`, built from the `// TODO(step): ...` comments in the code (Android Studio's TODO window shows them too), grouped the same way. Delete a TODO comment when its item is done.
- **"Only if" items:** many items say "raise it if ..." or "only if ...". If you never see the problem, the default is fine: delete the TODO and move on.
- **Live tuning:** every tunable is in **Panels** (robot Wi-Fi, then Panels in a browser), including the shot tables and auto poses. Changes are lost when the robot app restarts, so **copy every final value back into the code**. Panels is for practice only: the Competition Manual names it as a prohibited streaming tool (R704.D), so it stays off at events and the robot plays on the values in the code (step 10).
- Each item names where its value goes: `Class.FIELD`.

# Phase 1: TeleOp, bare minimum

## 1. Wiring and configuration

- [ ] Make the robot configuration on the Driver Station with the names in `robot/hardware/HardwareNames.java`. Write each port into the hardware table in `ROBOT.md`.
- [ ] Confirm all 8 motors and 4 servos (5 with the coax hood) show up (Driver Station, Configure Robot, Scan).

## 2. Drivetrain and localization (Pedro)

- [ ] Run the `Drive Test` OpMode first (no tuning needed). Hold the D-pad to spin one wheel at low power: up = frontLeft, right = frontRight, down = backRight, left = backLeft. Each must roll the robot forward. If a wheel goes backward, flip that motor's direction in `Constants.drivetrainConfig`. If a different wheel spins, the port is wrong.
- [ ] In `Drive Test`, check the sticks: left stick up drives toward the intake end, left stick left strafes left, right stick X left turns left. Right trigger held is slow mode.
- [ ] In `Drive Test`, check the Pinpoint (Options resets the pose to 0, 0, 0). Push the robot 48 in. forward by hand and x rises 48. Push it 48 in. left and y rises 48. Turn it left and heading goes positive. If x or y runs the wrong way, flip that pod's direction in `Constants.localizerConfig`.
- [ ] Pinpoint pod offsets: `ROBOT.md` has them from CAD, measured from the **turret center** (the robot's center in all our code). PinpointTuner confirms them; fix `ROBOT.md` if it disagrees.
- [ ] Run the tuners in order: `MecanumTuner`, `PinpointTuner`, `ForesightTuner`. They are registered in `pedro/TeamTuning.java`; only `Tests` gets added there later, once the Foresight output is pasted into `Constants` (see the TODO in that file).
- [ ] While ForesightTuner runs, write down the battery voltage as `DriveModel.TUNED_VOLTS`. Only AgateFlow uses it (step 13), but this is the only time to read it without running the tuner again.
- [ ] Paste their output into `pedro/Constants.java`. It already holds starting `drivetrainConfig` and `localizerConfig` values; the tuners' output replaces them. Fill in `create()` once `foresightConfig()` exists. The ForesightTuner output goes inside a `foresightConfig()` method, not in a field: the comment in that file shows how. Then every OpMode starts from a fresh config, and a path speed limit left on when an OpMode stopped can't carry over. Until this is done, Main TeleOp stops at init with an error saying so.
- [ ] Only if you change Foresight's braking settings (Pedro 3.0.1): set `maxDecelerationScale` or `maxDecelerationConstraint`, not both, since setting the scale throws the constraint away. `brakeAggression` above 1 overshoots the end of a path and below 1 stops short, the opposite of what Pedro's docs page says: raise it if the robot stops short.
- [ ] Run the Pedro `Tests` procedure (Localization, Line, Curve).
- [ ] Main TeleOp, red alliance: stick up drives **away from the driver**, stick left drives to the **driver's left**. Then the same on blue.
- [ ] Robot forward (heading 0) is the **intake end**. If Pedro's forward is the other end, the corner poses and the aiming are 180° off.
- [ ] `Drive.STICK_DEADBAND`: raise it if the robot creeps with the sticks let go.
- [ ] `Drive.HOLD_WHEN_STOPPED`: drive around and let go. If holding feels jumpy, set it to false.
- [ ] `Drive.HOLD_BELOW_SPEED`: with the sticks let go, the robot coasts until it's slower than this (in/s), then holds its pose. Raise it if it rolls on before holding. Lower it if the hold feels jumpy.

## 3. Storage and rail (Storage Test OpMode)

- [ ] First, `Door.CLOSED_POSITION` and `Door.OPEN_POSITION` with the Servo Position Finder OpMode. Storage Test holds the door closed, and the balls stack up against it for the rest of this section.
- [ ] `Storage.BLOCKED_READS_LOW`: block each beam by hand. Telemetry must show `true` when blocked. If it's backwards, flip this.
- [ ] `Rail.INTAKE_REVERSED`, `Rail.TRANSFER_REVERSED`: with Cross (collect) on, both ends must move balls **toward the door**.
- [ ] `Rail.TRANSFER_HOLD_POWER`: with a ball against the closed door, the lowest power that keeps it seated without the motor straining.
- [ ] `Rail.INTAKE_POWER`, `Rail.TRANSFER_INTAKE_POWER`: lower them if balls jam or bounce.
- [ ] `Rail.REVERSE_POWER` (-1.0, the unjam power): hold Circle with balls loaded. They must back out without flying out of the intake. Move it toward 0 if they do.
- [ ] `Storage.FULL_HOLD_MS`: feed balls one at a time. "Full" must only appear on the 4th. Raise it if a passing ball triggers full.
- [ ] `Storage.DEBOUNCE_MS`: raise it (in ms) if the count flickers. A beam change only counts after it holds this long.
- [ ] Watch the count go 0, 1, 2-3, 4 as balls come in.

## 4. Servos (Servo Position Finder OpMode)

The door positions were found in step 3.

- [ ] `FlowerIntake.UP_POSITION`, `FlowerIntake.DOWN_POSITION`: find them on the **left** servo. The right one is reversed in code.
- [ ] `FlowerIntake.RIGHT_REVERSED`: in Main TeleOp, LB must move both servos the same way. Flip it right away if they fight, or the servos can burn out.
- [ ] Check the flower intake reaches into a real FLOWER's Retrieval Opening (3.55 in. tall) and pulls POLLEN out.
- [ ] `Door.OPEN_TIME_MS`: time from closed to fully open. Film the door in slow motion in Shooter Test with the wheel off and no balls: hold RB to open it, let go to close it. Or, after step 6, start high and lower it until a ball catches the door.
- [ ] `Door.CLOSE_TIME_MS`: time from fully open to closed. The rail waits this long after a burst before pushing the next ball toward the door. Film it the same way. Or, after step 6, in Main TeleOp, start high and lower it until a ball reaches the door while it's still closing.

## 5. Turret (Turret Test OpMode)

Manual mode first (Cross toggles; left stick X drives the servo, capped at `Turret.MAX_POWER`):

- [ ] `Turret.SERVO_REVERSED`: stick **left** must turn the turret **counter-clockwise** (seen from above).
- [ ] `Turret.ENCODER_FULL_TURN_VOLTS`: turn the turret slowly through several full turns and watch "Encoder volts". Use the highest clean reading before it jumps back to 0 (Meta-Infinity found 3.2 V on the same encoder).
- [ ] `Turret.ENCODER_MIN_VOLTS`: same slow turns. Use the lowest clean reading just after the voltage drops at the wrap. Leave it 0 if it reads 0.000 there. Set it before `ENCODER_ZERO_DEG`, since changing it moves every angle a little.
- [ ] `Turret.ENCODER_REVERSED`: the angle must **increase** when the turret turns counter-clockwise. If the two REVERSED flags disagree, aiming mode turns the turret off within about 1.5 s ("turned ... against its push"): fix the flags and restart the OpMode.
- [ ] `Turret.ENCODER_ZERO_DEG`: point the shooter straight out the robot's front and read the angle. Put that number here so the angle reads 0.
- [ ] `Turret.KS`: manual mode sends the stick to the servo without KS, so changing KS there does nothing. Instead, push the stick very slowly and watch "Turret error / power". Note the power where the turret just starts to move, and set KS a little below it.

Aiming mode (D-pad picks targets, bumpers nudge ±10°):

- [ ] `Turret.FAR_KP`, `Turret.FAR_KD`: big moves (D-pad 0 to 180). Fast, small overshoot.
- [ ] `Turret.CLOSE_KP`, `Turret.CLOSE_KD`: small moves (bumper nudges). Settles without buzzing.
- [ ] `Turret.CLOSE_ZONE_DEG`, `Turret.KS_RAMP_DEG`: adjust if it chatters near the target.
- [ ] `Turret.D_FILTER`: only if the turret buzzes, or the D number in "Turret P / I / D / F / KS" jitters while it holds still, raise it toward 0.5 (0 = no smoothing). Too high and D reacts late, so it overshoots more.
- [ ] `Turret.KI`, `Turret.I_ZONE_DEG`, `Turret.I_MAX_POWER`: only if it stops a degree or two short of the target and stays there (bumper nudges). Set `I_ZONE_DEG` a little above the biggest gap and `I_MAX_POWER` a little above `KS`, then raise `KI` from 0 until the gap closes without overshoot. In "Turret P / I / D / F / KS", I should grow while the turret is short and drop to 0 once it crosses the target.
- [ ] `Turret.MAX_POWER`: lower it if the gearing or wiring complains.
- [ ] `Turret.WATCHDOG_MS`: only if a normal big move (D-pad 0 to 180) trips the watchdog, raise it.
- [ ] Watch the "Turret stuck check" telemetry line. TEST now and then is fine: the turret stopped short on friction and gets a harder push. REST while nothing holds the turret means it didn't move even when pushed hard: check the encoder wiring and the turret for rubbing.
- [ ] `Turret.ON_TARGET_DEG`: largest aim error that still scores. Find it with the shooter running (step 7).

## 6. Shooter (Shooter Test OpMode)

**Before the first spin:** the two motors are geared together. If one runs the wrong way they fight. Test each motor alone at low power first. In Shooter Test, with the wheel off, hold Square to run only the left motor, or Circle to run only the right one (`ShooterTest.MOTOR_TEST_POWER`, 0.2 by default). Each must push the shooting direction. If one doesn't, flip its `REVERSED` flag in Panels (works live).

- [ ] `Shooter.LEFT_REVERSED`, `Shooter.RIGHT_REVERSED`: both push the wheel in the shooting direction.
- [ ] `Shooter.ENCODER_REVERSED`: RPM reads positive while shooting.
- [ ] Check the encoder is on the **left** shooter motor's port. If it's on the right, change `encoderMotor` in `Shooter`.
- [ ] `Shooter.KV`: set KS and KP to 0. Raise KV until the wheel settles at the target at 2000, 3500 and 5000 RPM.
- [ ] `Shooter.KS`: raise it if low RPMs sit below target.
- [ ] `Shooter.KP`: raise it until it recovers quickly without oscillating.
- [ ] `Shooter.WATCHDOG_MS`: only if the watchdog trips on a normal spin-up, raise it (or lower `WATCHDOG_MIN_RPM`).
- [ ] `Shooter.BOOST_BELOW_RPM`: fire 4 balls (RB) and watch RPM through the burst. Recovery should be quick, with no big overshoot.
- [ ] `Shooter.RPM_TOLERANCE`: the largest RPM error that still scores at mid range. Not under 90: the 28-count encoder's reading moves in steps of about 43 RPM, so a tighter band flickers on and off (it was 75, now 90 for this reason).
- [ ] Coast-down: the shooter's motors float, so the wheel can't brake. In Shooter Test with the wheel on, press D-pad down once or twice (100 RPM each) and time how long the wheel takes to slow down 100 to 200 RPM. Driving toward the HIVE and a driver-2 trim down both lower the target, and `atSpeed()` holds off a burst until the wheel gets there. If that takes too long, tell the programmers to add braking or a small reverse power.

How a burst fires in Main TeleOp (`robot/FireControl`): fire is continuous. Once the shot has stayed ready for `READY_HOLD_MS`, the door opens, and from the moment it is fully open the rail runs at `ShotTables.SHOT_FEED_POWER` until RB is let go. So the fire rate comes from that feed power (step 7), not from stopping between balls, and the wheel slowing as balls go through is expected. Mid-burst the rail only pauses while the aim is off or the wheel is more than `BURST_RPM_FLOOR` under its target. The shot counter (an RPM dip) only reports shots: in TeleOp it never stops the rail.

- [ ] `FireControl.SHOT_DIP_RPM` (150): in Shooter Test, hold RB and watch RPM as a ball goes through. Set it a bit under the smallest dip you see. A shot counts when the RPM falls this far below the highest reading of the last `DIP_WINDOW_MS`, wherever the wheel was against its target. Keep it at least 86 (2 steps of the 43 RPM reading) plus what the wheel coasts down in `DIP_WINDOW_MS`. Counted shots show on the "Fire" line in Main TeleOp, and auto's `shoot()` ends on the first one (step 9).
- [ ] `FireControl.DIP_WINDOW_MS` (100): in Shooter Test, longer than a ball's dip takes to reach `SHOT_DIP_RPM`, and far shorter than the wheel takes to coast down `SHOT_DIP_RPM` with the power off (Cross turns the wheel off: do it at full speed and time it).
- [ ] The rest of `FireControl` is **tuned in Main TeleOp, not Shooter Test.** Shooter Test skips `FireControl`: RB just opens the door and runs the rail at `ShooterTest.FEED_POWER` once the door is open.
  - [ ] `FireControl.READY_HOLD_MS` (40): raise it if bursts start while the turret is still settling.
  - [ ] `FireControl.BURST_RPM_FLOOR` (600): hold RB with 4 balls and watch the lowest RPM during the burst. Set this well past that sag, so a normal burst never pauses; lower it if the last balls of a burst fall short. 0 turns the check off.
  - [ ] `FireControl.DOOR_CLOSE_DELAY_MS` (100): let go of RB in the middle of a burst. Raise it if the door closes on a ball; lower it if the door stays open longer than it needs to.
  - [ ] `FireControl.NO_SHOT_WARN_MS` (400): a bit longer than the slowest time from the rail starting to the first shot counted. When the rail runs this long with no shot counted, the "Fire" line adds "no shot seen ... jam?". It is only a note: nothing stops.

## 7. Shot tables (`robot/shot/ShotTables.java`)

All values there now are placeholders. Panels can change the numbers but can't add or remove rows; to add a row, edit the code. Keep the distances increasing from top to bottom (a warning shows if they aren't).

- [ ] `SHOT_RPM`: put the robot at measured distances (turret center to the center of the upward cell's opening, along the floor: `Field.RED_HIVE_X` and `Field.FAR_CELL_Y` / `Field.AUDIENCE_CELL_Y`) about every 10 in. from 20 to 110 in. At each one, find the RPM that scores reliably and write `{distance, rpm}`.
- [ ] Repeat a few distances with the **other** cell up to check one table works for both.
- [ ] `SHOT_FEED_POWER`: the rail's power for the whole burst, so it sets how fast the balls go out. Lower it at long range if back-to-back balls miss because RPM hasn't recovered.
- [ ] Check the robot can't reach a spot closer than the table's first row. If it can, add a row (in the code).

`SHOT_FLIGHT_TIME` only matters for shooting while moving (step 11).

## 8. Field and relocalizing (`field/`)

The field numbers (`Field.SIZE`, `Field.RED_HIVE_X`, `Field.FAR_CELL_Y`, `Field.AUDIENCE_CELL_Y`, `FieldZones`, `IntakePoses.GARDEN_LINE_LENGTH`) follow Pedro Visualizer's 141.5 in. field and the Competition Manual, and agree with the official field CAD, so they don't need measuring. Visualizer points copy into the code 1:1. `FieldZones.ROBOT_HEIGHT` is the robot's height from CAD: change it if the robot gets taller. Where each number came from is in `TELEOP_PLAN.md`, "Field coordinates".

TeleOp only shoots after a relocalize (or a fresh auto), so this step is part of the minimum.

- [ ] `Field.CENTER_TO_INTAKE_WALL`: push the intake flat against a wall and measure from the wall to the turret center (CAD says 7.99 in.). If the wedges or something else touches first, measure to that instead.
- [ ] `Field.CENTER_TO_SIDE_WALL`: the same for the robot's side (CAD says 5.87 in.). Check that nothing sticks out sideways past the chassis.
- [ ] Relocalize in all 4 corners (D-pad left/right, Square, Circle), then drive to a taped spot and check the pose.

**Phase 1 done:** play a full TeleOp match. Relocalize, intake, shoot standing still from near and far, unjam, use the flower intake.

# Phase 2: Autonomous and competition

## 9. First autonomous (`opmodes/auto/AutoTemplate.java`)

The goal is a simple auto that scores the 4 preloaded POLLEN and parks, with plain `RobotCommands` (no AgateFlow). Every pose in an auto has a `TODO(9)`. Copy `AutoTemplate` for each real auto, fill in the poses, and delete each TODO once its value is measured. A pose is three fields (`_X`, `_Y`, `_HEADING_DEG` in degrees) so you can try it live in Panels. The building blocks are in `robot/commands/RobotCommands.java`.

- [ ] Measure each start pose, with the robot sitting where it starts. Write it for red; the robot flips it for blue. Run it once on blue to check the flip.
- [ ] Pick the shooting spot and check its distance to the cell is inside the `ShotTables` range.
- [ ] Test the commands one at a time on the field, in this order:
  - [ ] `driveTo` a short distance.
  - [ ] `shoot()` with 1 ball.
  - [ ] `shootAll()`.
  - [ ] `shootContinuous()` standing still. Moving needs step 11 done.
- [ ] `AutoTemplate.DRIVE_TIMEOUT_MS` (4000 ms by default): time the drive to the shooting spot, then add margin. Pedro never gives up on a path by itself, so without a limit a blocked robot waits there for the rest of auto. Give every drive in a real auto a limit (`driveTo(pose, ms)`).
- [ ] `RobotCommands.SHOOT_ONE_TIMEOUT_MS`, `RobotCommands.SHOOT_ALL_TIMEOUT_MS`: time a normal 1-ball shot and a full burst, then add margin. Long enough that a good shot never times out, short enough that a misfeed doesn't eat the auto.
- [ ] `shoot()` holds shoot until one shot is counted, and `FireControl` stops the rail the moment it is (`Robot.stopAfterOneShot`). It relies on `FireControl.SHOT_DIP_RPM` from step 6: a shot that isn't counted keeps the rail feeding until `SHOOT_ONE_TIMEOUT_MS`, which can send more balls.
- [ ] `RobotCommands.LAST_BALL_MS` (300): `shootAll()` and `shootContinuous()` keep shoot held this long after storage reads empty, because the last ball has left the beams but not yet reached the wheel, and letting go stops the rail at once. Run `shootAll()` with 4 balls and watch the last ball. Raise it until the last ball always reaches the wheel before the rail stops; lower it to end bursts sooner.
- [ ] `RobotCommands.RETRY_UNJAM_MS`: jam a ball on purpose and run `shootAllWithRetry`. Raise it until the retry frees the jam, but balls must not come out of the intake. `RobotCommands.SHOOT_RETRIES`: each retry can cost `RETRY_UNJAM_MS` + `SHOOT_ALL_TIMEOUT_MS`, so set it to 0 if retries never fix anything.
- [ ] Replace the placeholder routine with the real one, and delete the `@Disabled` line.
- [ ] After auto, start Main TeleOp and check there's no "POSE UNKNOWN" warning.

## 10. Before competition

- [ ] Copy every value changed in Panels back into the code, `Robot.HOOD` included. At events the robot plays on what is in the code.
- [ ] `MainTeleOp.FULL_TELEMETRY` = false in the code: only warnings and loop time, every 200 ms, instead of everything every loop.
- [ ] Panels off for the whole event, pits included: R704.D names FTControl Panels as prohibited, and it doesn't say "during MATCH play" the way R704.C does. Before the event, set `PanelsSettings.EVENT_BUILD` = true in the code and install the app. Panels then stays off at every app start, and so does its Limelight plugin, which opens ports even while Panels is off. To check, INIT the `Enable/Disable Panels` OpMode on the Driver Station (TeleOp list, group "Panels"): its screen says whether Panels is on. Then press Stop, not Start, since Start switches Panels until the next app restart. Without the event build, that OpMode is the only way to turn Panels off, and **Panels turns itself back on whenever the robot app restarts** (power cycle, Restart Robot, new code). After the event, set `EVENT_BUILD` back to false to use Panels again. The rule text doesn't say whether Panels may stay in the app while it's off. If an inspector wants it out of the app, that needs a code change, so ask the programmers before the event.
- [ ] INIT check (G304.H, G403): INIT Main TeleOp, then the auto, and watch the robot. Nothing may move or twitch until Start. Do it again once `Constants.create()` is filled in, and after any change to init code.
- [ ] Gamepads: both wired to the Driver Hub by USB, none on Bluetooth (R901, R904).
- [ ] Practice: a full match with both drivers, including relocalizing and switching cells.
- [ ] Check the ROBOT.md TBDs are all filled in.

**Phase 2 done:** a full match, auto into TeleOp, on both alliances.

# Phase 3: Make it better

## 11. Shooting while moving (`robot/shot/ShotSolver.java`)

Only after stationary shots are reliable.

- [ ] With `ShotSolver.LEAD` and `ShotSolver.PREDICT` off: shooting while still must work everywhere.
- [ ] `Shooter.KA` (and `TARGET_RATE_FILTER`): in Main TeleOp, drive straight toward and away from the HIVE with a ball stored and watch Shooter RPM. Raise KA from 0 until the wheel keeps up with the target without overshooting it.
- [ ] `Shooter.MAX_TARGET_RATE` (8000 RPM/s): with KA on, drive full speed straight at the HIVE and watch "target changing" in telemetry. Set it a little above the biggest number you see.
- [ ] `Turret.KV`: in Main TeleOp, spin the robot in place. The turret should keep pointing at the cell. Raise KV until it stops lagging behind.
- [ ] `Turret.TARGET_RATE_FILTER`, `Turret.MAX_TARGET_RATE_DEG_PER_SEC`: while tuning KV, if the turret jerks when the robot starts or stops turning, raise the filter toward 0.8 (0 = no smoothing). Time a full-stick spin in place (degrees divided by seconds) and set the max a little above it, so a quick target step (like switching cells) can't send one huge KV push.
- [ ] `ShotTables.SHOT_FLIGHT_TIME`: slow-motion video of shots at near, mid and far distances. Count frames from leaving the wheel to entering the cell.
- [ ] Turn `LEAD` on. Drive slowly past the HIVE at a steady speed and fire. A ball that lands ahead of the cell along the driving direction means the flight times are too short; one that lands behind means they are too long.
- [ ] Turn `PREDICT` on. It solves from where the robot is about to be, not where it is: the turret aims from `TURRET_LATENCY_SEC` ahead, the distance, RPM and lead come from `RELEASE_LATENCY_SEC` ahead (when the ball leaves), and Pedro's velocity, which runs `VELOCITY_LATENCY_SEC` behind, is moved forward using the acceleration. Tune them in this order:
  - [ ] `ShotSolver.RELEASE_LATENCY_SEC`: about one loop ("Loop" in telemetry, usually 0.02 to 0.04 s). It is **not** the time the rail takes to feed a ball: the turret, hood and RPM keep re-aiming every loop while the ball travels, so the last aim before it leaves is only about a loop old. (GNCE Ditto could use their feed time because their ball sat against the wheel.)
  - [ ] `ShotSolver.TURRET_LATENCY_SEC`, how long the turret takes to get to a new angle: drive past the HIVE at a steady speed, then spin in place. If the turret lags behind the cell, raise it; if it aims ahead, lower it.
  - [ ] `ShotSolver.VELOCITY_LATENCY_SEC`, how far Pedro's velocity lags: drive past the HIVE, brake hard to a stop and fire right away. A ball that lands behind the cell (seen along the way the robot was driving) means the robot still thinks it's moving: raise it. One that lands ahead: lower it. At a steady speed it changes nothing, so only brakes show it. If raising it stops helping on the hardest brakes, raise `ShotSolver.MAX_ACCEL` toward `DriveModel.TRACTION_ACCEL`.
  - [ ] `ShotSolver.MAX_ANGULAR_ACCEL` (20 rad/s²), the same cap for turning: only if PREDICT's turret aim lags when the robot starts or stops spinning hard, raise it. If the aim twitches while turning steadily, lower it.
  - [ ] `ShotSolver.USE_ACCEL`, `ShotSolver.ACCEL_FILTER`: the acceleration estimate behind the velocity correction and the prediction. Only if the turret twitches while driving at a steady speed, raise `ACCEL_FILTER` toward 0.9 (0 = no smoothing; higher sees a brake later). If that doesn't settle it, set `USE_ACCEL` to false: PREDICT then assumes a constant speed, and `VELOCITY_LATENCY_SEC` does nothing.
- [ ] Try shooting while braking and while turning. Write down what misses for the next round of code.

## 12. Driver feel, feedback and loop time

- [ ] `Drive.DRIVE_CUBIC`, `Drive.TURN_CUBIC`: the stick curve, to the drivers' taste. Higher = finer control near the center, same top speed.
- [ ] `Field.CELL_SWITCH_BAND`: drive across the middle of the field. The aim should switch cells once, not flicker.
- [ ] `MainTeleOp.TURRET_TRIM_STEP_DEG` (2°), `MainTeleOp.RPM_TRIM_STEP` (25): driver 2 practice. One press should fix a typical miss. Raise a step if it takes several presses, lower it if one press overshoots.
- [ ] `MainTeleOp.TRIGGER_THRESHOLD` (0.5): raise it if a resting finger fires RT or LT; lower it if a full pull doesn't register.
- [ ] `Feedback.WARNING_*`: the warning pulses. Long enough to feel, short enough not to annoy. `WARNING_COOLDOWN_MS` stops a flickering warning from buzzing nonstop.
- [ ] `Feedback.FULL_STRENGTH`: the constant storage-full rumble. Soft enough to drive with, strong enough to notice.
- [ ] The gamepad light only works on PlayStation-style controllers. Check yours.
- [ ] Watch "Loop" in telemetry with everything running. Goal: under 15 ms. Then set `MainTeleOp.FULL_TELEMETRY` to false and check again (it then sends every `TELEMETRY_INTERVAL_MS`, 200 ms).

# Phase 4: AgateFlow and auto pickups

AgateFlow (`agateflow/`) is the path planner. No match OpMode uses it yet (only the AgateFlow Benchmark test OpMode does), so none of this matters until an auto does.

## 13. AgateFlow setup

- [ ] After ForesightTuner, register `DriveModelTuner` in `pedro/TeamTuning.java` (the code is in a comment there), run it, and paste its output into `agateflow/DriveModel.java`. AgateFlow reads Foresight's gains from the follower itself. `DriveModel.TUNED_VOLTS` was written down in step 2.
- [ ] `DriveModel.TRACTION_ACCEL`: drive full power forward from a stop on tiles. If the wheels spin, lower it.
- [ ] `AgateFlow.ROBOT_RADIUS`: the farthest point of the robot from the turret center (a circle that covers the whole robot). `AgateFlow.CONTACT_RADIUS`: the nearest one (about 5.7 in. at the back).
- [ ] AgateFlow's outline: `AgateFlow.ROBOT_FRONT` and `AgateFlow.ROBOT_HALF_WIDTH` take their values from `Field.CENTER_TO_INTAKE_WALL` and `Field.CENTER_TO_SIDE_WALL` (step 8) when the app starts, so once step 8's numbers are in the code they match. A Panels edit to the `Field` values doesn't reach them until the next app start, so check they match. `AgateFlow.ROBOT_BACK`: tape from the turret center to the chassis back (CAD says 5.72 in.).
- [ ] `DriveModel.SETTLE_SECONDS` (0.10): drive a few short paths and time from the robot reaching the end to the follower saying it is done. Set it to that.
- [ ] Run the **AgateFlow Benchmark** OpMode (the robot doesn't move; without a follower AgateFlow can't add speed limits, so the FLOWER and GARDEN routes find no way). Write down the median and 90th percentile plan times. Plans that hit `AgateFlow.PLAN_TIME_LIMIT_MS` stop trying more routes. Then open Panels' field page (Pedro Pathing preset): D-pad left/right shows each plan, D-pad up switches alliance. Zones are magenta, the planned path blue, the predicted path green, the robot's outline orange with its intake end red. The zones must sit on the picture's HIVE frame ends and FLOWERS.
- [ ] `AgateFlow.PLAN_TIME_LIMIT_MS` (100): from the Benchmark numbers. Raise it if many plans hit it and come out slower than on the desktop; lower it if replans come back too late while driving.

## 14. Pickups: FLOWERS, GARDEN and floor POLLEN

- [ ] `IntakePoses.FLOWER_INTAKE_REACH`: with the flower intake down, measure from the turret center to where it touches the tiles (ROBOT.md says 275.3 mm, 10.84 in.). AgateFlow plans every `Route.flowerIntakeDown()` stretch with the robot reaching this far ahead.
- [ ] `IntakePoses.FLOWER_REACH_INTO_OPENING`, `IntakePoses.FLOWER_APPROACH`: drive into a real FLOWER with the flower intake down. It must reach the bottom POLLEN without the robot's front hitting the FLOWER.
- [ ] `RobotCommands.FLOWER_RAISE_MS`, `AgateFlow.FLOWER_INTAKE_RAISE_RUN`: film the flower intake coming all the way up and time it. `flowerUp()` waits `FLOWER_RAISE_MS` before the next command, so set it a little longer. Set `FLOWER_INTAKE_RAISE_RUN` to how far the robot backs away from a FLOWER in that time: after a `flowerIntakeDown()` stretch, AgateFlow plans that far with the intake still reaching out.
- [ ] Flower intake with AgateFlow: drive to a FLOWER with a `Route` that ends in `flowerIntakeDown()`, through `RobotCommands.withFlowerIntake(...)`. The intake must go down only for that stretch, stay down at the FLOWER, and the robot's front must not hit the FLOWER.
- [ ] `IntakePoses.GARDEN_WALL_GAP`, `IntakePoses.GARDEN_RUN_UP`, `IntakePoses.GARDEN_END_GAP`: drive along the GARDEN line toward the corner. The intake must pick up all 4 POLLEN without the robot rubbing the wall.
- [ ] `AgateFlow.PICKUP_SPEED`: `Route.pickUp` drives over each POLLEN at most this fast (in/s). Drive into a POLLEN at rising speeds and set it to the fastest that still picks it up every time.
- [ ] `AgateFlow.INTAKE_DEPTH`: drive slowly over a POLLEN and measure how far past the intake's front edge it is when the intake grabs it.
- [ ] `AgateFlow.INTAKE_HALF_WIDTH` (5.51 in.): measure the intake's opening across, where a POLLEN gets grabbed, and set this to half of it.
- [ ] `AgateFlow.PICKUP_RUN` (12 in.), the straight run into each pickup: pick up POLLEN that sits at an angle to the robot's way in. Raise it if the intake hits them crooked and misses; lower it if pickUp routes are slow.
- [ ] `AgateFlow.PICKUP_EXIT` (4 in.), how far past each pickup its speed limit stays on: raise it if the robot speeds up before the POLLEN is all the way in; lower it if it stays slow for long after.
- [ ] `AgateFlow.PICKUP_SIDE_MARGIN` (1 in.): raise it if POLLEN gets missed at the intake's edges; lower it if `Plan.skipped()` leaves out ones the intake could get.

## 15. AgateFlow on the field

Once an auto uses `AgateFlowCommands`:

- [ ] Watch the robot pass the zones. If it brushes them, raise `AgateFlow.SAFETY_MARGIN`; if paths go needlessly wide, lower it. A plan with a "closer ... than SAFETY_MARGIN asks" note is a near miss AgateFlow took anyway (`AgateFlow.NEAR_MISS_FRACTION`); if the robot brushes a zone on one of those, set it to 0 so AgateFlow refuses them instead.
- [ ] `AgateFlow.WALL_GAP`, `AgateFlow.WALL_CONTACT_SPEED`: AgateFlow lets any part of the robot touch the field walls, but plans it never to move into a wall faster than `WALL_CONTACT_SPEED` (in/s). Drive a route along a wall. Raise `WALL_GAP` if rubbing the wall slows the robot down or drags its heading round. Lower `WALL_CONTACT_SPEED` if the odometry pods skip or the robot jolts when it reaches a wall; raise it if wall runs are slow.
- [ ] `AgateFlowCommands.TIMEOUT_SCALE`, `AgateFlowCommands.TIMEOUT_EXTRA_SECONDS`: a drive gives up after `TIMEOUT_SCALE` x its estimate + `TIMEOUT_EXTRA_SECONDS`. Watch a few autos. Raise them if a drive times out close to its target, lower them if a stuck robot waits too long.
- [ ] `AgateFlowCommands.REPLAN_EXTRA_SECONDS`, `AgateFlowCommands.REPLAN_OFF_PATH` (8 in.): push the robot off its path by hand while it drives. It should replan once it is `REPLAN_OFF_PATH` off its predicted path, and not from normal wobble, and the drive must still end.
- [ ] `AgateFlowCommands.MAX_DETOURS` (3): in a test OpMode, drop a vision zone in the robot's way a few times in one drive. It should go round each one and still end. Each detour gets its own plan's time, at most this many times per drive.
- [ ] `AgateFlowCommands.MAX_OVERTIME_SECONDS` (3): watch drives that run past their deadline. Raise it if one still closing in on its stop gives up; lower it if a crawling robot holds up the auto.
- [ ] `AgateFlowCommands.STOP_BRAKING_POWER`: the reverse power a drive brakes with when a new zone shows up within 30 in. ahead. In a test OpMode, add a vision zone 15 in. in front of the robot while it drives at full speed. It should stop short and straight, and the battery reading shouldn't drop under 9 V. Lower it if the wheels skid or the hub browns out.
- [ ] A route that gives "no plan" in a tight spot: AgateFlow never changes a heading the route asks for, and a fixed or linear heading crosses a gap at an angle, so the robot is wider across it. Try `Heading.tangent()`, a waypoint without a heading, or a through-waypoint just before the gap. The plan's notes also say when a waypoint was moved so the robot fits, or a pickup was left out.
- [ ] `ParkGuard.MARGIN_SECONDS` (once an auto uses it): run an auto that runs long. It must end parked with a little time to spare. Raise it if parking is late, lower it if it parks too early.
- [ ] `ParkGuard.REFRESH_MS` (250): run an auto that replans a lot. Raise it if replans come back late (they share the planning thread with park estimates); lower it if parking starts much earlier than needed.
- [ ] `ParkGuard.FALLBACK_SECONDS` (5): until a park estimate has worked, it parks this many seconds before the end. Block the park route with a vision zone in a test auto and check the robot still parks in time once it is clear.
- [ ] `EtaCalibration.TIME_SCALE` learns real/predicted drive time from each leg driven start to finish on its own plan that reached its stop (not the first leg of a replan, a leg the robot gave up settling on, or a `driveTo(estimate)` leg it joined partway). `EtaCalibration.LEARN_FILTER` (0.8) smooths it: each leg moves it a fifth of the way. After a few autos, copy `TIME_SCALE` from Panels into the code.

# Phase 5: Coax hood

Only once the coax hood is on the robot. Until then `Robot.HOOD` stays `STATIC` and nothing here matters. Steps 6 and 7 still hold either way: with `STATIC` picked, a coax hood on the robot holds `CoaxHood.STATIC_LAUNCH_DEG`, so the static `SHOT_RPM` table keeps working. Shooter Test holds it there too, so rows measured there in step 7 fit. `Robot.HOOD` switches live in Panels (page "Robot"), so the two can be compared shot for shot.

How it works: the wheel's target RPM comes from `ShotTables.COAX_RPM`, and every loop `HoodSolver` works out the launch angle that scores at the RPM the wheel **really** has. A wheel that dipped after a ball, or is still spinning up, still scores, because the hood moves much faster than the wheel can change speed. Angles are launch angles above horizontal: 0 is flat, 90 is straight up.

## 16. Coax hood (Coax Hood Test OpMode)

Setup:

- [ ] Add a servo named `hood` to the robot configuration (`HardwareNames.HOOD_SERVO`) and write its port in ROBOT.md. Without it, `Robot.HOOD = COAX` only gives the `HOOD_MISSING` warning.
- [ ] `CoaxHood.GEAR_RATIO`, `CoaxHood.SERVO_RANGE_DEG`: from the gearing (servo degrees per hood degree) and the servo's spec (how far it turns from position 0 to 1).
- [ ] A goBILDA or Axon hood servo goes in the configuration as "Full Range Servo" (500 to 2500 us), not plain "Servo" (600 to 2400 us). `SERVO_RANGE_DEG` depends on that choice: an Axon MINI MK2 turns 350° on Full Range, about 315° on plain Servo.

Angle mode (Coax Hood Test; Cross switches modes; D-pad up/down ±1°, bumpers ±5°; Circle switches raw servo on and off, where D-pad up/down moves the servo ±0.005 and bumpers ±0.05). Measure the real launch angle with a phone level or protractor along the hood's surface where the ball leaves. The servo mapping is pinned at one measured point (`ANCHOR_DEG`, `SERVO_AT_ANCHOR`), so `MIN_LAUNCH_DEG` and `MAX_LAUNCH_DEG` are only limits: changing them never moves any angle. In this order:

- [ ] `CoaxHood.ANCHOR_DEG`, `CoaxHood.SERVO_AT_ANCHOR`: in raw servo, nudge the hood until it just touches its flat stop. Measure the launch angle there: that's `ANCHOR_DEG`, and the servo position on the screen is `SERVO_AT_ANCHOR`. A level against the stop is easy to measure, but any measured angle works.
- [ ] `CoaxHood.REVERSED`: Circle back to angles. D-pad up must make the hood steeper. If it gets flatter (or pushes into the flat stop), flip it.
- [ ] From here on, watch for "Hood servo at its end" in the hood telemetry (the `HOOD_AT_END` warning in Main TeleOp): the mapping asks for a servo position past 0 to 1, so the hood stops short of the angle. Recheck `ANCHOR_DEG`, `SERVO_AT_ANCHOR`, `GEAR_RATIO` and `REVERSED`. "HOOD SERVO NOT SENT" means a hood value in Panels isn't usable (like `SERVO_RANGE_DEG` under 1).
- [ ] `CoaxHood.GEAR_RATIO`, `CoaxHood.SERVO_RANGE_DEG`: command a second, steep angle short of the steep stop and measure it. Steeper than commanded: raise `SERVO_RANGE_DEG`; flatter: lower it (or the gear ratio is off).
- [ ] `CoaxHood.MIN_LAUNCH_DEG`, `CoaxHood.MAX_LAUNCH_DEG`: a degree or two inside the hard stops, so the servo never pushes against them. The flat stop is `ANCHOR_DEG` if the anchor is there; measure the steep stop in raw servo the same way.
- [ ] `CoaxHood.SPEED_DEG_PER_SEC`, after MIN and MAX: Triangle jumps between them. Film it and divide the angle by the time; round down. There's no position feedback, so the code uses this to guess when the hood has arrived (`atTarget()`), and a shot waits for it.
- [ ] `CoaxHood.STATIC_LAUNCH_DEG`: the static hood's launch angle. Measure it before the static hood comes off.

Physics mode (D-pad left/right changes the test distance by 5 in., D-pad up/down adds ±100 RPM on top of the table). Hold RB to fire: the door opens and the rail feeds once the hood has an angle for this RPM and is there, and while RB stays held the rail pauses whenever the hood isn't ready, like a burst in Main TeleOp. So every ball leaves at the angle the physics picked.

- [ ] `HoodSolver.LAUNCH_HEIGHT`: height of the ball's center above the tiles at the moment it leaves the wheel.
- [ ] `ShotTables.COAX_EXIT_SPEED`: put the robot at a taped distance, set the same distance in the OpMode, and fire at two or three RPMs (D-pad up/down). At the same angle a faster ball arrives higher, so if balls land **high** at an RPM, the real ball is faster than the table says: raise the value at that RPM. If they land **low**, lower it. Slow-motion video works too: horizontal speed / cos(angle) / RPM.
- [ ] `ShotTables.COAX_EXIT_SPEED_ANGLE`: once `COAX_EXIT_SPEED` is right at one hood angle, fire at the same RPM with the hood flat and steep (angle mode, Square spins the wheel). The ball wraps the wheel for a different arc at each angle, so it can leave faster or slower. Lands high at an angle: raise that angle's multiplier.
- [ ] `ShotTables.COAX_RPM`: redo it once the physics numbers above are measured. Coax Hood Test's init screen shows a seed every 10 in. from 20 to 110: the slowest RPM where the hood can still score with the wheel `HoodSolver.SEED_RPM_WINDOW` (300) RPM off either way, so the hood always has room to make up for a dip or an overshoot, and a slower wheel recovers faster between balls. Where no RPM has that much room, it is the middle of the RPMs that score. The defaults come from it. `COAX_RPM`'s rows are every 15 in. (20, 35, 50, 65, 80, 95, 110), so only the 20, 50, 80 and 110 in. seeds match a row. For the other three, change those rows' distances to 30, 60 and 90 in Panels and copy those seeds, or edit the table in the code to use every seed (Panels can't add rows). On the field, raise the RPM at a distance if the hood often shows "no angle scores" there while the wheel dips.
- [ ] `HoodSolver.SEED_RPM_WINDOW`: keep it equal to `Robot.COAX_RPM_TOLERANCE` (both 300 now), so the seeds leave the hood room for exactly the RPM error a burst may start with.
- [ ] `HoodSolver.TARGET_BAND`: how far above or below the opening's center the ball may cross. Smaller makes the solver pickier, larger lets it shoot in more spots. Keep it above about 2 in. while `RPM_MARGIN` is 50: the margin moves where the ball crosses by 1 to 2 in., so a band of 1 or less stops nearly every coax shot. Lower the two together. `HoodSolver.LIP_HEIGHT`, `HoodSolver.LIP_BEFORE`: only if shots clip the opening's bottom edge.
- [ ] `HoodSolver.RPM_MARGIN`: the hood only takes an angle that still scores this many RPM either side of the reading, so encoder noise and the average running a loop late can't spoil a shot. Raise it if shots fired while the wheel recovers miss; lower it if the hood refuses shots it would make.
- [ ] `Shooter.AVERAGE_SAMPLES`: only if the hood shakes with the wheel's speed reading, raise it (and `RPM_MARGIN` with it: the average runs about half that many loops late).

In Main TeleOp, with `Robot.HOOD = COAX`:

- [ ] `Robot.HOOD`: set it to `COAX` in Panels (page "Robot"), and in the code once the hood works. It switches only between bursts.
- [ ] Shoot from all over the table's range. "Hood" in telemetry shows the angle, or "no angle scores" while the wheel is too slow.
- [ ] `Robot.COAX_RPM_TOLERANCE` (300): how far the wheel may be from its target for a coax burst to start (mid-burst, `FireControl.BURST_RPM_FLOOR` applies, as with the static hood). Hold RB with 4 balls. Raise it for faster bursts (the hood makes up for the RPM error); lower it if balls fired with the wheel far off target miss. Keep `HoodSolver.SEED_RPM_WINDOW` equal to it.
- [ ] `CoaxHood.ON_ANGLE_DEG` (0.5°): raise it if bursts wait on "moving" while the hood angle barely changes; lower it if shots fire before the hood has settled.
- [ ] `FireControl.SHOT_DIP_RPM`: check it again in Coax Hood Test physics mode, with the wheel anywhere in the 300 RPM ready window (D-pad up/down moves the RPM). A coax burst can start with the wheel still speeding up or above its target. The dip is measured the same way for both hoods, from the highest reading of the last `DIP_WINDOW_MS`.
- [ ] `MainTeleOp.HOOD_TRIM_STEP_DEG` (0.5°): driver 2's D-pad up/down trims the hood instead of the RPM in COAX. Up is steeper, which mostly lands higher. One press should fix a typical high or low miss: raise it if it takes several presses, lower it if one overshoots.
- [ ] Shooting while moving (step 11) in COAX: `LEAD` takes the flight time from the physics, so `SHOT_FLIGHT_TIME` isn't needed.
- [ ] `CoaxHood.IDLE_LAUNCH_DEG`: where it waits with nothing to aim at. Set it near the angle most shots use, so the hood has the least to move when a shot comes up.
- [ ] Passing: turn on `FireControl.PASS_ENABLED`. Pick where passes should land in our LOADING ZONE and set `Field.PASS_TARGET_RED_X/Y` (now a placeholder at (24, 106)). With the coax hood, a pass is a lob at about `CoaxHood.PASS_LAUNCH_DEG`: the RPM is what lands it on the pass target at that angle (`HoodSolver.passRpm`), and the hood solves for the real RPM like a shot. Higher lobs clear robots, lower ones get there sooner. Passes stay inside `ShotTables.PASS_RPM`'s distance range, so none can leave the field (G405): check from the far end of that range, and fill `PASS_RPM` from real passes like `SHOT_RPM`. `ShotTables.PASS_FEED_POWER`: lower it if back-to-back passes land short or scatter.
