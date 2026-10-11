# Software innovations

What is new or unusual in our robot code for BIOBUZZ (2026-27), how each part works, and where to find it. Paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Names in capitals are Panels tunables. The numbers given for them are the current defaults, and most are still guesses until the robot is tuned.

**Status (2026-10-10, evening).** None of this has run on the robot yet. Pedro isn't tuned (`Constants.create` still returns null), so no match OpMode can start, and the coax hood isn't built. 181 JUnit tests in 27 classes run on a laptop, and all passed in a Gradle run on 2026-10-10: 74 for AgateFlow, the path planner (against a simulated robot; no match OpMode uses it yet), and 107 for fire control, the shooting math, the turret controller, the write caches, the field frame and the small helpers (section 7). The Pedro Quickstart's latest fix (the Pinpoint tuner reads the pod directions before Stop) is merged. The rest gets checked on the robot with the test OpModes, following `TUNING_GUIDE.md`.

1. [How the code is built](#1-how-the-code-is-built)
2. [Shooting](#2-shooting)
3. [Storage and intake](#3-storage-and-intake)
4. [Safety](#4-safety)
5. [Driving and driver feedback](#5-driving-and-driver-feedback)
6. [AgateFlow, our path planner](#6-agateflow-our-path-planner)
7. [How the code is tested](#7-how-the-code-is-tested)

## 1. How the code is built

### Auto and TeleOp share one brain

`robot/Robot.update()` makes every decision once per loop: what the rail, door, shooter, turret and hood do, and when the rail feeds balls into the shooter. OpModes only say what the drivers want. TeleOp sets `shootHeld`, `passHeld` and `unjamHeld` from the buttons. Autonomous commands (`robot/commands/RobotCommands`) set the same fields and wait on what `Robot` reports, like `shotsFired()` or `storage.isEmpty()`. Aiming, firing and safety rules exist in one copy, so a fix found in TeleOp practice fixes auto too.

`Retry` reruns an auto command while a check fails. `shootAllWithRetry` uses it: if balls are still stored after a burst times out, it reverses the rail for 200 ms and fires again. An auto with no alliance picked does nothing, because shooting at the wrong goal is worse than sitting still.

### Read, decide, write

Each loop reads every sensor first, decides, then writes every output at the end. So the decision to fire uses this loop's turret angle and this loop's flywheel speed, never one of each. Shoot versus pass is decided once at the top, so aiming and fire control can't disagree about which target they serve.

The loop stays fast: one bulk read per hub, motor and servo writes only when the value changes (`robot/hardware/Cached*`), battery voltage re-read every 250 ms, and no motor current reads. A motor power of 0 always goes out, however small the change. The shooter's motors use a finer write step (0.002 instead of 0.01), since 0.01 power is about 60 RPM of feedforward and small Panels edits would look dead. Each cache's `raw()` hands out the device itself for anything the cache doesn't wrap, and clears the cache so the next write always goes out: a call through it (a new direction or pulse range, or an encoder reset that zeroes the power) can change what the last write did. With `MainTeleOp.FULL_TELEMETRY` off, telemetry drops to the warnings and loop time every 200 ms. The goal is under 15 ms per loop.

### Everything tunable live

Every tunable is a non-final `public static` field in a `@Configurable` class, read where it's used and never only in a constructor, so a Panels edit applies on the next loop. That includes the shot tables, direction flags and auto poses. Direction flags flip the sign when power is written instead of calling `setDirection`, which would also flip the shooter's encoder reading. If a shot table edit puts its distances out of order, the `SHOT_TABLE_ORDER` warning shows instead of silently wrong RPMs. Final values are copied back into the code, since Panels edits are lost when the app restarts.

### Panels off at events

R704.D in the Competition Manual names FTControl Panels as a prohibited tool, with no "during a match" limit. Panels starts with the robot app, and its own off switch on the Driver Station only lasts until the app restarts. `util/PanelsSettings` is our build switch: with `EVENT_BUILD` (a build setting, not a Panels tunable) set to true before installing for an event, Panels stays off at every start. So does its Limelight plugin, which otherwise opens four network ports at every start, even while Panels is off. No match OpMode needs Panels for a value or for driver info: warnings and loop time go to the Driver Station. The event build hasn't been tried on the robot yet; it is on the step 10 checklist.

### Tuning checklist built from the code

Anything that still needs measuring has a `// TODO(step): what to do` comment right above it, where the step is the `TUNING_GUIDE.md` section. The steps are numbered in order of importance and grouped into five phases: the bare minimum for TeleOp, a first auto and getting ready for competition, making both better, AgateFlow and auto pickups, then the coax hood once it is on the robot. The `todoList` Gradle task runs before every build and rewrites `TODO_LIST.md` in that order, with a count per phase, a link to each line and the field it sits on. A TODO written any other way, or with a step that doesn't exist, gets a build warning, so nothing drops off the list by accident.

## 2. Shooting

### Shoot-on-the-move in layers

`robot/shot/ShotSolver` turns the robot's pose and velocity into a turret angle, flywheel RPM and feed power. It has three layers, each with its own switch:

1. Static: aim at the cell, RPM and feed power from distance tables. Always on.
2. `LEAD`: a ball leaves with the robot's velocity, so aim at `target - velocity × flight time`. Moving the aim point changes the distance and so the flight time, so this repeats 3 times.
3. `PREDICT`: solve from where the robot will be, with two clocks (from GNCE Ditto's final [`ShotPlanner.plan`](https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/core/util/ShotPlanner.java)). Distance, RPM, hood and lead use the robot at the moment the ball leaves (`RELEASE_LATENCY_SEC`, 0.03 s). The turret aims from further ahead (`TURRET_LATENCY_SEC`, 0.08 s), since it needs that long to get there, so it is right when the ball leaves.

`LEAD` and `PREDICT` start off and get turned on one at a time after stationary shots work. On the field, a bad layer can be switched off without touching code.

The HIVE tips during the match, so the robot aims at the cell on its own half of the field. Within 6 in. of the center line it keeps the cell it has, so the turret doesn't flip back and forth while driving along the middle. Passing gets no lead or prediction, and with either hood it is refused outside the `PASS_RPM` table's distances, since a ball sent out of the field is a MAJOR FOUL (G405). It stays off until the coax hood is on.

### Predicting the robot's motion

`robot/shot/MotionPredictor` does `PREDICT`'s predicting, Ditto's way (`plan`, `predictWithStop` and `compensateVel` in the same ShotPlanner). It runs each axis (x, y and heading) forward from the speed and acceleration:

- Pedro's velocity is a smoothed difference that runs about 0.06 s late (`VELOCITY_LATENCY_SEC`), so it is moved forward by the acceleration first.
- The acceleration is the change since the last loop only, capped at 200 in/s² (`MAX_ACCEL`) and smoothed (`ACCEL_FILTER`, 0.7). An average over many loops still shows a brake that has already ended, and the shot over-leads.
- A braking axis stops at 0 and stays there. The prediction never runs on into reverse.

Two things are ours, not Ditto's: an axis the lag correction finds already stopped stays put, and an axis's acceleration starts over from 0 when its speed reaches or crosses 0. Without them, a robot that just stopped would look like it's backing up for a few loops while the smoothed braking acceleration fades. In a desktop test of a robot braking to a stop, the reading still said 8 in/s after the robot had stopped: without `PREDICT` that is a 4 in. lead it doesn't need, with `PREDICT` it is none. The acceleration estimate runs even with `PREDICT` off, so it is ready the moment `PREDICT` is turned on. All of these timings get tuned in step 11.

### Continuous fire, with a shot counter that only watches

`robot/FireControl` runs a burst in three steps:

1. Wait. A burst needs 2 balls in TeleOp (auto may allow 1). The turret must be on target and the flywheel at speed (with the coax hood: an angle that scores, the hood there, and the wheel within 300 RPM), steady for 40 ms (`READY_HOLD_MS`), so one lucky reading can't start it. Then the door opens, and the rail starts once the door has had time to open fully.
2. Fire. The rail runs for as long as the button is held, at a feed power looked up by distance (`ShotTables.SHOT_FEED_POWER`).
3. Close. Letting go stops the rail at once. The door stays open 100 ms longer (`DOOR_CLOSE_DELAY_MS`), so it never closes on a ball that is already past it, and then the burst is over. After that the rail waits until the door is fully closed, so the next ball can't reach a half-open door.

Why it works this way:

- The fire rate comes from the transfer power, not from stopping between balls. The first version fed one ball, waited for the wheel to dip, paused 60 ms and waited for the wheel to be ready again, so every ball paid for the wheel's recovery. Now the rail doesn't stop between balls, and how far apart they go is set by one number per distance that we tune with real shots (step 7).
- The wheel slowing as balls go through is expected. Each ball takes some speed, and boost (full power whenever the wheel is more than 50 RPM under target) pulls it back up. The ready check only starts a burst; it isn't asked again.
- The rail pauses for only two reasons, and the door stays open while it does: the turret (or coax hood) is off target, or the wheel has sagged more than 600 RPM under its target (`BURST_RPM_FLOOR`), far more than a normal burst should. It goes on by itself as soon as both are back.
- Switching between shoot and pass mid-burst closes the burst the same way, since the wheel needs a different speed. The new burst starts like any other.

The flywheel still tells us when a ball went through. A ball hitting the wheel takes it down fast, in tens of ms, so a shot counts when the RPM falls 150 below the highest reading of the last 100 ms (`SHOT_DIP_RPM`, `DIP_WINDOW_MS`). A wheel coasting down to a lower target takes far longer than 100 ms to lose that much, and a wheel still speeding up has its highest reading right now, so neither counts. The rule is the same for both hoods and wherever the wheel is against its target. The old rule measured the dip from the target latched when the feed started, so a wheel that started above its target had to drop more than 150 RPM to count; the 2026-10-10 review found that. After a shot, the next one only counts once the wheel has come back up half a dip, so one deep dip counts once. Two balls that hit before the wheel comes back up count as one, so `shootAll()` ends when storage reads empty, not on the shot count.

The counter only watches. It feeds telemetry and `Robot.shotsFired()`, and in TeleOp it never moves the rail, so a wrong `SHOT_DIP_RPM` can't stop the drivers from shooting. If the rail runs 400 ms with no shot counted (`NO_SHOT_WARN_MS`), the Fire line on the screen adds a note (a jam, or a dip setting that's too big), and that is all it does. The count matters in one place: auto's one-ball `shoot()` sets `stopAfterOneShot`, and the burst closes by itself the moment the first shot is counted. `shootAll()` keeps the button held 300 ms after storage reads empty (`LAST_BALL_MS`), because the last ball has left the beams but not yet reached the wheel, and letting go would stop the rail with that ball still in it. `FireControlTest` checks all of this on its own clock (section 7).

### Flywheel at speed before the trigger

`robot/subsystems/Shooter` uses feedforward plus P control, scaled for battery voltage (capped at 1.5×, so a bad 0 V reading can't run away). During a burst, boost switches to full power whenever the wheel falls more than 50 RPM below target, so it recovers between balls as fast as the motors can. The wheel spins up whenever a ball is stored and stays up through the whole burst, so it is ready when the trigger is pulled and isn't cut while the last ball goes through.

### Flywheel anticipation

When the robot drives toward or away from the HIVE, the distance changes, so the flywheel's target RPM keeps moving, and a wheel that only reacts to its error is always a little behind. `Shooter.KA` adds a push proportional to how fast the target is changing, so the wheel keeps up. It is Ditto's `FLYWHEEL_ACCEL_KFF` (in their [`Shooter`](https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystems/Shooter.java)). The target's rate of change is smoothed (`TARGET_RATE_FILTER`, 0.7) and capped at 8000 RPM per second (`MAX_TARGET_RATE`). A change of 200 RPM or more in one loop (spinning up from 0, switching from shot to pass) and a driver's trim press count as no change, since neither is the robot driving. The push is zero while the target holds still, so it never biases a shot. It is off (`KA` is 0) until it is tuned in step 11.

### A coax hood aimed by physics from the wheel's real speed

The shooter takes two hoods, and `Robot.HOOD` picks one live in Panels. `STATIC` is the fixed launch angle and the `SHOT_RPM` table. `COAX` is a hood that pivots around the flywheel axle (`robot/subsystems/CoaxHood`), so turning the hood 1° turns the launch direction 1°. With the coax hood on and `STATIC` picked, it holds the static hood's angle (`STATIC_LAUNCH_DEG`), so the two can be swapped mid-practice and compared shot for shot. The coax hood isn't built yet: its gearing and every number in `CoaxHood` are placeholders for step 16.

The idea comes from GNCE Ditto ([`ShotPlanner`](https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/core/util/ShotPlanner.java), `solveHoodDeg` and `plan`). A flywheel takes a long time to change speed, but a servo tilts a hood in a fraction of a second. So the wheel's target RPM still comes from a distance table (`COAX_RPM`), but every loop `robot/shot/HoodSolver` works out the launch angle from projectile physics using the RPM the wheel **really** has, averaged over the last 3 loops (`Shooter.AVERAGE_SAMPLES`) so the servo doesn't shake. A wheel that dipped after a ball, or is still spinning up, still scores: the hood makes up the difference. Near the slowest RPM that scores, the right angle changes fast with the RPM, so it only takes an angle that would still score with the wheel 50 RPM slower or faster (`RPM_MARGIN`). A coax burst starts when that angle exists, the hood is there, the turret is on target and the wheel is within 300 RPM of target (`Robot.COAX_RPM_TOLERANCE`) instead of the static hood's 90 (`Shooter.RPM_TOLERANCE`), so it starts sooner. During the burst the hood keeps solving for the wheel's sagging speed, and the rail only pauses while no angle scores or the hood hasn't reached it yet.

How the solver picks an angle:

1. A quick check first. If the ball can't reach the bottom of the target band at any angle, even at the slowest RPM it must score at (the reading minus `RPM_MARGIN`) with the fastest exit speed any hood angle gives, it stops right there. This runs every loop while the wheel spins up, and the full search is slow when nothing scores.
2. The CELL opening isn't a point. It tries 7 crossing heights across a ±3 in. band (`TARGET_BAND`), the center first, then outward, so the most forgiving shot wins.
3. For each height it solves both arcs, low then high, and clamps each into the hood's real range.
4. A clamped angle is only kept if the ball still crosses inside the band and passes over the opening's bottom edge, which sticks out 3.5 in. toward the robot because the opening leans back 30°. It has to do that at the reading and at 50 RPM either side.
5. The low arc is tried at every height before any high arc. And when the wheel is too fast for the hood's flattest angle, it waits instead of switching to a lob. Either way, the hood never jumps 30° onto a different kind of shot in the middle of a burst as the wheel's speed changes. A desktop test sweeps every 25 RPM at every distance from 20 to 110 in. and checks the angle never moves more than 5° in one step.
6. Up close, all 7 heights can fall between the band and the lip even though some angles score. Then it walks the hood's whole range in 0.1° steps and takes the middle of the flattest stretch that scores, which leaves the most room for the hood being a little off.
7. Exit speed is a small table by RPM (`COAX_EXIT_SPEED`), times a factor by hood angle (`COAX_EXIT_SPEED_ANGLE`): on a coax hood the ball wraps the wheel for a different arc at each angle, so it can leave faster or slower. Ditto switched between a near and a far value at one distance, which was probably catching the same thing. Since the speed now depends on the angle it is solving for, the solver re-solves a few times until the angle settles, then checks that final angle with its own speed before using it.

The same physics gives the flight time for shoot-on-the-move (`LEAD`), so no flight time table is needed, and it works out passes too: a lob at about `CoaxHood.PASS_LAUNCH_DEG`, with the RPM that lands it on the pass target and the hood solved for the real RPM like a shot. When the wheel is within 300 RPM of target and still no angle scores (too close or too far for the hood), the drivers get the out-of-range warning. `Robot.HOOD` only switches between bursts, and a switch resets the trims, since each was set for the other hood. While the coax hood shoots, driver 2's up/down trim moves the hood (0.5° a press, `HOOD_TRIM_STEP_DEG`) instead of the RPM, since an RPM trim would just be solved away.

### Hood position without a sensor

A hobby servo can't report where it is: `Servo.getPosition()` only gives back the last position sent. So `CoaxHood` keeps an estimate that moves toward the commanded angle at the hood's turning speed (`SPEED_DEG_PER_SEC`, a 200°/s placeholder until it is timed with slow-motion video in step 16), and `atTarget()` turns true once the estimate is within 0.5° (`ON_ANGLE_DEG`). A shot waits for that. A slow loop counts as at most 0.1 s, so the estimate never runs ahead of the hood. At power-on the hood could be anywhere, so the estimate isn't trusted until a move across the whole range would have finished (35° at 200°/s is under 0.2 s). `Robot` moves the hood before it checks whether the shot is ready, so this loop's move already counts. If the angle-to-servo mapping ever asks for a servo position past 0 to 1, the servo stops at its end while the estimate still arrives, so the drivers get the `HOOD_AT_END` warning instead of shots that quietly go at the wrong angle.

### A seed table from physics

`COAX_RPM` needs a wheel speed for each distance before any test shot is fired. `HoodSolver.seedRpm` works one out from the physics, and the Coax Hood Test OpMode shows the seeds in init, ready to copy. This is Ditto's other idea ([`LutTuningSolver`](https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/LutTuningSolver.java)) with a different rule. Ditto seeded the flattest shot. That doesn't suit our CELL, which is high for the distances we shoot from: the flattest shots took about 5400 to 5500 RPM from 25 to 60 in., and from 65 in. out they sat on the hood's flattest angle with no room to correct a fast wheel. So our seed is the slowest RPM where the hood can still score with the wheel 300 RPM off either way (`SEED_RPM_WINDOW`). The hood always has room both ways, and a slower wheel recovers faster between balls. Where no RPM has that much room (20, 95 and 110 in.), the seed is the middle of the RPMs that score. The seeds rise from 1850 RPM at 20 in. to 2400 at 95 and 110 in. 20 in. is at the edge of what a 75° hood can do: only about 1810 to 1850 RPM scores there. The exit speeds behind these seeds are guesses too, so the rows get corrected with real shots in step 16.

### Infinite-rotation turret with no homing

`robot/subsystems/Turret` is an Axon servo in continuous mode, geared so the turret turns twice per servo turn, with an absolute encoder on the turret itself. The true angle is known at power-on, so there is no homing, and the turret always takes the shortest way to the target since it can spin forever.

### Turret encoder calibration

The encoder's voltage climbs through one turn and drops back at the end. Dividing by the most the hub port can read (`AnalogInput.getMaxVoltage()`) is the easy way, but that isn't the encoder's real range, so a turn would read a little more or less than 360° and the angle would jump where the output wraps. We measure the encoder's lowest and highest clean voltages instead (`ENCODER_MIN_VOLTS`, 0, and `ENCODER_FULL_TURN_VOLTS`, 3.2, the value Meta-Infinity used, until ours is measured in step 5), so one turn reads exactly 360°. The encoder's top end is noisy, which is why the highest clean reading is used, not the highest reading.

### Our own turret controller

The turret runs on our own PIDF controller (`util/PIDF`, 17 desktop tests), scaled for battery voltage:

- P pushes harder the farther the turret is from its target. There are two gain sets, one for big moves and one for settling, with a 2° band (`CLOSE_ZONE_BAND_DEG`) so it doesn't flicker between them.
- I closes the last degree or two when friction stops the turret just short. It only builds up within 5° of the target (`I_ZONE_DEG`), never pushes more than 0.1 power (`I_MAX_POWER`), stops growing while the output is already at its limit (anti-windup), and starts over when the turret crosses the target or the target jumps. It holds still while the stuck check tests the turret and starts over while it rests. It stays off (`KI` is 0) unless tuning shows the turret stopping short.
- D brakes using how fast the turret really turns, not how fast the error changes, so picking a new target, a trim press or the angle wrapping past 180° can't kick it. A reading that jumps faster than 2000°/s is a bad reading, and that loop keeps the last speed.
- F turns the turret along with the robot. When the robot spins, the turret's target angle moves, and a push proportional to that speed (`KV`, off until step 11) starts before an error builds up. The target speed is smoothed and capped at 720°/s (`MAX_TARGET_RATE_DEG_PER_SEC`), and a target that jumps more than 30° in one loop counts as a new target, not a fast one.

A static push (`KS`) fades in over the last 5° so it doesn't buzz at the target. Telemetry shows the size of each part, for tuning.

## 3. Storage and intake

### 4 slots from 3 sensors

`robot/subsystems/Storage` has break beams at slots 1, 2 and 4, each debounced by time (30 ms). Every ball crosses the slot 4 beam on its way in, so "full" means that beam stays blocked for 150 ms. Firing only needs to know "2 or more", so slot 3 needs no sensor. The drivers see 0, 1, 2-3 or 4.

### A rail that holds without stalling

The door-end rail motor runs at full power until a ball sits in slot 1, then drops to 0.15, enough to keep the ball against the closed door without stalling the motor. The intake end stops at 4 balls, the possession limit (G407). Full storage doesn't raise the flower intake: in TeleOp only driver 1's LB moves it. Unjam reverses everything and wins over feeding and collecting, any time.

## 4. Safety

### Nothing fires without an alliance and a trusted pose

`Robot.canFire()` needs both. A pose is trusted only when it comes fresh from auto or from a relocalize. TeleOp without one still drives, but won't shoot until the drivers relocalize.

### Nothing moves during INIT

The rules say the robot must sit still once INIT is done (G304.H), and nothing may move under power between auto and TeleOp (G403). Our INIT code sends no motor power and no servo position: the subsystems only look up their hardware, and the write caches send nothing until the first loop after Start. That is when the door closes, the flower intake goes up, the turret and the coax hood turn to their targets, the rail starts collecting, and the shooter spins up if balls are stored. So Main TeleOp can be INITed during the switch from auto. Whether anything twitches still has to be checked on the robot (step 10). Button presses made during INIT are thrown away at Start (`resetEdgeDetection()`), in Main TeleOp and every test OpMode. Without that, a D-pad press in INIT would relocalize on the first loop and mark a guessed pose as trusted.

### Safe pose hand-off

`robot/RobotState` carries the alliance, target cell and pose from auto into TeleOp. Only trusted poses are saved, and a saved pose expires after 60 s. The age is measured with `System.nanoTime()`, because the Control Hub's wall clock can jump when it syncs. So auto's pose reaches TeleOp, but a guess or last match's pose never does. Switching alliance in init drops the saved pose, since it is in the other alliance's frame.

### NaN poses

An odometry glitch can return NaN. A NaN pose or velocity stops aiming (NaN would send the flywheel to its top RPM and the turret to a NaN angle), raises the `POSE_NAN` warning, and is never saved. A burst that is already firing pauses its rail but keeps its last good RPM and turret angle, so a ball already past the door doesn't go into a slowing wheel.

### Watchdogs that latch off

Each watchdog cuts its own mechanism's power and keeps it off until the OpMode restarts, with a warning on screen and on the gamepad.

The shooter's watchdog catches an unplugged or reversed encoder, either of which would make the controller hold full power forever. If power stays above 0.5 for 500 ms while the signed RPM stays under 100, it turns off. The RPM is signed on purpose: a wrong sign reads large and negative, which an absolute value would miss.

The turret runs four checks:

- Wrong way. Degrees turned against its own push add up, and degrees turned with it take away. Past 90° it turns off. That catches the two `REVERSED` flags disagreeing within about 1.5 s (about 0.3 s when the target is near). The encoder measures the turret against the chassis, so the robot turning doesn't count, and a reading that jumps faster than 2000°/s is skipped as noise.
- Stuck. Pushing at the static power for 500 ms without moving 3° starts a test, because friction, a dead encoder and a hand holding the turret all look alike. The test ramps the push up to 0.4. Friction gives way and the turret moves on. If it still hasn't moved, power goes off for 1 s and it watches again. Three failed tests in a row latch it off. A turret held for a few seconds comes back, and a dead encoder is off in about 4 to 6 s.
- Creep. A push under the static power can't start a turret at rest, but it can keep a moving one creeping. Once the angle has stood still for 500 ms, that push is cut to 0, so an encoder that freezes mid-move can't leave the turret creeping.
- Full power, no progress. A backup: 1.5 s at full power without the error shrinking 10° turns it off, unless the target moved more than 20° in that time.

One case is left: an encoder that freezes right next to the target. The turret stops within half a second, but it believes the frozen reading, since a frozen reading and a turret at rest look the same.

### A coax hood that is optional

The coax hood's servo is optional in the robot configuration (`CoaxHood.isConfigured`). Without it, `Robot.coaxHood` is null and the robot shoots with the static hood's numbers, so the same code runs on the robot with either hood. If `Robot.HOOD` asks for `COAX` and the servo is missing, it falls back to `STATIC` and shows the `HOOD_MISSING` warning. A configuration mistake is a warning on the Driver Station, not a crash at init.

### Path speed limits put back

Pedro writes a path's speed limit into the follower's settings and only restores the old value when the follower lets go of the path. Every OpMode calls `robot.stop()` after its loop, so an auto that ends mid-path can't leave TeleOp driving slowly.

### Auto drives that end safely

Two things in the libraries could leave an auto stuck or moving. Ivy's ready-made follow command has no end step, so when it is cancelled the robot keeps driving the old path. And Pedro never gives up on a path by itself, so a blocked robot could wait there all auto. `RobotCommands.follow` and `driveTo` hold the robot where it is when they are cancelled or time out. `AutoTemplate` gives its drive a time limit (`DRIVE_TIMEOUT_MS`, 4000 ms until the real drive is timed), the shooting commands give up after `SHOOT_ONE_TIMEOUT_MS` (1500 ms) or `SHOOT_ALL_TIMEOUT_MS` (3000 ms), and every AgateFlow drive has its own deadline (section 6). The plain `driveTo(pose)` has no limit, and its comment says to use the timed one wherever the robot could get stuck.

## 5. Driving and driver feedback

### Field-centric driving that holds its ground

`robot/subsystems/Drive`: stick up drives away from the driver on either alliance. The stick is shaped as one vector (deadband, then a cubic curve), so a diagonal stays at the angle the stick points. When the sticks are let go and the robot slows under 2 in/s, Pedro holds the pose, so a defender can't push it off a shot.

### One-button relocalize

The robot can be pushed into either corner on our alliance wall, intake against the alliance wall or the side wall. That gives 4 spots, each with its own button. The pose snaps to that spot, becomes trusted, and the aim trims reset, since they mostly covered pose drift. Every pose in the code is written once for red and spun 180° about the field center for blue (`field/Alliance.fromRed`). The BIOBUZZ field is a spin, not a mirror.

### One source for driver warnings

`robot/Warning` lists everything the drivers must know: no alliance, pose unknown, NaN pose, out of range, shooter off, turret off, shot table out of order, coax hood picked but no hood servo configured (`HOOD_MISSING`), and hood servo at the end of its range (`HOOD_AT_END`). `Robot.warnings()` says which are active. Telemetry prints them even in competition mode, and `driver/Feedback` buzzes them on driver 1's gamepad. A new warning is one line in one place.

Driver 1's gamepad has only two rumbles, so each always means the same thing. Three strong pulses mean a warning appeared (the same warning won't buzz again for 3 s, and "out of range" never buzzes since it's normal while driving). A soft constant rumble means storage is full. Driver 1's gamepad light shows the ball count: red for 0-1 (can't shoot), yellow for 2-3, green for 4. In init both gamepads light up in the alliance color, or white when none is picked.

## 6. AgateFlow, our path planner

AgateFlow (`agateflow/`) plans Pedro paths on the fly, from wherever the robot is, around everything on the field, and picks the fastest way that keeps the robot clear. A route reads like `new Route().to(SHOOT_SPOT).heading(Heading.finishBy(0.5))`. `AgateFlow.plan` returns a `Plan` of legs, one Pedro `Path` per stop. Each class's top comment has the full details.

### Making a plan

1. Shortest ways around. Each zone is grown by the robot's radius plus a 2 in. safety margin. The shortest way around convex shapes runs along lines that touch their corners, so `VisibilityGraph` joins the grown corners with tangent lines and searches with A*. Each turn adds a cost of `weight × (1 - cos(turn angle))`. The search runs with weights 0, 12 and 36, with and without 4 in. of extra room at corners, which gives a few candidates: tight and twisty, or a little longer with fewer, gentler turns.
2. Round the corners. Each corner becomes a `CornerCurve`, a degree-5 Bezier curve that meets both lines with zero curvature (G2), so the robot is never jerked sideways where a line meets a curve. Its control points were found by search to give the lowest peak curvature, within about 3% of a circular arc, which can't be G2. Each corner is as wide as the room around it allows.
3. Drive it in simulation. `MotionModel` drives each candidate the way Pedro's Foresight follower does, in 10 ms steps, on `DriveModel` physics: motor speed proportional to voltage, a time constant to get there, battery sag under load, different forward and strafe top speeds, and grip limited to about 0.8 g. Foresight steers toward a point one brake distance ahead, so at speed it cuts inside curves. The model predicts that too, and the predicted path is what gets checked against the zones.
4. Slow down only where needed. Where the predicted path comes too close, a Pedro speed-limit Modifier goes on that part of the path, starting early enough to slow down in time, and the candidate is driven again.
5. Pick the fastest safe one. If none keeps the full margin, it takes the quickest one that misses by under a quarter of it and still keeps the robot itself clear, and says so in `Plan.notes`.

Planning stops trying new candidates after 100 ms, and at least one is always tried. With no time limit, the same route plans to the same result every time. Paths never contain hairpins, which Foresight can stick on: a through-waypoint that would need one becomes a stop, with a note.

### The robot's real shape, and the walls

The robot is checked as its outline, a rectangle around the turret center (7.99 in. to the front, 5.72 in. to the back, 5.87 in. to each side), on both the planned and the predicted path. So it may pass something side-on closer than it could facing it. The four field walls are treated differently from everything else. Any side of the robot may touch a wall, but it is never planned to move into one faster than 10 in/s, so it slides along a wall or eases onto it. Every other zone keeps the full margin from the whole outline.

Where the robot doesn't fit, AgateFlow fixes it or refuses, and writes a note. It moves a waypoint (keeping its heading) until the outline is 0.5 in. clear, leaves out a pickup target it can't reach, and refuses a turn in place that would swing a corner into something. Starting against a wall, it keeps its start heading for up to 24 in. before turning, and it may finish a turn up to 24 in. before a waypoint next to one. It never changes a heading the route asks for.

### The field from the rule book

`field/FieldZones` builds the keep-out zones from the Competition Manual (TU03, Section 9), checked against the official field CAD. The HIVES never come lower than 30.6 in. and our robot is 12.74 in. tall, so it fits under them. The only things it can hit are the 4 FLOWERS and the HIVE A-frames' foot bars and legs, filled in from the tiles up to the robot's top. Coordinates match Pedro Visualizer's frame and its 141.5 in. field, so points copy over 1:1.

### Picking up POLLEN in the best order

`Route.pickUp(targets)` drives over floor targets intake first, in the fastest order (`agateflow/Pickup`). Targets close enough for one pass of the intake become one group. Every hop between groups gets a quick time estimate: shortest way around the zones, full speed between groups, 30 in/s through each one, slower where it turns. With up to 7 groups it tries every order. With more (up to 12) it uses Held-Karp dynamic programming, which can miss the best order by a few percent because the time lost turning depends on the group before. The best few orders within 15% are then planned in full, and the fastest real plan wins. `.most(n)` takes at most n, for when storage only has room for that many.

After planning, every target is run past the predicted path. It counts as picked up only if it goes 2 in. into the intake mouth at no more than 33 in/s. `Plan.pickedUp()` and `Plan.skipped()` say which.

### The flower intake changes the robot's shape

Stretches marked `flowerIntakeDown()` are planned with the robot's front at the flower intake's reach, which lets it reach into a FLOWER's Retrieval Opening on purpose. Every other stretch is planned with the intake up, except the first 12 in. after a down stretch: there the robot backs straight out without turning, so the intake can't sweep through the FLOWER while it rises. `RobotCommands.withFlowerIntake(drive)` lowers and raises the real intake to match the plan, stretch by stretch. A replan in those first 12 in., or a park drive that starts at a FLOWER with the intake down, plans the same back-out.

### Driving a plan

`AgateFlowCommands.driveTo(route)` gives an Ivy command, `DriveRoute`.

- Planning runs on one background thread, so the loop never waits. `planFrom(startPose)` in init has the first plan ready when auto starts.
- It replans when the robot is pushed more than 8 in. off its predicted path, or when new zones take room from the path ahead. Near a stop, where the robot is meant to get close to things, only a new zone it would hit counts (one that leaves less than `ROBOT_RADIUS`), so a partner robot parked on our stop makes it replan, and the replan moves the stop out of the way. A replan that comes back after the robot has moved on to the next leg is dropped.
- While the zones differ from the plan's, it checks the predicted path 30 in. ahead every loop. A new zone there makes it brake straight against its motion at 0.5 power until nearly still, hold, and replan from there. Full reverse power would sag the battery to about 7.4 V by `DriveModel`'s numbers, a brownout risk.
- It gives up at 1.5 × its first estimate + 1 s, so an auto moves on instead of hanging. Replans can push that deadline at most 5 s later (`REPLAN_EXTRA_SECONDS`). A detour around something new gets its own plan's time instead, up to 3 times (`MAX_DETOURS`). The 2026-10-10 review found that this never happened (a flag was cleared before it was read). It is fixed, and `DriveRouteTest` now drives the case both ways: with the detour's time the robot gets there, and without it the same drive times out. A robot still getting closer at the deadline gets up to 3 s more, and a stuck one stops at once. Progress is measured from scratch with each new plan, since a detour may first lead away from the stop.
- Vision feeds zones through `FieldMap.setZones("vision", ...)`, which is safe to call from a camera thread. Each plan works on a snapshot that never changes, null or NaN zones are dropped, and calling it every frame with the same zones rebuilds nothing.

### Learning its own error

After each leg driven start to finish on its own plan that reached its stop, `EtaCalibration` compares the predicted and real times and moves `TIME_SCALE` a fifth of the way toward real/predicted (`LEARN_FILTER`, 0.8, a smoothing knob like every other in the code). It skips legs that say nothing about the estimate: the first leg of a replan (it starts wherever the robot was when the replan was asked for), a leg the robot was pushed or stopped on, a leg it gave up settling on, and a leg of `driveTo(estimate)` it joined partway. Legs predicted under 0.5 s, and ratios outside 0.5 to 2 (bumped or stuck), don't count either. It shows in Panels, so we can watch it settle over a few autos and copy it into the code.

### Estimates and parking on time

`AgateFlowCommands.estimate(route)` times a route in the background without driving it, so an auto can check whether there's time for one more cycle. `driveTo(estimate)` then reuses that plan without waiting, as long as it still fits: the robot is within 1 in. and 5° of where it was made, or the estimate is under 0.5 s old and the robot is on its predicted path.

`ParkGuard` re-estimates the park drive every 250 ms from wherever the robot is. When the time left reaches that estimate plus 0.75 s, it interrupts every running command (the park drive requires every subsystem, at a higher priority) and parks. If no estimate has worked, it parks 5 s before the end. If the park drive ends without getting there (no plan, out of time, interrupted) or was never allowed to start, it starts it again while time is left. Its estimates share the one planning thread with the drives, so it waits while a drive's own plan is being made. It also reads whether the flower intake is down, so parking from a FLOWER plans the back-out first. One gap is left: the intake reads as up as soon as parking commands it up, so a retry in the first 0.4 s (`FLOWER_RAISE_MS`) plans without the back-out. `DriverAssist` does the same kind of drive in TeleOp: one button drives a route around everything on the map, and touching the sticks hands control straight back.

### Knowing the drivetrain

`DriveModel` gets its numbers from `DriveModelTuner` (`pedro/procedures/`, ours). It makes full-power runs forward, strafing and turning, then fits acceleration against volts and speed with a straight line, which gives top speed per volt, the time constant and the battery's resistance. The volts are measured during each run, so battery sag doesn't skew the fit.

Foresight's gains are read straight from the follower instead of copied by hand. Some of its controllers keep their gains private, so each one is probed: reset it, ask for its output at two small errors, check the two ratios agree, and reset it again. It only probes while the follower is idle or driven by hand, never while it follows a path or holds a pose. A gain that isn't simply proportional falls back to the tuned fields.

### Seeing plans

`PlanDrawing.toPanels` draws the zones, the planned path, the predicted path, the robot's outline at each stop and the pickup targets on the Panels field page, at most every 100 ms. Every packet carries the whole drawing over Wi-Fi, so paths are drawn in 2 in. pieces. The `AgateFlow Benchmark` test OpMode times planning on the Control Hub without moving, then shows each plan on Panels.

## 7. How the code is tested

181 JUnit tests in 27 classes run on a laptop with `./gradlew :TeamCode:testDebugUnitTest`. All passed in a Gradle run on 2026-10-10. They test the planner, fire control and the math. `Robot`'s per-loop decisions and the subsystems' hardware side have no desktop tests yet; they get checked on the robot.

### AgateFlow (74 tests)

`TestRobot` is Pedro's real `Follower` and `Foresight` driving a simulated mecanum drivetrain with the physics `DriveModel` assumes, on a virtual clock (10 ms steps, as fast as the computer goes). Foresight is set up the way its tuner would set it up. So the tests drive AgateFlow's plans with the real follower code, not a copy of it.

`TrueRule` is the tests' own collision judge, written separately from the planner's `ClearanceRule`: the robot's outline may not overlap any zone, with no safety margin. A bug in the planner's rule can't hide by also being in the judge.

What the tests require:

- Predicted drive time within 4% of the simulated drive for straight lines and turns, and within 5% for corners.
- 40 random fields (fixed seed, up to 3 random boxes and circles, 1 to 3 waypoints, some moving starts, both alliances). At least 80% must plan. Every plan keeps `TrueRule` on both its planned and its predicted path. 12 of them are driven to the end, within 15% of the predicted time.
- Held-Karp against all 40,320 orders of 8 groups, in 30 random cases: never more than 10% slower than the best, under 2% on average, and exactly the best in at least half.
- Blue plans are red plans spun around, the same route gives the same plan, bad input (NaN pose, no alliance, empty route) comes back as a problem instead of a crash, the flower intake goes up and down with the plan, and every Benchmark route plans for both alliances.

Pictures of the test plans are saved to `TeamCode/build/agateflow-svg/`.

The simulated robot follows `DriveModel`'s physics exactly, so these numbers only measure how well AgateFlow copies Foresight. The real robot will differ more until `DriveModelTuner` has run, and `EtaCalibration` is there to close the rest of the gap.

Three classes test the layer that drives the plans, on the same virtual clock (`DriveRoute.NOW`). They were added after the detour bug in section 6 got through with no test:

- `DriveRouteTest` (8): a zone that shows up ahead makes the robot brake, hold and detour with the detour's own time, and the same drive times out without it. A partner parked on a stop moves the stop. Replans for pushes end at the cap. A robot still getting there gets overtime, and a stuck one doesn't. A replan that comes back after the next leg started is dropped. A route that starts with the flower intake down backs out.
- `ParkGuardTest` (3): it starts parking when the time left reaches the estimate plus the margin, interrupts the auto and gets there, and it starts again after no plan or after being interrupted.
- `EtaCalibrationTest` (6): the filter, the legs it ignores, and which legs a drive reports: every leg of a clean drive, only the later legs after a replan, none that gave up settling, and only the later legs of a drive joined partway.

### Fire control, shooting, turret and helpers (107 tests)

- `FireControlTest` (14): a burst starts once the shot stays ready for `READY_HOLD_MS`, and the rail waits for the door to open fully. Feeding goes on while the wheel sags 400 RPM. The rail pauses while the aim is off and below `BURST_RPM_FLOOR`, and goes on after. Release stops the rail at once and closes the door after the delay. A switch to pass ends the burst, and the pass waits for its own shot. `stopAfterOneShot` ends the burst after one dip. A sharp dip counts wherever the wheel starts, a slow coast or a wheel speeding up doesn't, and a deep dip counts once.
- `HoodSolverTest` (24): every angle it gives crosses inside the band and clears the lip, also when the exit speed changes with the angle. The RPM margin refuses an angle that only scores at the exact reading. Up close it finds every angle a 0.1° walk finds. A sweep every 25 RPM at every distance from 20 to 110 in. checks the hood never moves more than 5° in one step and never flips onto a lob. Bad inputs give NaN, and flight time, seeds and passes agree with the physics.
- `MotionPredictorTest` (10): constant velocity predicts a straight line, a braking robot stops and never backs up (also right after it stops), the lag correction never flips the sign, and the acceleration estimate is capped and skips samples too close together, bad readings and old samples.
- `ShotSolverTest` (4): with `PREDICT` off every shot is solved from where the robot is now, with the velocity as read, a robot braking to a stop gets no lead the wrong way, the turret aims from the pose at `TURRET_LATENCY_SEC` while the distance uses the release, and the acceleration learned with `PREDICT` off is used the moment it is turned on.
- `PIDFTest` (17): each part's sign and size, the integral's zone, cap, anti-windup, pause and reset when the turret crosses the target, no kick from a target jump, a trim change or the angle wrapping past 180°, the target speed cap, and a NaN target that doesn't stick.
- `CachedHardwareTest` (4): the write caches skip repeats, but never a stop, and never the first write after `raw()` handed out the device.
- `ShotSolverCoaxTest` (6): a coax pass is valid only inside `PASS_RPM`'s distances, gets no hood angle with the wheel too slow, and gives nothing from a bad Panels number. Coax `LEAD` uses the physics flight time, and falls back to `SHOT_FLIGHT_TIME` when no angle scores.
- `RetryTest` (5): it stops at the first try that works, gives up after the limit, uses the retry command for later tries, and ends each try exactly once, even when it is ended twice from outside.
- `FieldTest` (7): red poses stay as written, blue is red spun 180° about the center, the cells mirror only in x, corner poses touch the walls, the target cell follows the robot's half with a band in the middle, and the pass target is written for red.
- `AnglesTest` (3): wrapping into -180 to 180 works for negative angles too (Java's `%` keeps the sign), and a difference takes the short way around.
- `LookupTableTest` (5), `DriveCurveTest` (4), `DebouncerTest` (4): the tables interpolate in a straight line between rows and hold their end values past either end; the stick curve is zero inside the deadband, starts from zero just past it and gives full speed at full stick; the debouncer ignores a flicker shorter than its hold time and reads the hold time on every call.

---

Some ideas came from two DECODE (2025-26) teams: [GNCE Ditto](https://github.com/GNCE/DECODEv2-Ditto) and [Meta-Infinity](https://github.com/Meta-Infinity/DecodePublicRelease). Ditto's links above point at their final commit (`c78b526`, June 30 2026), so they keep showing the code we read: the coax hood physics, the seed table, the hood's angle-to-servo mapping through a gear ratio, averaging the wheel's speed for the hood, the flywheel anticipation term and the shoot-on-the-move timing all come from there, with the changes each section names. Meta-Infinity's turret encoder setting (3.2 V at a full turn) is our starting value until ours is measured.

**For the Control Award portfolio** (Competition Manual 6.3.7, A201 and A202, TU04). This file is source material, not the portfolio. The portfolio says what each control part is, which game problem it solves and how it works, and judges look for parts that use sensor feedback. It has at most 15 pages of content plus a cover page, stays under 15 MB, shows only work from 2026-01-01 on, and needs no code. Judges don't click links or watch videos, so credits like the ones above go in words. AI tools are allowed if the portfolio credits them in a footnote or endnote. Parts of this code and of these notes were written with an AI coding assistant, so the portfolio must say so.
