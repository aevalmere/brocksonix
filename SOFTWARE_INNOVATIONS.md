# Software innovations

What is new or unusual in our robot code for BIOBUZZ (2026-27), how each part works, and where to find it. Paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Names in capitals are Panels tunables. The numbers given for them are the current defaults, and most are still guesses until the robot is tuned.

**Status (2026-10-08).** None of this has run on the robot yet. Pedro isn't tuned (`Constants.create` still returns null), so no match OpMode can start. AgateFlow, the path planner, is tested on a laptop against a simulated robot (57 JUnit tests, all passing) and isn't used by any match OpMode yet. The rest gets checked on the robot with the test OpModes, following `TUNING_GUIDE.md`.

1. [How the code is built](#1-how-the-code-is-built)
2. [Shooting](#2-shooting)
3. [Storage and intake](#3-storage-and-intake)
4. [Safety](#4-safety)
5. [Driving and driver feedback](#5-driving-and-driver-feedback)
6. [AgateFlow, our path planner](#6-agateflow-our-path-planner)
7. [How AgateFlow is tested](#7-how-agateflow-is-tested)

## 1. How the code is built

### Auto and TeleOp share one brain

`robot/Robot.update()` makes every decision once per loop: what the rail, door, shooter and turret do, and when a ball fires. OpModes only say what the drivers want. TeleOp sets `shootHeld`, `passHeld` and `unjamHeld` from the buttons. Autonomous commands (`robot/commands/RobotCommands`) set the same fields and wait on what `Robot` reports, like `shotsFired()` or `storage.isEmpty()`. Aiming, firing and safety rules exist in one copy, so a fix found in TeleOp practice fixes auto too.

`Retry` reruns an auto command while a check fails. `shootAllWithRetry` uses it: if balls are still stored after a burst times out, it reverses the rail for 200 ms and fires again. An auto with no alliance picked does nothing, because shooting at the wrong goal is worse than sitting still.

### Read, decide, write

Each loop reads every sensor first, decides, then writes every output at the end. So the decision to fire uses this loop's turret angle and this loop's flywheel speed, never one of each. Shoot versus pass is decided once at the top, so aiming and fire control can't disagree about which target they serve.

The loop stays fast: one bulk read per hub, motor and servo writes only when the value changes (`robot/hardware/Cached*`), battery voltage re-read every 250 ms, and no motor current reads. The shooter's motors use a finer write step (0.002 instead of 0.01), since 0.01 power is about 60 RPM of feedforward and small Panels edits would look dead. With `MainTeleOp.FULL_TELEMETRY` off, telemetry drops to the warnings and loop time every 200 ms. The goal is under 15 ms per loop.

### Everything tunable live

Every tunable is a non-final `public static` field in a `@Configurable` class, read where it's used and never only in a constructor, so a Panels edit applies on the next loop. That includes the shot tables, direction flags and auto poses. Direction flags flip the sign when power is written instead of calling `setDirection`, which would also flip the shooter's encoder reading. If a shot table edit puts its distances out of order, the `SHOT_TABLE_ORDER` warning shows instead of silently wrong RPMs.

### Tuning checklist built from the code

Anything that still needs measuring has a `// TODO(step): what to do` comment right above it, where the step is the `TUNING_GUIDE.md` section. The steps are numbered in order of importance and grouped into four phases: the bare minimum for TeleOp, a first auto, making both better, then AgateFlow. The `todoList` Gradle task runs before every build and rewrites `TODO_LIST.md` in that order, with a count per phase, a link to each line and the field it sits on. A TODO written any other way, or with a step that doesn't exist, gets a build warning, so nothing drops off the list by accident.

## 2. Shooting

### Shoot-on-the-move in layers

`robot/shot/ShotSolver` turns the robot's pose and velocity into a turret angle, flywheel RPM and feed power. It has three layers, each with its own switch:

1. Static: aim at the cell, RPM and feed power from distance tables. Always on.
2. `LEAD`: a ball leaves with the robot's velocity, so aim at `target - velocity × flight time`. Moving the aim point changes the distance and so the flight time, so this repeats 3 times.
3. `PREDICT`: the turret needs time to react (`TURRET_LATENCY_SEC`, 0.08 s), so solve from where the robot will be by then.

`LEAD` and `PREDICT` start off and get turned on one at a time after stationary shots work. On the field, a bad layer can be switched off without touching code.

The HIVE tips during the match, so the robot aims at the cell on its own half of the field. Within 6 in. of the center line it keeps the cell it has, so the turret doesn't flip back and forth while driving along the middle. Passing uses the static layer only and is refused outside its table's range, since a ball sent out of the field is a MAJOR FOUL (G405). It stays off until the variable hood is on.

### One ball at a time, confirmed by the flywheel

`robot/FireControl` takes each ball through three steps:

1. Wait: turret on target and flywheel at speed, steady for 40 ms, so one lucky reading can't fire.
2. Feed: push the ball until the flywheel dips 150 RPM, which means the ball hit the wheel. No dip within 400 ms counts as a misfeed. The target RPM is latched when the feed starts, because the live target moves while driving and could fake a dip.
3. Recover: pause 60 ms, then wait again.

The door opens at the first ready moment and stays open for the whole burst. The rail is what pauses between balls, so the door never closes on a ball. Letting go of the button mid-feed lets that ball finish. After the burst the rail waits until the door is fully closed, so the next ball can't reach a half-open door. A burst needs 2 balls in TeleOp. Auto may allow 1.

### Flywheel at speed before the trigger

`robot/subsystems/Shooter` uses feedforward plus P control, scaled for battery voltage (capped at 1.5×, so a bad 0 V reading can't run away). During a burst, boost switches to full power whenever the wheel falls more than 50 RPM below target, so it recovers between balls as fast as the motors can. The wheel spins up whenever a ball is stored and stays up through the whole burst, so it is ready when the trigger is pulled and isn't cut while the last ball goes through.

### Infinite-rotation turret with no homing

`robot/subsystems/Turret` is an Axon servo in continuous mode, geared so the turret turns twice per servo turn, with an absolute encoder on the turret itself. The true angle is known at power-on, so there is no homing, and the turret always takes the shortest way to the target since it can spin forever.

Control has three parts, all scaled for battery voltage. Two PD gain sets, one for big moves and one for settling, with a 2° band so it doesn't flicker between them. A static push that fades in over the last 5° so it doesn't buzz at the target. And a term for how fast the target moves, which cancels the robot turning underneath the turret.

## 3. Storage and intake

### 4 slots from 3 sensors

`robot/subsystems/Storage` has break beams at slots 1, 2 and 4, each debounced by time (30 ms). Every ball crosses the slot 4 beam on its way in, so "full" means that beam stays blocked for 150 ms. Firing only needs to know "2 or more", so slot 3 needs no sensor. The drivers see 0, 1, 2-3 or 4.

### A rail that holds without stalling

The door-end rail motor runs at full power until a ball sits in slot 1, then drops to 0.15, enough to keep the ball against the closed door without stalling the motor. The intake end stops at 4 balls, the possession limit (G407), and the flower intake raises itself the moment storage fills. Unjam reverses everything and wins over feeding and collecting, any time.

## 4. Safety

### Nothing fires without an alliance and a trusted pose

`Robot.canFire()` needs both. A pose is trusted only when it comes fresh from auto or from a relocalize. TeleOp without one still drives, but won't shoot until the drivers relocalize.

### Safe pose hand-off

`robot/RobotState` carries the alliance, target cell and pose from auto into TeleOp. Only trusted poses are saved, and a saved pose expires after 60 s. The age is measured with `System.nanoTime()`, because the Control Hub's wall clock can jump when it syncs. So auto's pose reaches TeleOp, but a guess or last match's pose never does. Switching alliance in init drops the saved pose, since it is in the other alliance's frame.

### NaN poses

An odometry glitch can return NaN. A NaN pose or velocity stops aiming (NaN would send the flywheel to its top RPM and the turret to a NaN angle), raises the `POSE_NAN` warning, and is never saved. A burst that is already firing keeps its last good RPM, turret angle and feed power, so the ball being fed doesn't go into a slowing wheel.

### Watchdogs that latch off

Each watchdog cuts its own mechanism's power and keeps it off until the OpMode restarts, with a warning on screen and on the gamepad.

The shooter's watchdog catches an unplugged or reversed encoder, either of which would make the controller hold full power forever. If power stays above 0.5 for 500 ms while the signed RPM stays under 100, it turns off. The RPM is signed on purpose: a wrong sign reads large and negative, which an absolute value would miss.

The turret runs four checks:

- Wrong way. Degrees turned against its own push add up, and degrees turned with it take away. Past 90° it turns off. That catches the two `REVERSED` flags disagreeing within about 1.5 s (about 0.3 s when the target is near). The encoder measures the turret against the chassis, so the robot turning doesn't count, and a reading that jumps faster than 2000°/s is skipped as noise.
- Stuck. Pushing at the static power for 500 ms without moving 3° starts a test, because friction, a dead encoder and a hand holding the turret all look alike. The test ramps the push up to 0.4. Friction gives way and the turret moves on. If it still hasn't moved, power goes off for 1 s and it watches again. Three failed tests in a row latch it off. A turret held for a few seconds comes back, and a dead encoder is off in about 4 to 6 s.
- Creep. A push under the static power can't start a turret at rest, but it can keep a moving one creeping. Once the angle has stood still for 500 ms, that push is cut to 0, so an encoder that freezes mid-move can't leave the turret creeping.
- Full power, no progress. A backup: 1.5 s at full power without the error shrinking 10° turns it off, unless the target moved more than 20° in that time.

One case is left: an encoder that freezes right next to the target. The turret stops within half a second, but it believes the frozen reading, since a frozen reading and a turret at rest look the same.

### Path speed limits put back

Pedro writes a path's speed limit into the follower's settings and only restores the old value when the follower lets go of the path. Every OpMode calls `robot.stop()` after its loop, so an auto that ends mid-path can't leave TeleOp driving slowly.

## 5. Driving and driver feedback

### Field-centric driving that holds its ground

`robot/subsystems/Drive`: stick up drives away from the driver on either alliance. The stick is shaped as one vector (deadband, then a cubic curve), so a diagonal stays at the angle the stick points. When the sticks are let go and the robot slows under 2 in/s, Pedro holds the pose, so a defender can't push it off a shot.

### One-button relocalize

The robot can be pushed into either corner on our alliance wall, intake against the alliance wall or the side wall. That gives 4 spots, each with its own button. The pose snaps to that spot, becomes trusted, and the aim trims reset, since they mostly covered pose drift. Every pose in the code is written once for red and spun 180° about the field center for blue (`field/Alliance.fromRed`). The BIOBUZZ field is a spin, not a mirror.

### One source for driver warnings

`robot/Warning` lists everything the drivers must know: no alliance, pose unknown, NaN pose, out of range, shooter off, turret off, and shot table out of order. `Robot.warnings()` says which are active. Telemetry prints them even in competition mode, and `driver/Feedback` buzzes them. A new warning is one line in one place.

The gamepad has only two rumbles, so each always means the same thing. Three strong pulses mean a warning appeared (the same warning won't buzz again for 3 s, and "out of range" never buzzes since it's normal while driving). A soft constant rumble means storage is full. The gamepad light shows the ball count: red for 0-1 (can't shoot), yellow for 2-3, green for 4. In init both gamepads light up in the alliance color, or white when none is picked.

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

Stretches marked `flowerIntakeDown()` are planned with the robot's front at the flower intake's reach, which lets it reach into a FLOWER's Retrieval Opening on purpose. Every other stretch is planned with the intake up, except the first 12 in. after a down stretch: there the robot backs straight out without turning, so the intake can't sweep through the FLOWER while it rises. `RobotCommands.withFlowerIntake(drive)` lowers and raises the real intake to match the plan, stretch by stretch.

### Driving a plan

`AgateFlowCommands.driveTo(route)` gives an Ivy command, `DriveRoute`.

- Planning runs on one background thread, so the loop never waits. `planFrom(startPose)` in init has the first plan ready when auto starts.
- It replans when the robot is pushed more than 8 in. off its predicted path, or when new zones take room from the path ahead. A replan that comes back after the robot has moved on to the next leg is dropped.
- While the zones differ from the plan's, it checks the predicted path 30 in. ahead every loop. A new zone there makes it brake straight against its motion at 0.5 power until nearly still, hold, and replan from there. Full reverse power would sag the battery to about 7.4 V by `DriveModel`'s numbers, a brownout risk.
- It gives up at 1.5 × its first estimate + 1 s, so an auto moves on instead of hanging. Replans can add at most 5 s. A detour around something new gets its own plan's time, at most 3 times. A robot still getting closer at the deadline gets up to 3 s more, and a stuck one stops at once.
- Vision feeds zones through `FieldMap.setZones("vision", ...)`, which is safe to call from a camera thread. Each plan works on a snapshot that never changes, null or NaN zones are dropped, and calling it every frame with the same zones rebuilds nothing.

### Learning its own error

After every leg that ran without a replan, `EtaCalibration` compares the predicted and real times and moves `TIME_SCALE` 20% of the way toward real/predicted. Legs predicted under 0.5 s, and ratios outside 0.5 to 2 (bumped or stuck), don't count. It shows in Panels, so we can watch it settle over a few autos and copy it into the code.

### Estimates and parking on time

`AgateFlowCommands.estimate(route)` times a route in the background without driving it, so an auto can check whether there's time for one more cycle. `driveTo(estimate)` then reuses that plan without waiting, as long as it still fits: the robot is within 1 in. and 5° of where it was made, or the estimate is under 0.5 s old and the robot is on its predicted path.

`ParkGuard` re-estimates the park drive every 250 ms from wherever the robot is. When the time left reaches that estimate plus 0.75 s, it interrupts every running command (the park drive requires every subsystem, at a higher priority) and parks. If no estimate has worked, it parks 5 s before the end. `DriverAssist` does the same kind of drive in TeleOp: one button drives a route around everything on the map, and touching the sticks hands control straight back.

### Knowing the drivetrain

`DriveModel` gets its numbers from `DriveModelTuner` (`pedro/procedures/`, ours). It makes full-power runs forward, strafing and turning, then fits acceleration against volts and speed with a straight line, which gives top speed per volt, the time constant and the battery's resistance. The volts are measured during each run, so battery sag doesn't skew the fit.

Foresight's gains are read straight from the follower instead of copied by hand. Some of its controllers keep their gains private, so each one is probed: reset it, ask for its output at two small errors, check the two ratios agree, and reset it again. It only probes while the follower is idle or driven by hand, never while it follows a path or holds a pose. A gain that isn't simply proportional falls back to the tuned fields.

### Seeing plans

`PlanDrawing.toPanels` draws the zones, the planned path, the predicted path, the robot's outline at each stop and the pickup targets on the Panels field page, at most every 100 ms. A 117 in. route takes 166 lines and about 23 KB per packet. The `AgateFlow Benchmark` test OpMode times planning on the Control Hub without moving, then shows each plan on Panels.

## 7. How AgateFlow is tested

57 JUnit tests run on a laptop with `./gradlew :TeamCode:testDebugUnitTest`. All pass as of 2026-10-08.

`TestRobot` is Pedro's real `Follower` and `Foresight` driving a simulated mecanum drivetrain with the physics `DriveModel` assumes, on a virtual clock (10 ms steps, as fast as the computer goes). Foresight is set up the way its tuner would set it up. So the tests drive AgateFlow's plans with the real follower code, not a copy of it.

`TrueRule` is the tests' own collision judge, written separately from the planner's `ClearanceRule`: the robot's outline may not overlap any zone, with no safety margin. A bug in the planner's rule can't hide by also being in the judge.

What the tests require:

- Predicted drive time within 4% of the simulated drive for straight lines and turns, and within 5% for corners.
- 40 random fields (fixed seed, up to 3 random boxes and circles, 1 to 3 waypoints, some moving starts, both alliances). At least 80% must plan. Every plan keeps `TrueRule` on both its planned and its predicted path. 12 of them are driven to the end, within 15% of the predicted time.
- Held-Karp against all 40,320 orders of 8 groups, in 30 random cases: never more than 10% slower than the best, under 2% on average, and exactly the best in at least half.
- Blue plans are red plans spun around, the same route gives the same plan, bad input (NaN pose, no alliance, empty route) comes back as a problem instead of a crash, the flower intake goes up and down with the plan, and every Benchmark route plans for both alliances.

Pictures of the test plans are saved to `TeamCode/build/agateflow-svg/`.

The simulated robot follows `DriveModel`'s physics exactly, so these numbers only measure how well AgateFlow copies Foresight. The real robot will differ more until `DriveModelTuner` has run, and `EtaCalibration` is there to close the rest of the gap.

---

Some ideas came from two DECODE (2025-26) teams: [GNCE Ditto](https://github.com/GNCE/DECODEv2-Ditto) and [Meta-Infinity](https://github.com/Meta-Infinity/DecodePublicRelease).
