# TeleOp and robot code plan

Plan for the BIOBUZZ robot's code, written before any of it exists. Hardware facts are in `ROBOT.md`; this file covers how the code should be built and driven. Review it, change it, then build from it.

Decisions already made:

- Field-centric drive, one driver does everything (gamepad1).
- Shoot-on-the-move (SOTM) from the start, built in layers so each layer is testable alone.
- Hold-to-fire on RB (score at the HIVE). It needs at least 2 balls stored to start; it never starts with 1.
- RT pass (lob balls to a spot on the floor) is built but **on hold** until the variable hood (`FireControl.PASS_ENABLED = false`).
- Rail intake runs automatically; LB toggles the flower intake.
- Four relocalize buttons, one per corner pose on our alliance wall.
- No endgame.
- Panels for live tuning.
- Camera undecided, so localization is Pinpoint only, built so a camera can be added later.
- Command scheduling with Ivy (already a dependency), on Pedro Pathing 3.x.

## Reference code

Two DECODE (2025-26) teams with robots close to ours: turret, two-motor flywheel, a rail with break beams, Pinpoint.

| | [GNCE Ditto](https://github.com/GNCE/DECODEv2-Ditto) | [Meta-Infinity](https://github.com/Meta-Infinity/DecodePublicRelease) |
|---|---|---|
| Stack | Pedro 2.x, SolversLib commands, Panels | Pedro 2.x, plain LinearOpMode, Panels + FTC Dashboard |
| Turret | 2 positional servos, ±180°, encoder only for wrap counting | **3 CR servos + ELC analog encoder + own PID** (closest to ours) |
| Shooter | PIDF + kV, voltage compensated, adjustable hood | PID + F, **bang-bang while firing**, adjustable hood |
| SOTM | Full model: latency, accel, stop-clamped prediction (~600 lines) | Simple: aim at `goal - velocity * flightTime(distance)` |
| Drive | Robot-centric through Pedro, brake mode | Robot-centric hand-written mecanum, heading lock on a bumper |
| Loop | ~40 ms (slow, they spent effort fixing it) | Telemetry sent only every 300 ms |

Neither repo copies over directly: both are Pedro 2.x and neither has our fixed-angle shooter or an infinite turret. We take ideas, not files.

### What we take, and from whom

| Idea | From | Where it goes |
|---|---|---|
| CR turret PID on the analog encoder, shortest-path error, close/far gain sets with hysteresis, ramped static feedforward, power cap, cached writes | Meta `Turret`, `PIDController` | `Turret` |
| Lead the turret by robot angular velocity | Meta `TURRET_ANG_VEL_LEAD_SEC` | `Turret` / `ShotSolver` |
| Encoder wrap at 3.2 V instead of 3.3 V (the top end of the ELC output is noisy) | Meta `ENCODER_WRAP_VOLTAGE` | `Turret` (measure on ours) |
| Lookup table by distance for RPM, flight time, and feed power | Meta `LookUpTable` | `ShotTable` |
| Bang-bang while firing so RPM recovers between balls | Meta `Shooter.update` | `Shooter` |
| PIDF + kV with battery-voltage compensation, voltage read cached (not every loop) | Ditto `Shooter`, `VoltageCompensatedMotorGroup` | `Shooter` |
| Latency model: lead the turret by its response time, predict the release pose | Ditto `ShotPlanner` | `ShotSolver` layer 3 |
| Fresh one-step acceleration + light smoothing, stop-clamped prediction (no fake reversal when braking) | Ditto `ShotPlanner` and memory notes | `ShotSolver` layer 3 |
| Bulk caching MANUAL, one clear per loop | both | `Robot` |
| Only write motors and servos when the value changes | both | `CachedMotor`, `CachedServo`, `CachedCRServo` |
| Never call `getCurrent()` in the match loop (separate hub round-trip, ~2-3 ms each) | Ditto memory notes | everywhere |
| Telemetry toggle + send at most every ~200 ms | both | `Robot` |
| Save pose to a static every loop, read it at teleop start | Meta `Storage` (better than Ditto's save-on-stop: survives a crash or restart) | `RobotState` |
| Relocalize button that snaps pose to a known spot | both | drive controls |
| Rumble once on full, once when the shot clears | both | `Feedback` |
| Debounced break beams, "sustained beam = full" fallback | Ditto `Storage` | `Storage` |
| Alliance chosen in init, carried from auto to teleop | Ditto | `RobotState` |
| Build only the subsystems a test OpMode needs | Ditto `SubsystemConfig` | `Robot` constructor |
| Offline projectile solver to seed the RPM table | Ditto `LutTuningSolver` | test OpMode |

### What we don't take

- **Ditto's firing gates.** They deliberately fire without waiting for the flywheel to reach speed, because the hood angle is solved from the current RPM. We have no hood: RPM is the only adjustment, so **we wait for RPM** before every ball.
- **Ditto's wrap counting and Meta's wire limits.** Our encoder is on the turret itself and the turret spins forever, so a single-turn absolute reading is the full story.
- **A 900-line robot class** (Ditto `MyRobot`) and **25 copy-pasted autos** (both teams). Keep subsystems small; build autos from shared pieces.
- **Statics everywhere for tuning that also hold state.** Panels-tunable constants are fine as statics; runtime state is not.
- **Commented-out code piles** (Meta `RobotTeleop`). Delete; git remembers.

## Driving (driver 1, gamepad1)

Pedro 3.x has what we need built in: `ManualDrive.fieldCentric(...)`, `ManualDrive.headingLock(...)`, and `ManualDrive.driveOrHold(...)` (holds position when the sticks are released, so a defender can't push us while we shoot). Brake mode on.

| Input | Action |
|---|---|
| Left stick | Translate, field-centric (stick up = away from the driver, flipped per alliance) |
| Right stick X | Rotate |
| Stick shaping | Deadband + cubic blend (`util/DriveCurve`), applied to the stick's length so diagonals keep their angle |
| Sticks released | Hold pose (`driveOrHold`), so the robot resists pushing while firing |
| **RB (hold)** | **Shoot** at the upward CELL (see "Firing") |
| RT (hold) | Pass: low-power lob to the pass target. **On hold** until the variable hood; does nothing for now |
| **LB** | **Flower intake** down/up toggle |
| **LT (hold)** | **Unjam**: run the rail and intake backwards (`Robot.unjamHeld`) |
| **D-pad right** | Relocalize: **right** corner, intake facing the driver |
| **D-pad left** | Relocalize: **left** corner, intake facing the driver |
| **Circle** (B) | Relocalize: **right** corner, intake facing the side wall |
| **Square** (X) | Relocalize: **left** corner, intake facing the side wall |

"Left" and "right" are as the driver sees them, standing in our ALLIANCE AREA. Corner poses are in "Field coordinates" below.

**Driver 2 (gamepad2)** does what's easy to watch from the side:

| Input | Action |
|---|---|
| D-pad left/right | Turret trim +2° / -2° (left = counter-clockwise) |
| D-pad up/down | RPM trim ±25 |
| Options | Telemetry details on/off (loop time always shows) |

**Target cell is automatic:** the robot aims at the cell on its own half of the field (far half = far cell, audience half = audience cell), with a small band around the center line so it doesn't flicker (`Field.CELL_SWITCH_BAND`).

**Init:** D-pad up on either gamepad toggles alliance; both gamepads light up in its color. Telemetry shows alliance and the starting pose (saved from the last OpMode, or the right corner if none).

**Driver feedback (no lights on the robot, so use the gamepad):**

- Gamepad LED color by ball count (`gamepad1.setLedColor`, PS controllers only): 0-1 red (can't shoot), 2-3 yellow, 4 green.
- Short rumble on reaching 4 balls.
- Short rumble when the last ball clears, meaning "drive off".

Trims reset on relocalize (Meta does this), since trims mostly cover up pose drift.

## Firing

Two modes, same mechanics, different target:

| | Shoot (RB) | Pass (RT, on hold) |
|---|---|---|
| Turret aims at | Upward CELL of our HIVE (with SOTM lead) | `Field.PASS_TARGET_RED_X/Y`, a field point |
| RPM | `ShotTables.SHOT_RPM` + trim | `ShotTables.PASS_RPM`, low power, separate table |
| Arc | Must clear into an opening 53.5-65.6 in. up | Lands on the floor; only distance matters |
| Range limit | Distance must be inside `SHOT_RPM`'s rows (20-110 in., which covers every spot on the field) | Distance must be inside `PASS_RPM`'s rows (24-165 in., the whole field). Stops the robot guessing an RPM for a distance nobody measured; a ball lobbed out of the field is a MAJOR FOUL (G405) |

Hold the button. A burst can only **start** with 2 or more balls stored (beam A and beam B both blocked), then keeps going while the button is held. If both buttons are held, Shoot wins. All of this is in `robot/FireControl.java`.

**One ball at a time.** Every ball goes through three steps:

1. **Wait** until the shot is ready and has stayed ready for `READY_HOLD_MS` (40 ms), so one lucky reading can't fire.
2. **Feed** until the flywheel's RPM dips, which means the ball hit the wheel. No dip within `FEED_TIMEOUT_MS` counts as a misfeed (shown in telemetry).
3. **Recover**: stop the rail for `RECOVER_MS`, then back to Wait. The flywheel must be back at speed before the next ball.

Ready means all of:

1. The alliance is picked and the pose is trusted (see "Stale pose").
2. The target is inside the table's distance range.
3. `|flywheel RPM - target RPM| < Shooter.RPM_TOLERANCE`.
4. `|turret error| < Turret.ON_TARGET_DEG`.

The door opens on the first ready moment and stays open for the rest of the burst; the rail is what pauses, so the door never closes on a ball. While firing, the flywheel uses bang-bang (full power below `target - BOOST_BELOW_RPM`, from Meta) to recover between balls as fast as possible.

### Stale pose

Saved state lives in static fields, which survive until the robot app restarts, possibly several matches later.

- The pose is saved every loop with a timestamp. Teleop only uses it if it's under 60 s old (covers auto into teleop, or a quick restart). Otherwise the pose is **untrusted** and nothing fires until a corner relocalize; a warning stays on the Driver Station until then.
- The alliance is picked in every init (teleop and auto) with **D-pad up** on either gamepad, any time during init (`opmodes/AllianceSelector`). Both gamepads light up red or blue, or **white** when nothing is picked.
- Right after auto (saved pose under 60 s old), teleop keeps auto's alliance, so it starts already picked. Otherwise it starts white, and nothing fires until it's picked. This works as long as autos use `Robot.update()`, which saves the pose every loop.

## Intake and storage

Rail slots, as in `ROBOT.md`: slot 1 at the door (beam A), slot 2 (beam B), slot 3 (no beam), slot 4 at the intake end (intake beam, only stays blocked when all 4 are stored).

- **Rail runs automatically** whenever we are not full and not firing. Intake and transfer motors both run.
- **Transfer drops to hold power** once beam A has been blocked for a short time and the door is closed, so it doesn't stall against the door (Meta's `BEAM3_TRANSFER_OFF_MS` idea).
- **Full** = intake beam blocked continuously for `FULL_HOLD_MS` (a ball passing through on entry only blocks it briefly). Intake stops when full.
- **Count:** beams give 0, 1, 2, then "2 or 3" (slot 3 has no beam), then 4. Capacity is 4. The firing rule only needs "at least 2" (beams A and B), and the auto-stop only needs "4", so slot 3 doesn't need its own beam.
- Beams are debounced over a few loops (Ditto's `ModeSmoother`).
- **Flower intake:** LB toggles down/up. It also raises itself when we reach 4, so the driver can turn straight to shoot.

## Turret

CR Axon, geared 1:2 up (the turret turns twice per servo turn), ELC absolute encoder on the turret, analog output only, infinite rotation.

- **Angle:** `wrap360(voltage / WRAP_VOLTAGE * 360 - ZERO_DEG)`. Measure `WRAP_VOLTAGE` on ours (Meta found 3.2 V better than 3.3 V) and `ZERO_DEG` with the turret facing robot-forward.
- **Error:** shortest way around, `IEEEremainder(target - current, 360)`. No limits, no wrap counting.
- **Control:** PID on error + ramped static feedforward (`kS * sign(error) * min(|error| / ramp, 1)`, which avoids chatter near zero) + a feedforward on the target's rate of change. That last term mostly cancels robot rotation: when the robot turns at ω, the turret has to turn at -ω.
- **Gain sets:** "far" (large error, move fast) and "close" (small error, settle), switching with hysteresis (Meta).
- Power capped by `TURRET_MAX_POWER`; output voltage-compensated; written only when it changes.
- The turret is geared up, so torque and resolution are lower and `kS` will matter. Tune it first.

## Shooter

Two coupled motors, one encoder used, fixed launch angle, no hood.

- PIDF with kV (velocity feedforward) and kS, voltage-compensated; voltage read at most every ~250 ms.
- Target RPM from `ShotTable.rpm(distance)`; bang-bang while firing (see above).
- **Idle:** spins at the table RPM for the current distance whenever we hold a ball, so it's already at speed when RT is pressed. Off with 0 balls, or by the A override.
- Built-in velocity is in ticks/s; convert using 28 ticks/rev and the 1:1 ratio.

## Shot solver (SOTM), in layers

The HIVE is in the middle of the field and the upward cell's opening is 53.5-65.6 in. above the tiles (aim at about 59.5 in.), so we will shoot from all around it, roughly 20 to 95 in. away. Each layer is a flag in Panels, so we can always fall back to a simpler one on the field.

1. **Static.** Aim the turret at the goal from the turret's field position (robot pose + turret offset; ours is at the chassis center, wedges excluded). Distance → RPM. This must be solid before anything else.
2. **Velocity lead (Meta's version, slightly improved).** Aim at `virtualGoal = goal - robotVelocity * flightTime(distance)` and use the virtual goal's distance for RPM. Iterate 2-3 times, because moving the goal changes the distance, which changes the flight time. Flight time comes from the table, measured with slow-motion video.
3. **Latency (Ditto's model).** Predict the pose forward by `TURRET_LATENCY` for aiming and by `RELEASE_LATENCY` for the shot. Use a fresh one-step acceleration with light smoothing, and stop-clamp the prediction so a braking robot coasts to zero instead of "reversing". Our own PID should make the turret latency smaller than Ditto's 0.135 s servo delay. Measure ours.
4. **RPM tracking while moving.** Feedforward on the target RPM's rate of change (Ditto's `FLYWHEEL_ACCEL_KFF`). With no hood, the RPM gate in "Firing" is the safety net.

Telemetry for tuning (Ditto's set): virtual goal, lead in x/y, flight time, raw vs compensated velocity, estimated acceleration.

## Field coordinates

Source: Competition Manual Section 9 (TU02) and its figures. The manual says drawings are good to about ±1 in. Values marked *computed* are derived from the figures, not printed in the manual; check them on a real field.

**Frame (Pedro convention, inches):** stand where the audience stands and look at the field.

- (0, 0) = near-left corner: red alliance wall × audience wall.
- +x toward the blue alliance wall, +y away from the audience. Field is 144 × 144.
- Heading 0 = facing +x, counter-clockwise positive (Pedro's unit-circle convention).
- Red ALLIANCE AREA is on the left (x < 0), blue on the right (x > 144).
- The field is symmetric under a 180° spin about (72, 72): blue = `(144 - x, 144 - y, heading + 180°)`. Enter red values only; compute blue.

**HIVE** (Section 9.6, Figures 9-8 to 9-10):

| | Value |
|---|---|
| Frame base | 49.46 × 38.95 in., centered on (72, 72) |
| Pivot height | 43.95 in. |
| Red HIVE x / blue HIVE x | 59.25 / 84.75 (25.5 in. center to center, red on the left) |
| Cells per HIVE | 2, one toward the far wall, one toward the audience; centers 30.88 in. apart along y |
| Tilt when one cell is up | 30° |
| Upward cell center, y | *computed:* far cell ≈ 85.4, audience cell ≈ 58.6 (72 ± 15.44·cos 30°) |
| Upward cell opening height | bottom 53.5 in., top 65.6 in. |
| Opening size | about 20 wide × 14 tall × 12 deep |

**AprilTags** (on the bottom of each cell, facing down): red far 30-33, red audience 34-37, blue audience 38-41, blue far 42-45.

**Other elements** (red values; blue by 180° spin):

| Element | Position |
|---|---|
| FLOWERS (4, shared) | centered on (48, 144), (0, 48), and by spin (96, 0), (144, 96); about 6.6 wide × 6 out from the wall |
| Red LOADING ZONE | x 0-11, y 96-120 |
| Red GARDEN | x 0-24, y 0-2 (the near-left corner) |

### Relocalize poses

Robot center = turret center (chassis center, wedges excluded). From the CAD bottom view: half-width over the outer frame rails 149 mm = **5.87 in.**; turret center to intake front 203 mm = **7.99 in.** (Both still to be checked with a tape.)

These assume the **intake** is what touches the wall when it faces a wall, and the **chassis side** touches the other wall. If something else sticks out further (wedges, flower intake, turret, bumpers), measure the real contact distances on the robot and change the two numbers.

From the driver's view: red's right corner is the audience corner (y = 0), left is the far corner (y = 144). For blue it's the opposite.

| Button | Corner | Intake faces | Red pose (x, y, heading) | Blue pose |
|---|---|---|---|---|
| D-pad right | right | driver | (7.99, 5.87, 180°) | (136.01, 138.13, 0°) |
| D-pad left | left | driver | (7.99, 138.13, 180°) | (136.01, 5.87, 0°) |
| Circle | right | side wall | (5.87, 7.99, 270°) | (138.13, 136.01, 90°) |
| Square | left | side wall | (5.87, 136.01, 90°) | (138.13, 7.99, 270°) |

The red right corner is the red GARDEN corner. Its tape is only 2 in. deep, so the robot fits, but its POLLEN must already be picked up.

## Loop and structure

Target loop time: **10-15 ms** (Ditto was at 40 ms). Order each loop:

1. Clear the bulk cache (MANUAL mode, one bulk read per hub).
2. Read inputs (gamepad edges).
3. `follower.update()`.
4. Read sensors: beams, turret encoder, flywheel velocity, cached voltage.
5. Shot solver.
6. Subsystems compute and write (cached writes only).
7. `Scheduler.execute()` (Ivy) for sequences: fire, auto routines.
8. Save pose to `RobotState`.
9. Telemetry, if on and ≥ 200 ms since the last send.

Layout under `teamcode/` (scaffolded):

```
field/        Alliance (red/blue flip), Cell, Corner, Field (all field coordinates)
robot/        Robot (owns everything, loop order), FireControl (the 2-ball burst rule),
              Feedback (rumble + light), RobotState (alliance, cell, last pose), HardwareNames
subsystems/   Drive, Storage, Rail, Door, FlowerIntake, Shooter, Turret
shot/         ShotSolver (layers 1-3), ShotSolution, ShotTables (distance -> RPM, flight time, feed power)
util/         Angles, LookupTable, Debouncer, CachedMotor, CachedServo, CachedCRServo, VoltageCache, BulkReads
opmodes/      MainTeleOp
opmodes/test/ StorageTest, TurretTest, ShooterTest, ServoPositionFinder
```

Every subsystem takes a `HardwareMap` and works alone, so test OpModes build just the parts they test instead of the whole `Robot`. Constants that need tuning are `public static` on `@Configurable` classes for Panels. What to tune and in what order: `TUNING_GUIDE.md`, and the generated `TODO_LIST.md`.

Not built yet: SOTM layer 4 (RPM feedforward while moving), acceleration in the prediction, autos, and an offline projectile solver for seeding the RPM table.

## Dependencies

Panels `com.bylazar:fullpanels:1.0.17` from `https://mymaven.bylazar.com/releases` is added. The APK builds with it on SDK 12 and Pedro 3.x; it hasn't been run on the robot yet. Its Pedro field-drawing preset may target Pedro 2.x, so the scaffold doesn't use it.

## Build order

Each step is tested on the robot before moving on.

1. Hardware config names + port map (fills the TBDs in `ROBOT.md`), Pedro tuners, `Constants.create` filled in.
2. `Robot` skeleton, loop timing, cached writes, Panels, telemetry toggle. Check loop time with everything idle.
3. Field-centric drive, stick curve, hold, relocalize, pose saved and carried over.
4. Storage beams + auto rail + door + flower intake. `BeamTest` first.
5. Turret: encoder reading and zero, then `TurretTuner` (kS, then far gains, then close gains).
6. Shooter: PIDF tune, then fill `ShotTable` by shooting from measured distances.
7. Static shooting (SOTM layer 1) + hold-to-fire.
8. SOTM layers 2, 3, 4, tuned in that order.
9. Autos built from shared pieces.

## Open questions

Answered: 4-ball capacity, ≥ 2 balls to shoot, button map, the four corner poses, no endgame.

Still open:

- **Pass (on hold until the variable hood).** Pick the pass target and fill `PASS_RPM`, then set `FireControl.PASS_ENABLED = true`.
- **Wall contact distances** for the relocalize poses (measure on the robot, see above).
- **Upward cell position**: check the computed y values (≈ 85.4 / 58.6) on a real field, or with a tape measure on the field drawing.
- Hub and port for every device (`ROBOT.md` TBDs).
- Pinpoint pod offsets.
- Camera: if one is added, the HIVE tags face down from under the cells, so it has to look up. It could also tell which cell is up, instead of the Triangle button.
