# TeleOp and robot code plan

The BIOBUZZ robot's code now exists under `TeamCode/.../teamcode/`, and this file describes it and why it is built this way. Hardware facts are in `ROBOT.md`; this file covers how the code is built and driven. When the code and this file disagree, the code is right: fix the file. The tunable numbers are placeholders until they're measured on the robot (`TODO_LIST.md` lists what's left), and `Main TeleOp` can't start until `Constants.create` is filled in from the Pedro tuners (the `Robot` constructor throws while it returns `null`).

Decisions already made:

- Field-centric drive, one driver does everything (gamepad1).
- Shoot-on-the-move (SOTM) from the start, built in layers so each layer is testable alone.
- Hold-to-fire on RB (score at the HIVE). In TeleOp it needs at least 2 balls stored to start; it never starts with 1 (auto can allow 1).
- RT pass (lob balls to a spot on the floor) is built but **on hold** until the variable hood (`FireControl.PASS_ENABLED = false`).
- Rail intake runs automatically; LB toggles the flower intake.
- Four relocalize buttons, one per corner pose on our alliance wall.
- No endgame.
- Panels for live tuning.
- Camera undecided, so localization is Pinpoint only, built so a camera can be added later.
- Command scheduling with Ivy (already a dependency), on Pedro Pathing 3.x. Only autos use it (`RobotCommands`); TeleOp calls `Scheduler.execute()` every loop but schedules nothing.

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
| CR turret control (kP + kD, no I term) on the analog encoder, shortest-path error, close/far gain sets with hysteresis, ramped static feedforward, power cap, cached writes | Meta `Turret`, `PIDController` | `Turret` |
| Lead the turret by robot angular velocity | Meta `TURRET_ANG_VEL_LEAD_SEC` | `Turret` (`KV`, a feedforward on the target's rate of change) and `ShotSolver` (`PREDICT` turns the heading forward by `omega * TURRET_LATENCY_SEC`) |
| Encoder wrap at 3.2 V instead of 3.3 V (the top end of the ELC output is noisy) | Meta `ENCODER_WRAP_VOLTAGE` | `Turret.ENCODER_FULL_TURN_VOLTS` (measure on ours) |
| Lookup table by distance for RPM, flight time, and feed power | Meta `LookUpTable` | `ShotTables` |
| Bang-bang while firing so RPM recovers between balls | Meta `Shooter.update` | `Shooter` |
| kS + kV feedforward + kP (no I or D) with battery-voltage compensation, voltage read cached (not every loop) | Ditto `Shooter`, `VoltageCompensatedMotorGroup` | `Shooter`, `VoltageCache` |
| Latency model, simplified: solve from where the robot will be after the turret's response time | Ditto `ShotPlanner` | `ShotSolver` layer 3 (`PREDICT`) |
| Bulk caching MANUAL, one clear per loop | both | `Robot` (`BulkReads`) |
| Only write motors and servos when the value changes | both | `CachedMotor`, `CachedServo`, `CachedCRServo` |
| Never call `getCurrent()` in the match loop (separate hub round-trip, ~2-3 ms each) | Ditto memory notes | everywhere |
| Telemetry toggle + send at most every ~200 ms | both | `MainTeleOp` |
| Save pose to a static every loop (trusted poses only), read it at teleop start | Meta `Storage` (better than Ditto's save-on-stop: survives a crash or restart) | `RobotState` |
| Relocalize button that snaps pose to a known spot | both | drive controls |
| Rumble when storage is full (ours is a soft constant rumble for as long as it stays full; there is no buzz when the shot clears) | both | `Feedback` |
| Debounced break beams, "sustained beam = full" fallback | Ditto `Storage` | `Storage` |
| Alliance chosen in init, carried from auto to teleop | Ditto | `RobotState` |
| Build only the subsystems a test OpMode needs | Ditto `SubsystemConfig` | test OpModes build their own subsystems; `Robot` builds all of them |
| Offline projectile solver to seed the RPM table | Ditto `LutTuningSolver` | not built yet |

### What we don't take

- **Ditto's firing gates.** They deliberately fire without waiting for the flywheel to reach speed, because the hood angle is solved from the current RPM. We have no hood: RPM is the only adjustment, so **we wait for RPM** before every ball.
- **Ditto's wrap counting and Meta's wire limits.** Our encoder is on the turret itself and the turret spins forever, so a single-turn absolute reading is the full story.
- **Ditto's acceleration estimate and stop-clamped prediction.** Layer 3 assumes the robot keeps its current velocity. Acceleration isn't built (see "Not built yet").
- **A 900-line robot class** (Ditto `MyRobot`) and **25 copy-pasted autos** (both teams). Keep subsystems small; build autos from shared pieces.
- **Statics everywhere for tuning that also hold state.** Panels-tunable constants are fine as statics; runtime state is not.
- **Commented-out code piles** (Meta `RobotTeleop`). Delete; git remembers.

## Driving (driver 1, gamepad1)

Pedro 3.x has what we need built in: `ManualDrive.fieldCentric(...)` and `ManualDrive.driveOrHold(...)` (holds position when the sticks are released, so a defender can't push us while we shoot). `ManualDrive.headingLock(...)` is there too, but we don't use it. Brake mode on.

| Input | Action |
|---|---|
| Left stick | Translate, field-centric (stick up = away from the driver, flipped per alliance) |
| Right stick X | Rotate |
| Stick shaping | Deadband + cubic blend (`util/DriveCurve`), applied to the stick's length so diagonals keep their angle. Turning has its own blend (`Drive.TURN_CUBIC`) |
| Sticks released | Hold pose (`driveOrHold`) once the robot has slowed below `Drive.HOLD_BELOW_SPEED`, so the robot resists pushing while firing |
| **RB (hold)** | **Shoot** at the upward CELL (see "Firing") |
| RT (hold) | Pass: low-power lob to the pass target. **On hold** until the variable hood; does nothing for now |
| **LB** | **Flower intake** down/up toggle |
| **LT (hold)** | **Unjam**: run the rail and intake backwards (`Robot.unjamHeld`) |
| **D-pad right** | Relocalize: **right** corner, intake facing the driver |
| **D-pad left** | Relocalize: **left** corner, intake facing the driver |
| **Circle** (B) | Relocalize: **right** corner, intake facing the side wall |
| **Square** (X) | Relocalize: **left** corner, intake facing the side wall |

RT and LT count as pressed once pulled past `MainTeleOp.TRIGGER_THRESHOLD` (0.5, editable in Panels). A relocalize needs an alliance picked; without one the buttons do nothing.

"Left" and "right" are as the driver sees them, standing in our ALLIANCE AREA. Corner poses are in "Field coordinates" below.

**Driver 2 (gamepad2)** does what's easy to watch from the side:

| Input | Action |
|---|---|
| D-pad left/right | Turret trim +2° / -2° (left = counter-clockwise) |
| D-pad up/down | RPM trim ±25 |
| Options | Telemetry details on/off (loop time and driver warnings always show) |

The trim steps (`TURRET_TRIM_STEP_DEG`, `RPM_TRIM_STEP`), the trigger threshold and the telemetry interval are `public static` on `MainTeleOp`, so they are editable in Panels.

**Target cell is automatic (there is no button for it):** the robot aims at the cell on its own half of the field (far half = far cell, audience half = audience cell). Within a small band around the center line (`Field.CELL_SWITCH_BAND`, 6 in.) it keeps whichever cell it had, so it doesn't flicker.

**Init:** D-pad up on either gamepad toggles alliance (the first press picks red); both gamepads light up in its color. Telemetry shows the alliance and the start pose: `from the last OpMode` plus the pose if a recent trusted one is saved, otherwise "unknown: relocalize in a corner before shooting". Switching alliance drops a pose saved by auto (it is in the other alliance's frame), so the screen goes back to "unknown" and the drivers relocalize. See "Stale pose".

**Driver feedback (no lights on the robot, so use the gamepad and the Driver Station screen).** Everything below is in `robot/Feedback.java` and `robot/Warning.java`.

- Gamepad LED color by ball count (`gamepad1.setLedColor`, PS controllers only): 0-1 red (can't shoot), 2-3 yellow, 4 green.
- Rumble on driver 1's controller. There are only two kinds, so each always means the same thing:
  - **Warning:** 3 strong pulses when a warning appears. The same warning only buzzes again after `Feedback.WARNING_COOLDOWN_MS` (3 s). "Target out of range" never buzzes: it's normal while driving, so it stays on the screen only.
  - **Full:** a soft constant rumble for as long as storage is full. It stops when balls leave. A warning buzz plays over it, then it comes back.
  - There is no buzz when a burst finishes. Strengths, pulse timing and the cooldown are `public static` on `Feedback`, so they're editable in Panels.
- Warnings: a `!!` line on the Driver Station and Panels screens, shown even when telemetry details are off. The enum order is the order the lines show.

| Warning | Shows when | Buzzes |
|---|---|---|
| `NO_ALLIANCE` | No alliance was picked in init. Restart the OpMode and pick one | yes |
| `POSE_UNKNOWN` | An alliance is picked but the pose isn't trusted. Relocalize in a corner | yes |
| `POSE_NAN` | The pose or velocity is NaN (odometry glitch). Nothing is aimed or saved until it clears | yes |
| `OUT_OF_RANGE` | The target is outside the shot table's distance range | no |
| `SHOOTER_OFF` | The shooter watchdog tripped (see "Shooter"). It stays off until the OpMode restarts | yes |
| `TURRET_OFF` | The turret watchdog tripped (see "Turret"). It stays off until the OpMode restarts | yes |
| `SHOT_TABLE_ORDER` | A Panels edit put a shot table's distances out of order, so aiming would use wrong numbers | yes |

Trims reset on relocalize (Meta does this), since trims mostly cover up pose drift.

## Firing

Two modes, same mechanics, different target:

| | Shoot (RB) | Pass (RT, on hold) |
|---|---|---|
| Turret aims at | Upward CELL of our HIVE (with SOTM lead when `LEAD` is on) | `Field.PASS_TARGET_RED_X/Y`, a field point |
| RPM | `ShotTables.SHOT_RPM` + trim | `ShotTables.PASS_RPM`, low power, separate table |
| Arc | Must clear into an opening 53.5-65.6 in. up | Lands on the floor; only distance matters |
| Range limit | Distance must be inside `SHOT_RPM`'s first and last rows (20-110 in. for now, until the table is measured) | Distance must be inside `PASS_RPM`'s rows (24-165 in., the whole field). Stops the robot guessing an RPM for a distance nobody measured; a ball lobbed out of the field is a MAJOR FOUL (G405) |

Hold the button. A burst can only **start** with 2 or more balls stored (beam A and beam B both blocked, or storage full), then keeps going while the button is held. That is the TeleOp rule; auto sets `Robot.singleBallBursts` so a burst can start with 1. If both buttons are held, Shoot wins. All of this is in `robot/FireControl.java`.

Releasing the button ends the burst right away, with one exception: **releasing mid-feed lets that ball finish** (shot seen, or `FEED_TIMEOUT_MS`), then the burst ends, so the door never closes on a ball.

**One ball at a time.** Every ball goes through three steps:

1. **Wait** until the shot is ready and has stayed ready for `READY_HOLD_MS` (40 ms), so one lucky reading can't fire. Then the door opens (if it isn't already) and the feed starts once it is fully open (`Door.OPEN_TIME_MS`).
2. **Feed** until the flywheel's RPM dips (`SHOT_DIP_RPM` below the target the feed started with), which means the ball hit the wheel. No dip within `FEED_TIMEOUT_MS` counts as a misfeed (shown in telemetry), and it goes back to Wait.
3. **Recover**: stop the rail for `RECOVER_MS`, then back to Wait. The flywheel must be back at speed before the next ball.

Ready means all of:

1. The alliance is picked, the pose is trusted (see "Stale pose") and it isn't NaN.
2. The target is inside the table's distance range.
3. `|flywheel RPM - target RPM| < Shooter.RPM_TOLERANCE`.
4. `|turret error| < Turret.ON_TARGET_DEG`, and the turret watchdog hasn't tripped.

The door stays open for the rest of the burst; the rail is what pauses, so the door never closes on a ball. After the burst the door closes, and the rail stays stopped until it is fully closed (`Door.CLOSE_TIME_MS`), or the next ball would reach it half open. Rail priority each loop (`Robot.update`): unjam (LT), then feed, then stopped (while firing or the door isn't fully closed), then collect. While firing, the flywheel uses bang-bang (full power below `target - BOOST_BELOW_RPM`, from Meta) to recover between balls as fast as possible.

### Stale pose

Saved state lives in static fields, which survive until the robot app restarts, possibly several matches later.

- The pose is saved every loop with a timestamp, but only a **trusted** pose (auto's start pose, a recent saved pose, or a relocalize) that isn't NaN. A guessed pose is never saved, or a quick restart would restore it as trusted. Teleop only uses the saved pose if it's under 60 s old (covers auto into teleop, or a quick restart). Otherwise the pose is **untrusted** and nothing fires until a corner relocalize; the `POSE_UNKNOWN` warning stays on the Driver Station (and buzzes when it appears) until then. With an alliance picked but no saved pose, teleop puts the robot at the right-corner pose as a guess, marked untrusted. With neither, it sets no pose at all.
- The alliance is picked in every init (teleop and auto) with **D-pad up** on either gamepad, any time during init (`opmodes/AllianceSelector`). Both gamepads light up red or blue, or **white** when nothing is picked.
- Switching alliance in init drops the saved pose (it is in the other alliance's frame), so the drivers relocalize.
- Right after auto (saved pose under 60 s old), teleop keeps auto's alliance, so it starts already picked. Otherwise it starts white, and nothing fires until it's picked. This works as long as autos use `Robot.update()`, which saves the pose every loop.
- An alliance can't be picked after Start: the `NO_ALLIANCE` warning says to restart the OpMode. Until one is picked, driving works (field-centric as red), but nothing fires and relocalize does nothing.

## Intake and storage

Rail slots, as in `ROBOT.md`: slot 1 at the door (beam A), slot 2 (beam B), slot 3 (no beam), slot 4 at the intake end (intake beam, only stays blocked when all 4 are stored).

- **Rail collects automatically** whenever we are not unjamming, feeding or firing, and the door is fully closed. Intake and transfer motors both run, except that the intake motor stops when full.
- **Transfer drops to a gentle hold power** (`Rail.TRANSFER_HOLD_POWER`) as soon as beam A is blocked (debounced), so it doesn't stall against the closed door (the idea from Meta's `BEAM3_TRANSFER_OFF_MS`; we hold gently instead of stopping).
- **Full** = intake beam blocked continuously for `FULL_HOLD_MS` (a ball passing through on entry only blocks it briefly). Intake stops when full.
- **Count:** beams give 0, 1, "2-3" (slot 3 has no beam, so 2 and 3 look the same), or 4. Capacity is 4. The firing rule only needs "at least 2" (beams A and B, or full), and the auto-stop only needs "4", so slot 3 doesn't need its own beam.
- Beams are debounced over a few loops (`Storage.DEBOUNCE_LOOPS`; Ditto's `ModeSmoother`).
- **Unjam:** LT runs the rail and intake backwards and wins over everything else the rail does.
- **Flower intake:** LB toggles down/up. It also raises itself when we reach 4, so the driver can turn straight to shoot.

## Turret

CR Axon, geared 1:2 up (the turret turns twice per servo turn), ELC absolute encoder on the turret, analog output only, infinite rotation.

- **Angle:** `wrapDegrees(voltage / ENCODER_FULL_TURN_VOLTS * 360 - ENCODER_ZERO_DEG)`, wrapped to ±180°, with `ENCODER_REVERSED` flipping the sign before the zero is subtracted. 0° = shooter pointing out the robot's front, counter-clockwise positive. Measure `ENCODER_FULL_TURN_VOLTS` on ours (Meta found 3.2 V better than 3.3 V, so that is the default) and `ENCODER_ZERO_DEG` with the turret facing robot-forward.
- **Error:** shortest way around, `IEEEremainder(target + trim - current, 360)`. No limits, no wrap counting.
- **Control:** `kP * error + kD * (rate of change of the error)` (no I term) + ramped static feedforward (`kS * sign(error) * min(|error| / KS_RAMP_DEG, 1)`, which avoids chatter near zero) + `KV` times the target's rate of change (starts at 0). That last term mostly cancels robot rotation: when the robot turns at ω, the turret has to turn at -ω.
- **Gain sets:** "far" (large error, move fast) and "close" (small error, settle), switching at `CLOSE_ZONE_DEG` with a hysteresis band (Meta).
- Power capped by `MAX_POWER`; output voltage-compensated; written only when it changes.
- **Watchdog:** if the turret runs at full power (95% of `MAX_POWER` or more) for `WATCHDOG_MS` and the error hasn't shrunk by at least 10°, power is cut until the OpMode restarts (the `TURRET_OFF` warning). It catches an unplugged or stuck encoder, and `SERVO_REVERSED` and `ENCODER_REVERSED` disagreeing. A window where the target moved more than 20° (the robot is spinning) doesn't count. While tripped, `Turret.onTarget()` is false, so nothing fires.
- The turret is geared up, so torque and resolution are lower and `kS` will matter. Tune it first.

## Shooter

Two coupled motors, one encoder used, fixed launch angle, no hood.

- Power = `kS + kV * target + kP * (target - RPM)` (kV is the velocity feedforward; no I or D), times the battery-voltage compensation; voltage read at most every 250 ms (`VoltageCache`).
- Target RPM is the `ShotTables.SHOT_RPM` value for the current distance (`LookupTable.at`, called from `ShotSolver`), plus driver 2's RPM trim; bang-bang while firing (see above).
- **Pre-spin:** the flywheel runs at the table RPM for the current distance whenever storage isn't empty or a burst is running, so it's already at speed when RB is pressed, and it isn't cut while the last ball is still going through. Off with 0 balls (and no burst), and off when nothing can be aimed (no alliance, or a NaN pose).
- **Watchdog:** if the power stays above `WATCHDOG_POWER` (0.5) for `WATCHDOG_MS` (500) while the signed RPM is under `WATCHDOG_MIN_RPM` (100), the shooter shuts off until the OpMode restarts (the `SHOOTER_OFF` warning). It catches an unplugged encoder (RPM reads 0) and a wrong `ENCODER_REVERSED` (RPM reads negative), where the controller would hold full power forever.
- Built-in velocity is in ticks/s; convert using 28 ticks/rev and the 1:1 ratio.

## Shot solver (SOTM), in layers

The HIVE is in the middle of the field and the upward cell's opening is 53.5-65.6 in. above the tiles (aim at about 59.5 in.), so we will shoot from all around it, roughly 20 to 110 in. away (the `ShotTables.SHOT_RPM` range). Each layer is a flag in Panels, so we can always fall back to a simpler one on the field. `LEAD` and `PREDICT` both start **off**; layer 1 is always on. Passes use layer 1 only, since they land on the floor and don't need to be exact.

1. **Static.** Aim the turret at the goal from the turret's field position. Ours is at the chassis center (wedges excluded), so the robot pose is used as is, with no turret offset. Distance → RPM. This must be solid before anything else.
2. **Velocity lead (`LEAD`, Meta's version, slightly improved).** Aim at `virtualGoal = goal - robotVelocity * flightTime(distance)` and use the virtual goal's distance for RPM. Iterate `LEAD_ITERATIONS` (3) times, because moving the goal changes the distance, which changes the flight time. Flight time comes from the `SHOT_FLIGHT_TIME` table, measured with slow-motion video.
3. **Latency (`PREDICT`, a simpler version of Ditto's model).** The turret needs time to react, so solve from where the robot will be `TURRET_LATENCY_SEC` (0.08 s) from now, assuming it keeps its current velocity (x, y, and heading from the angular velocity). That one predicted pose is used for both the aim and the distance. There is no acceleration and no separate release latency. Our own kP + kD control should make the turret latency smaller than Ditto's 0.135 s servo delay. Measure ours.
4. **RPM tracking while moving (not built).** Feedforward on the target RPM's rate of change (Ditto's `FLYWHEEL_ACCEL_KFF`). With no hood, the RPM gate in "Firing" is the safety net.

Telemetry (with details on) is whatever `Robot.addTelemetry` prints: alliance and target cell, the fire state (step, shots, misfeeds, "(ready)"), aim distance and RPM, pose and follower mode, ball count and beams, rail, door and flower intake state, shooter RPM, power and trim, and turret angle, error, power and gain set. The virtual goal, lead, flight time and velocity/acceleration estimates are not shown yet.

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

Target loop time: **10-15 ms** (Ditto was at 40 ms). Order each loop of `MainTeleOp`:

1. Read inputs (gamepad edges): sticks go to `Drive`, buttons set the `Robot` fields, trims, relocalize.
2. `Robot.update()`, in this order:
   1. Clear the bulk cache (MANUAL mode, one bulk read per hub).
   2. `follower.update()`.
   3. Read the beams (`Storage`).
   4. Shot solver: pick the target cell, then turret angle, RPM and feed power.
   5. `FireControl` and the decisions: door, rail, flower intake, shooter and turret targets.
   6. Subsystems write (cached writes only). The turret encoder and flywheel velocity are read here, inside `Turret.update()` and `Shooter.update()`, so the decisions in step 5 use the last loop's values. Voltage is cached (`VoltageCache`).
   7. Save the pose to `RobotState` (only a trusted pose that isn't NaN).
3. `Scheduler.execute()` (Ivy). TeleOp schedules nothing, so this does no work there. Auto calls it before `Robot.update()`, so a command's changes apply in the same loop.
4. `Feedback.update()`: rumble and gamepad light.
5. Telemetry, if ≥ 200 ms since the last send: warnings and loop time always, the rest only when details are on.

Layout under `teamcode/`:

```
field/        Alliance (red/blue flip), Cell, Corner, Field (all field coordinates)
robot/        Robot (owns everything, loop order), FireControl (the burst rule: wait, feed, recover),
              Feedback (rumble + light), Warning (driver warnings), RobotState (alliance, cell, last pose),
              RobotCommands (Ivy commands for auto), HardwareNames
subsystems/   Drive, Storage, Rail, Door, FlowerIntake, Shooter, Turret
shot/         ShotSolver (layers 1-3), ShotSolution, ShotTables (distance -> RPM, flight time, feed power)
util/         Angles, LookupTable, Debouncer, DriveCurve, CachedMotor, CachedServo, CachedCRServo, VoltageCache, BulkReads
pedro/        Constants (builds the Follower), TeamTuning, Pedro tuners
opmodes/      MainTeleOp, AllianceSelector
opmodes/auto/ AutoTemplate
opmodes/test/ StorageTest, TurretTest, ShooterTest, ServoPositionFinder
```

Subsystems are built from a `HardwareMap` (`Drive` takes a `Follower` instead, and `Shooter` and `Turret` also take a `VoltageCache`) and work alone, so test OpModes build just the parts they test instead of the whole `Robot`. Constants that need tuning are `public static` on `@Configurable` classes for Panels. Panels can change a shot table's numbers but can't add or remove rows, and its edits are lost when the app restarts, so copy final numbers back into the code. What to tune and in what order: `TUNING_GUIDE.md`, and the generated `TODO_LIST.md`.

**Auto** is a template only (`opmodes/auto/AutoTemplate`, disabled until copied), built on `RobotCommands`, whose commands set the same `Robot` fields the gamepads set so auto and TeleOp share the aiming and firing logic. It picks the alliance in init (`AllianceSelector`) and sets the start pose with `Robot.setStartPose`, and `Robot.update()` saves the pose every loop, so TeleOp starts with auto's pose and alliance.

Not built yet: SOTM layer 4 (RPM feedforward while moving), acceleration in the prediction, real autos (only the template exists), and an offline projectile solver for seeding the RPM table.

## Dependencies

Panels `com.bylazar:fullpanels:1.0.17` from `https://mymaven.bylazar.com/releases` is added. The APK builds with it on SDK 12 and Pedro 3.x; it hasn't been run on the robot yet. Its Pedro field-drawing preset may target Pedro 2.x, so the code doesn't use it.

## Build order

Each step is tested on the robot before moving on.

1. Hardware config names + port map (fills the TBDs in `ROBOT.md`), Pedro tuners, `Constants.create` filled in.
2. `Robot` skeleton, loop timing, cached writes, Panels, telemetry toggle. Check loop time with everything idle.
3. Field-centric drive, stick curve, hold, relocalize, pose saved and carried over.
4. Storage beams + auto rail + door + flower intake. `StorageTest` first.
5. Turret: encoder reading and zero, then `TurretTest` (kS, then far gains, then close gains).
6. Shooter: tune kS, kV and kP in `ShooterTest`, then fill `ShotTables` by shooting from measured distances.
7. Static shooting (SOTM layer 1) + hold-to-fire.
8. SOTM layers 2, 3, 4, tuned in that order (layer 4 isn't built yet).
9. Autos built from shared pieces (`RobotCommands` and `AutoTemplate` exist; the real autos don't).

## Open questions

Answered: 4-ball capacity, ≥ 2 balls to shoot, button map, the four corner poses, no endgame.

Still open:

- **Pass (on hold until the variable hood).** Pick the pass target and fill `PASS_RPM`, then set `FireControl.PASS_ENABLED = true`.
- **Wall contact distances** for the relocalize poses (measure on the robot, see above).
- **Upward cell position**: check the computed y values (≈ 85.4 / 58.6) on a real field, or with a tape measure on the field drawing.
- Hub and port for every device (`ROBOT.md` TBDs).
- Pinpoint pod offsets.
- Camera: if one is added, the HIVE tags face down from under the cells, so it has to look up. It could also tell which cell is up, instead of picking the cell from the robot's position.
