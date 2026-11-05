# TeleOp controls and field reference

Button map, driver warnings, match rules and field coordinates. How each mechanism works is in the comment at the top of its class. Hardware is in `ROBOT.md`, tuning in `TUNING_GUIDE.md`, and how the code runs in `RUNDOWN.md`.

## Init

- **D-pad up** on either gamepad toggles alliance (the first press picks red). Both gamepads light up red or blue, or white when nothing is picked. An alliance can't be picked after Start.
- Right after auto (pose saved under 60 s ago), TeleOp keeps auto's alliance and pose. Otherwise the pose is unknown: driver 2 relocalizes in a corner before shooting. Switching alliance drops a saved pose, since it is in the other alliance's frame.
- Button presses made during init are thrown away at Start (`resetEdgeDetection()`), so a press in init can't relocalize, pick the up cell, move the flower intake or turn zone fire on in the first loop. In init only D-pad up (alliance) does anything.
- **Without a saved pose, set the robot down with the intake facing you.** TeleOp guesses that heading, and field-centric driving uses it right away. If the robot faced another way at Start, the stick directions are off by that much (stick up can even drive toward you) until you relocalize in a corner. The init screen and the `POSE_UNKNOWN` warning both say this.

## Driver 1 (gamepad1)

| Input | Action |
|---|---|
| Left stick | Drive, field-centric (stick up = away from the driver) |
| Right stick X | Turn |
| Sticks released | Hold position once the robot has slowed down, so a defender can't push it |
| **RB (hold)** | **Fire**, from anywhere. The robot shoots or passes by itself (see Up goal, shoot or pass) |
| **Triangle (Y)** | **Zone fire** on/off. Off at every start. While on, being in the shooting zone fires as if RB were held (see Zone fire) |
| **LB** | **Flower intake** down/up |
| **LT (hold)** | **Unjam**: run the rail backwards |

- **Shooting is continuous.** Hold RB with 2 or more balls stored. Once the turret is on target and the wheel is at speed (with the coax hood: the hood at a scoring angle and the wheel within 300 RPM of target) for 40 ms (`FireControl.READY_HOLD_MS`), the door opens and the rail feeds the balls one after another for as long as RB is held.
- How fast the balls go is set by the rail's feed power (`ShotTables.SHOT_FEED_POWER`), not by waiting between balls, so the wheel slowing a little with each ball is normal. The rail only pauses, with the door left open, while the turret (or coax hood) is off target, or while the wheel is more than 600 RPM under its target (`FireControl.BURST_RPM_FLOOR`). It goes on by itself when both are back.
- Let go of RB and the rail stops at once. The door closes 100 ms later (`FireControl.DOOR_CLOSE_DELAY_MS`), so it doesn't close on a ball that is already past it. With zone fire on, leaving the zone does the same when RB is not held.
- The rail intakes on its own. The flower intake goes up at Start, then moves only when you press LB: it stays down when storage fills (the soft rumble says full), so raise it yourself.
- Whether RB shoots or passes, and at which cell, depends on driver 2's up cell and the half of the field the robot is on (see Up goal, shoot or pass, below).
- LT counts as pressed past `MainTeleOp.TRIGGER_THRESHOLD` (0.5). RT, the D-pad, Circle, Square and Cross do nothing on driver 1.

## Driver 2 (gamepad2)

| Input | Action |
|---|---|
| **LB** / **RB** | Pick the up cell: the HIVE cell on the driver's left / right. Red: LB = FAR, RB = AUDIENCE. Blue: LB = AUDIENCE, RB = FAR. Ignored without an alliance |
| Left stick X | Turret trim while held: stick left = counter-clockwise, up to 4°/s at full stick (`MainTeleOp.TURRET_TRIM_RATE_DEG_PER_SEC`) |
| Right stick Y | RPM trim while held: stick up = more, up to 75 RPM/s (`RPM_TRIM_RATE_PER_SEC`). While the coax hood shoots (`robot.usingCoaxHood()`: `Robot.HOOD` is COAX and the `hood` servo is configured; it only switches between bursts): hood trim instead, up = steeper, which mostly lands higher, up to 1°/s (`HOOD_TRIM_RATE_DEG_PER_SEC`). With the `HOOD_MISSING` warning it stays an RPM trim |
| D-pad down | Relocalize: **left** corner on our wall, intake facing the driver (`Corner.NEAR_LEFT`) |
| D-pad up | Relocalize: **left** corner on the far wall, intake facing away (`Corner.FAR_LEFT`) |
| D-pad left | Relocalize the **heading only**, intake flat on the left side wall |
| Cross (A) | Relocalize: **right** corner on our wall, intake facing the driver (`Corner.NEAR_RIGHT`) |
| Triangle (Y) | Relocalize: **right** corner on the far wall, intake facing away (`Corner.FAR_RIGHT`) |
| Circle (B) | Relocalize the **heading only**, intake flat on the right side wall |

- The left hand does the left side and the right hand the right side, and the button points the way the intake faces: down is toward the driver, up is away, and D-pad left or Circle point at the side wall. In every spot the intake is flat on a wall. "Left" and "right" are as the driver sees them from the ALLIANCE AREA. The poses are in Relocalize poses, below.
- A corner relocalize sets the pose, trusts it and resets every trim. A heading-only one keeps x, y and whether the pose is trusted (a heading doesn't make a guessed position right, but it does fix field-centric driving), and resets only the turret trim. It does nothing while the pose is NaN. Both need an alliance picked.
- Every relocalize that happens gives both drivers one strong buzz (see Driver feedback).
- The trims move at a rate while the stick is held, so they feel the same at any loop speed. A stick inside `MainTeleOp.TRIM_STICK_DEADBAND` (0.15) counts as centered.
- D-pad right and Square do nothing on driver 2. In init, D-pad up on either gamepad picks the alliance; that press is thrown away at Start.

## Up goal, shoot or pass

The HIVE tips during the match, so one of our two cells is up. Its opening faces out along y, so it only scores from its own half of the field. Driver 2 tells the robot which cell is up (LB or RB). From then on the robot picks shoot or pass by itself, from the half it is on (`Field.cellForSide`; within `Field.CELL_SWITCH_BAND`, 6 in., of the center line it keeps the half it had, so it doesn't flicker):

| Situation | Firing does |
|---|---|
| No up cell picked: the start of every TeleOp, and every auto | **Shoot** at the cell on the robot's half |
| On the up cell's half | **Shoot** at the up cell |
| On the other half | **Pass** to the pass target on the robot's own half (see Other elements) |
| On the other half, with `FireControl.PASS_ENABLED` false | Nothing |

- "Firing" is RB, or zone fire while in the zone. Nothing fires without an alliance and a trusted pose.
- The robot decides this every loop, fire held or not, so the wheel is already at the speed of the shot or the pass when you press RB.
- Driving across the half line mid-burst closes the burst, since the wheel needs a different speed. The next burst starts when its own shot is ready.
- The static hood passes from `ShotTables.PASS_RPM` and `PASS_FEED_POWER`, which are placeholders (`TUNING_GUIDE.md`, step 7). A ball that leaves the field is a MAJOR FOUL (G405), so check them before relying on passes, or set `PASS_ENABLED` false. With the coax hood, passes are worked out from the physics. A pass outside the table's distances is out of range and doesn't fire.
- The Driver Station shows one Goal line, also with `FULL_TELEMETRY` off:
  - `Goal : not picked (D2 LB/RB) | own half (FAR)`: no up cell yet, so firing shoots at the cell on the robot's half (named at the end).
  - `Goal : FAR up | shooting`: the robot is on the up cell's half.
  - `Goal : FAR up | wrong half: passing`: it is on the other half, and firing passes.
  - `Goal : FAR up | wrong half, passing off`: it is on the other half, and firing does nothing.

## Zone fire

Driver 1's Triangle turns zone fire on and off. It is off at the start of every run. While it is on, the robot fires as if RB were held whenever any part of it is in the shooting zone and `Robot.canFire()` is true (an alliance and a trusted pose). RB still fires from anywhere.

- The zone is a trapezoid in front of the up cell's opening (`field/ShootingZone`; the cell on the robot's half if none is picked). The opening faces out along y and the places that can see it fan out with distance, so the short side is near the cell and the long side toward the wall.
- Any part counts: the robot's outline (the rectangle AgateFlow uses, intake up) only has to touch the zone. The turret center can be outside it.
- Leaving the zone stops the rail at once, unless RB is held. The door closes 100 ms later, as after letting go of RB.
- Parked in the zone, the rail keeps feeding, so balls picked up there go straight through. That is on purpose.
- The corners are `ShootingZone.RED_FAR_CORNERS` (Panels page "ShootingZone"), written for red with the FAR cell up: (48, 104), (68, 104), (90, 135), (26, 135). They are a placeholder (`TUNING_GUIDE.md`, step 12). The AUDIENCE cell's zone is the same shape mirrored across the center line (y), and blue's is mirrored across x. It is a mirror, not the 180° spin, since the cells keep their y for both alliances.
- With the placeholder, near the short side the shot can be closer than the first row of `SHOT_RPM` (20 in.), so `OUT_OF_RANGE` shows and a burst waits there.
- The Driver Station shows `Zone fire : OFF`, `Zone fire : ON, in zone` or `Zone fire : ON, out of zone`, also with `FULL_TELEMETRY` off. With `FULL_TELEMETRY` on, Panels' field page also draws the zone in yellow and the robot's outline, green in the zone and red outside it. The drawing is for practice only.

## Driver feedback

- **Gamepad light**, on driver 1's gamepad only (PlayStation controllers only): red = 0-1 balls (can't shoot), yellow = 2-3, green = 4.
- **Rumble**, three kinds. On driver 1's gamepad: 3 strong pulses = a new warning; a soft constant rumble = storage is full. On both gamepads: one short, strong pulse (200 ms, `Feedback.RELOCALIZE_MS`) = a relocalize just happened. Driver 2 knows the press counted, and driver 1 knows the robot can leave the corner. Driver 1 feels it after a warning buzz that is already playing, then the soft rumble comes back.
- **Warnings** show as `!!` lines on the Driver Station. With `MainTeleOp.FULL_TELEMETRY` off (competition), they, the Goal and Zone fire lines and the loop time are all that's sent:

| Warning | Shows when | Buzzes |
|---|---|---|
| `NO_ALLIANCE` | No alliance was picked in init. Restart the OpMode and pick one | yes |
| `POSE_UNKNOWN` | The pose isn't trusted. Relocalize in a corner. Until then, driving assumes the intake faced you at Start | yes |
| `POSE_NAN` | Odometry glitch. Nothing is aimed or saved until it clears | yes |
| `OUT_OF_RANGE` | The target is outside the shot table's distances. With the coax hood, also when the wheel is within 300 RPM of its target (`Robot.COAX_RPM_TOLERANCE`) but no hood angle scores from here | no |
| `SHOOTER_OFF` | The shooter watchdog tripped. Restart the OpMode | yes |
| `TURRET_OFF` | The turret watchdog tripped. Restart the OpMode. (A turret held by a robot or a ball pauses and comes back on its own; this only shows after 3 failed tries, about 4 to 6 s with a dead encoder, or within about 1.5 s (about 0.3 s when the target is near) if the REVERSED flags disagree) | yes |
| `SHOT_TABLE_ORDER` | A Panels edit put a shot table's distances (or `COAX_EXIT_SPEED`'s RPMs, or `COAX_EXIT_SPEED_ANGLE`'s angles) out of order | yes |
| `HOOD_MISSING` | `Robot.HOOD` is COAX but the configuration has no `hood` servo, so it shoots with the static hood's numbers | yes |
| `HOOD_AT_END` | The coax hood's servo would need a position past 0 to 1 for this angle, so the hood stops short and shots miss. The `CoaxHood` mapping numbers are off (`ANCHOR_DEG`, `SERVO_AT_ANCHOR`, `GEAR_RATIO`, `REVERSED`) | yes |

## Match rules

From the 2026-27 Competition Manual (TU04). The code is built around them:

- **Wired gamepads only.** Both plug into the Driver Hub by USB cable (R901). Wireless and Bluetooth gamepads are not allowed (R904). Any wired gamepad works, but the light only shows on PlayStation-style ones.
- **Still after INIT.** Once INIT is done, nothing may move until Start (G304.H). Between AUTO and TELEOP nothing may move under power (G403). Our INIT code writes no motor or servo, so Main TeleOp can be INITed during that transition. Everything moves in the first loop after Start: the door closes, the flower intake goes up, the turret turns to its target, the coax hood (if fitted) moves to its angle, the rail starts collecting, and the shooter spins up if balls are stored and an alliance is picked.
- **If INIT ever moves something** (R103 allows setting servos to a still position in INIT), it must be done before INIT ends, and after auto the drive team must wait until TELEOP has started before pressing INIT (G403). Check on the robot: `TUNING_GUIDE.md`, step 10.
- **No endgame.** TELEOP is 2:00 with no named ENDGAME, and the code has no match timer. Watch the field clock for the 1:00 cue.
- **No Panels at events** (R704.D). An event build keeps it off (`PanelsSettings.EVENT_BUILD`, `TUNING_GUIDE.md` step 10). Everything the drivers need is on the Driver Station: the warnings above, the Goal and Zone fire lines and the loop time. The shooting zone drawing is Panels only, for practice.

## Field coordinates

Source: the real field. FIRST's Field Acceptance Checklist (V26-27.2) wins where it controls a dimension, since events measure those. Then the official field CAD (STEP v26-27.2; manual 9.1 calls it the official field), then the Competition Manual's figures (TU07, Section 9). Every number is written in our 141.5 frame. A distance off a wall is measured from the wall; anything in the middle, or placed by tile seams or wall panel joints, is at the CAD's distance from the center. A number already within 0.1 in. of its source stays, so Pedro Visualizer points still copy 1:1. Zenith's field file uses the same numbers (see Zenith frame, below). A real field can be off by about 1 in.; AgateFlow's (the path planner's) 2 in. margin covers that.

**Frame (Pedro convention, inches),** standing where the audience stands:

- (0, 0) = near-left corner: red alliance wall × audience wall. Field is 141.5 × 141.5, so the center is (70.75, 70.75).
- +x toward the blue alliance wall, +y away from the audience.
- Heading 0 = facing +x, counter-clockwise positive.
- Blue = red spun 180° about the center: `(141.5 - x, 141.5 - y, heading + 180°)`. Enter red values only.

**Pedro Visualizer** (visualizer.pedropathing.com, BIOBUZZ field) uses exactly this frame: same corner, same directions, same headings, same 141.5 size, and its picture has red on the left and the audience at the bottom. Points copy 1:1, no scaling. Example: red's far-wall FLOWER is at (47.3, 141.5) in the visualizer and in the code.

For blue, write the red pose and let the code spin it. Don't use the visualizer's "mirror" export: it flips left to right, but the BIOBUZZ field is a 180° spin.

**Field size:** 141.5 is Pedro Visualizer's size. The official field CAD is 141.17 between the wall rails, which is where the robot touches and where the tiles end (6 tiles on a 23.5 pitch, the edge ones trimmed), and 141.35 between the clear wall panels. The manual says about 144 × 144 (9.2).

**HIVE** (Section 9.6, Figures 9-8 to 9-11):

| | Value |
|---|---|
| Frame base | 49.46 × 38.95 in., centered on (70.75, 70.75); each foot bar 2.0 wide |
| Frame legs | 1 in. tubes, leaning in from the base's corners to the pivot. For our 12.74 in. robot the red end's zone is x 46.02-50.69, y 51.28-90.23 (`FieldZones.frameEnd`) |
| Pivot height | 43.95 in. |
| Red HIVE x / blue HIVE x | 58.0 / 83.5 (25.5 in. center to center, red on the left) |
| Cells per HIVE | 2, one toward the far wall, one toward the audience; centers 30.88 in. apart along y |
| Tilt when one cell is up | 30° |
| Upward cell opening | the cell's outer end, tilted 30°: bottom 53.5 in. up, top 65.6 in.; 20 wide × 14 tall, 12 deep |
| Upward cell opening center, y | far cell 87.25, audience cell 54.25 (70.75 ± 16.5) |
| Lowest point of a HIVE | 30.6 in. up, so the robot fits under |

**AprilTags** (under each cell, facing down): red far 30-33, red audience 34-37, blue audience 38-41, blue far 42-45.

**Other elements** (red values; blue by 180° spin):

| Element | Position |
|---|---|
| FLOWERS (4, shared) | centered on (47.3, 141.5), (0, 47.3), and by spin (94.2, 0), (141.5, 94.2), at wall panel joints (CAD 23.39 from the center). Ring plates 5.95 wide and 4.9 out from the wall, ring center 2.5 out (Figure 9-12). The Field Wall Bracket on top of the wall is 6.7 wide, 1.58 out and 11.44 to 11.84 in. up, under the robot's top (CAD part am-5860). `FieldZones` covers both with one 6.7 × 4.9 zone |
| Red LOADING ZONE | x 0-11, y 94.66-117.35 (tile A5, tape just inside seams 4 and 5). Depth: the Field Acceptance Checklist's 11 ± 0.125 from the wall; the CAD's tape ends 11.48 from the wall rail |
| Red GARDEN | tape x 0.48-23.34, y 0.48-2.48 (tile A1, the near-left corner, CAD). Its 4 POLLEN sit in a line from the corner along the audience wall: centers 1.31 off both walls, 2.89 apart, 11.38 long (CAD; `IntakePoses.GARDEN_LINE_LENGTH`) |
| Pass target (`Field.PASS_TARGET_RED_X/Y`, `Field.passTarget`) | A pass lands on the robot's own half, near its own alliance wall. Red, far half: (24, 106), in front of the middle of the red LOADING ZONE. Red, audience half: (24, 35.5), the same point mirrored across the center line, toward the GARDEN side. Blue: far half (117.5, 106), audience half (117.5, 35.5). Placeholders until real passes are measured (`TUNING_GUIDE.md`, step 7) |

Nothing else on the field is lower than the robot's top: the HIVE frame's crossbar and banners are above 34 in. and the bars that hold the frame down are under the tiles.

### Zenith frame

Zenith's auto files (`autos/`) put (0, 0) at the center of the field, not the corner. The directions, the inches and the headings are the same as ours. To turn a red pose from this page into a pose for an auto file, subtract 70.75 from x and from y: (x - 70.75, y - 70.75, heading). To go the other way, add 70.75. Example: the red far-wall FLOWER at (47.3, 141.5) is (-23.45, 70.75) in a file. Files hold red poses only. The code flips them once for blue, and `ZenithFrame` (`zenith/`) does the shift. Zenith's own field file is drawn on the manual's 144 in. field, walls at ±72. Our copy, `autos/field/biobuzz.field.json`, is converted to 141.5 (walls at ±70.75) with the numbers on this page, so the file and the code describe the same field: FLOWER feet, wall brackets and rings, GARDEN POLLEN and LOADING ZONES are measured from the ±70.75 walls, and the HIVE objects are unchanged since they are measured from the center.

### Relocalize poses

Robot center = turret center. From CAD: half-width over the outer frame rails **5.87 in.**, turret center to intake front **7.99 in.** These assume the intake touches the wall it faces and the chassis side touches the other; measure on the robot (`TUNING_GUIDE.md`, step 8).

Driver 2 relocalizes with the robot pushed into one of the 4 corners. In each one the intake is flat on an alliance wall and one side of the chassis is flat on a side wall. NEAR is on our own alliance wall (intake facing the driver), FAR on the other alliance's wall (intake facing away).

From the driver's view, red's right corner is the audience corner (y = 0) and left is the far corner (y = 141.5). For blue it's the opposite.

| Button (driver 2) | `Corner` | Intake faces | Red pose (x, y, heading) | Blue pose |
|---|---|---|---|---|
| D-pad down | `NEAR_LEFT` | driver | (7.99, 135.63, 180°) | (133.51, 5.87, 0°) |
| Cross | `NEAR_RIGHT` | driver | (7.99, 5.87, 180°) | (133.51, 135.63, 0°) |
| D-pad up | `FAR_LEFT` | away | (133.51, 135.63, 0°) | (7.99, 5.87, 180°) |
| Triangle | `FAR_RIGHT` | away | (133.51, 5.87, 0°) | (7.99, 135.63, 180°) |

Two more buttons fix only the heading. Push the intake flat against the side wall and press: x and y stay as they were (`Robot.relocalizeHeading`). The heading comes from `Field.sideWallHeading`:

| Button (driver 2) | Side wall | Red heading | Blue heading |
|---|---|---|---|
| D-pad left | left (red: the far wall, blue: the audience wall) | 90° | 270° |
| Circle | right (red: the audience wall, blue: the far wall) | 270° | 90° |

The red near right corner (Cross) is the red GARDEN corner. Its tape is only 2 in. deep, so the robot fits, but its POLLEN must already be picked up.
