# TeleOp controls and field reference

Button map, driver warnings, match rules and field coordinates. How each mechanism works is in the comment at the top of its class. Hardware is in `ROBOT.md`, tuning in `TUNING_GUIDE.md`.

## Init

- **D-pad up** on either gamepad toggles alliance (the first press picks red). Both gamepads light up red or blue, or white when nothing is picked. An alliance can't be picked after Start.
- Right after auto (pose saved under 60 s ago), TeleOp keeps auto's alliance and pose. Otherwise the pose is unknown: relocalize in a corner before shooting. Switching alliance drops a saved pose, since it is in the other alliance's frame.
- Button presses made during init are thrown away at Start (`resetEdgeDetection()`), so a press in init can't relocalize, move the flower intake or add a trim on the first loop. In init only D-pad up (alliance) does anything.
- **Without a saved pose, set the robot down with the intake facing you.** TeleOp guesses that heading, and field-centric driving uses it right away. If the robot faced another way at Start, the stick directions are off by that much (stick up can even drive toward you) until you relocalize in a corner. The init screen and the `POSE_UNKNOWN` warning both say this.

## Driver 1 (gamepad1)

| Input | Action |
|---|---|
| Left stick | Drive, field-centric (stick up = away from the driver) |
| Right stick X | Turn |
| Sticks released | Hold position once the robot has slowed down, so a defender can't push it |
| **RB (hold)** | **Shoot** at the upward CELL |
| RT (hold) | Pass. On hold until the coax hood is on the robot (`FireControl.PASS_ENABLED`); does nothing now. With the coax hood, passes are worked out from the physics |
| **LB** | **Flower intake** down/up |
| **LT (hold)** | **Unjam**: run the rail backwards |
| D-pad right | Relocalize: **right** corner, intake facing the driver |
| D-pad left | Relocalize: **left** corner, intake facing the driver |
| Circle (B) | Relocalize: **right** corner, intake facing the side wall |
| Square (X) | Relocalize: **left** corner, intake facing the side wall |

- **Shooting is continuous.** Hold RB with 2 or more balls stored. Once the turret is on target and the wheel is at speed (with the coax hood: the hood at a scoring angle and the wheel within 300 RPM of target) for 40 ms (`FireControl.READY_HOLD_MS`), the door opens and the rail feeds the balls one after another for as long as RB is held.
- How fast the balls go is set by the rail's feed power (`ShotTables.SHOT_FEED_POWER`), not by waiting between balls, so the wheel slowing a little with each ball is normal. The rail only pauses, with the door left open, while the turret (or coax hood) is off target, or while the wheel is more than 600 RPM under its target (`FireControl.BURST_RPM_FLOOR`). It goes on by itself when both are back.
- Let go of RB and the rail stops at once. The door closes 100 ms later (`FireControl.DOOR_CLOSE_DELAY_MS`), so it doesn't close on a ball that is already past it. If RB and RT are both held, Shoot wins.
- The rail intakes on its own. The flower intake goes up at Start, then moves only when you press LB: it stays down when storage fills (the soft rumble says full), so raise it yourself.
- The target cell is picked automatically: the cell on the robot's half of the field.
- "Left" and "right" are as the driver sees them from the ALLIANCE AREA. Relocalizing needs an alliance picked, and it resets the trims.
- Triggers count as pressed past `MainTeleOp.TRIGGER_THRESHOLD` (0.5).

## Driver 2 (gamepad2)

| Input | Action |
|---|---|
| D-pad left/right | Turret trim +2° / -2° (left = counter-clockwise) |
| D-pad up/down | RPM trim ±25. While the coax hood shoots (`robot.usingCoaxHood()`: `Robot.HOOD` is COAX and the `hood` servo is configured; it only switches between bursts): hood trim ±0.5° instead (up = steeper, which mostly lands higher). With the `HOOD_MISSING` warning it stays an RPM trim |

## Driver feedback

- **Gamepad light**, on driver 1's gamepad only (PlayStation controllers only): red = 0-1 balls (can't shoot), yellow = 2-3, green = 4.
- **Rumble**, on driver 1's gamepad only, two kinds: 3 strong pulses = a new warning; a soft constant rumble = storage is full.
- **Warnings** show as `!!` lines on the Driver Station. With `MainTeleOp.FULL_TELEMETRY` off (competition), they and the loop time are all that's sent:

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
- **No Panels at events** (R704.D). An event build keeps it off (`PanelsSettings.EVENT_BUILD`, `TUNING_GUIDE.md` step 10). Everything the drivers need is on the Driver Station: the warnings above and the loop time.

## Field coordinates

Source: positions follow Pedro Visualizer's BIOBUZZ field, and sizes come from Competition Manual Section 9 (TU03) and its figures. Both were checked against the official field CAD (the manual's figures are drawn from it), and every value below agrees across them within about 0.5 in. A real field can be off by about 1 in.; AgateFlow's (the path planner's) 2 in. margin covers that.

**Frame (Pedro convention, inches),** standing where the audience stands:

- (0, 0) = near-left corner: red alliance wall × audience wall. Field is 141.5 × 141.5, so the center is (70.75, 70.75).
- +x toward the blue alliance wall, +y away from the audience.
- Heading 0 = facing +x, counter-clockwise positive.
- Blue = red spun 180° about the center: `(141.5 - x, 141.5 - y, heading + 180°)`. Enter red values only.

**Pedro Visualizer** (visualizer.pedropathing.com, BIOBUZZ field) uses exactly this frame: same corner, same directions, same headings, same 141.5 size, and its picture has red on the left and the audience at the bottom. Points copy 1:1, no scaling. Example: red's far-wall FLOWER is at (47.3, 141.5) in the visualizer and in the code.

For blue, write the red pose and let the code spin it. Don't use the visualizer's "mirror" export: it flips left to right, but the BIOBUZZ field is a 180° spin.

**Field size:** 141.5 is Pedro Visualizer's size. The official field CAD is 141.06 inside, wall to wall (6 tiles of 23.5), and the manual says about 144 × 144 (9.2).

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
| FLOWERS (4, shared) | centered on (47.3, 141.5), (0, 47.3), and by spin (94.2, 0), (141.5, 94.2); 4.9 out from the wall; the rings are 5.9 wide, the bracket on top of the wall 6.7 (Figures 9-12 and 9-17; `FieldZones`) |
| Red LOADING ZONE | x 0-11.5, y 94.5-117.6 (tile A5, tape just inside its seams) |
| Red GARDEN | x 0-23, y 0-2 (the near-left corner). Its 4 POLLEN start in a line from the corner, touching the audience wall, 11.2 long |
| Pass target (`Field.PASS_TARGET_RED_X/Y`) | (24, 106), in front of the middle of the red LOADING ZONE; blue (117.5, 35.5). A placeholder until passing is on (`FireControl.PASS_ENABLED`) |

Nothing else on the field is lower than the robot's top: the HIVE frame's crossbar and banners are above 34 in. and the bars that hold the frame down are under the tiles.

### Relocalize poses

Robot center = turret center. From CAD: half-width over the outer frame rails **5.87 in.**, turret center to intake front **7.99 in.** These assume the intake touches the wall it faces and the chassis side touches the other; measure on the robot (`TUNING_GUIDE.md`, step 8).

From the driver's view, red's right corner is the audience corner (y = 0) and left is the far corner (y = 141.5). For blue it's the opposite.

| Button | Corner | Intake faces | Red pose (x, y, heading) | Blue pose |
|---|---|---|---|---|
| D-pad right | right | driver | (7.99, 5.87, 180°) | (133.51, 135.63, 0°) |
| D-pad left | left | driver | (7.99, 135.63, 180°) | (133.51, 5.87, 0°) |
| Circle | right | side wall | (5.87, 7.99, 270°) | (135.63, 133.51, 90°) |
| Square | left | side wall | (5.87, 133.51, 90°) | (135.63, 7.99, 270°) |

The red right corner is the red GARDEN corner. Its tape is only 2 in. deep, so the robot fits, but its POLLEN must already be picked up.
