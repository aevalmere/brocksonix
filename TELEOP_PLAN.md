# TeleOp controls and field reference

Button map, driver warnings and field coordinates. How each mechanism works is in the comment at the top of its class. Hardware is in `ROBOT.md`, tuning in `TUNING_GUIDE.md`.

## Init

- **D-pad up** on either gamepad toggles alliance (the first press picks red). Both gamepads light up red or blue, or white when nothing is picked. An alliance can't be picked after Start.
- Right after auto (pose saved under 60 s ago), TeleOp keeps auto's alliance and pose. Otherwise the pose is unknown: relocalize in a corner before shooting. Switching alliance drops a saved pose, since it is in the other alliance's frame.
- **Without a saved pose, set the robot down with the intake facing you.** TeleOp guesses that heading, and field-centric driving uses it right away. If the robot faced another way at Start, the stick directions are off by that much (stick up can even drive toward you) until you relocalize in a corner. The init screen and the `POSE_UNKNOWN` warning both say this.

## Driver 1 (gamepad1)

| Input | Action |
|---|---|
| Left stick | Drive, field-centric (stick up = away from the driver) |
| Right stick X | Turn |
| Sticks released | Hold position once the robot has slowed down, so a defender can't push it |
| **RB (hold)** | **Shoot** at the upward CELL |
| RT (hold) | Pass. On hold until the variable hood (`FireControl.PASS_ENABLED`); does nothing now |
| **LB** | **Flower intake** down/up |
| **LT (hold)** | **Unjam**: run the rail backwards |
| D-pad right | Relocalize: **right** corner, intake facing the driver |
| D-pad left | Relocalize: **left** corner, intake facing the driver |
| Circle (B) | Relocalize: **right** corner, intake facing the side wall |
| Square (X) | Relocalize: **left** corner, intake facing the side wall |

- A burst only starts with 2 or more balls stored, then fires one ball at a time while RB is held. Letting go mid-feed lets that ball finish. If RB and RT are both held, Shoot wins.
- The rail intakes on its own. The flower intake raises itself when storage is full.
- The target cell is picked automatically: the cell on the robot's half of the field.
- "Left" and "right" are as the driver sees them from the ALLIANCE AREA. Relocalizing needs an alliance picked, and it resets the trims.
- Triggers count as pressed past `MainTeleOp.TRIGGER_THRESHOLD` (0.5).

## Driver 2 (gamepad2)

| Input | Action |
|---|---|
| D-pad left/right | Turret trim +2° / -2° (left = counter-clockwise) |
| D-pad up/down | RPM trim ±25 |

## Driver feedback

- **Gamepad light** (PlayStation controllers only): red = 0-1 balls (can't shoot), yellow = 2-3, green = 4.
- **Rumble**, only two kinds: 3 strong pulses = a new warning; a soft constant rumble = storage is full.
- **Warnings** show as `!!` lines on the Driver Station. With `MainTeleOp.FULL_TELEMETRY` off (competition), they and the loop time are all that's sent:

| Warning | Shows when | Buzzes |
|---|---|---|
| `NO_ALLIANCE` | No alliance was picked in init. Restart the OpMode and pick one | yes |
| `POSE_UNKNOWN` | The pose isn't trusted. Relocalize in a corner. Until then, driving assumes the intake faced you at Start | yes |
| `POSE_NAN` | Odometry glitch. Nothing is aimed or saved until it clears | yes |
| `OUT_OF_RANGE` | The target is outside the shot table's distances | no |
| `SHOOTER_OFF` | The shooter watchdog tripped. Restart the OpMode | yes |
| `TURRET_OFF` | The turret watchdog tripped. Restart the OpMode. (A turret held by a robot or a ball pauses and comes back on its own; this only shows after 3 failed tries, about 4 to 6 s with a dead encoder, or within about 1.5 s (about 0.3 s when the target is near) if the REVERSED flags disagree) | yes |
| `SHOT_TABLE_ORDER` | A Panels edit put a shot table's distances out of order | yes |

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
