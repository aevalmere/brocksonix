# Robot: BIOBUZZ 2026-2027

Hardware reference for the team robot. Update it when the robot changes.

Items marked **TBD** are not decided or not yet measured. All dimensions are in mm unless noted.

## Game context (BIOBUZZ)

What matters for this robot, from the Competition Manual (TU03):

- **Scoring element:** POLLEN. Yellow polyethylene balls, about 2.8 in. (7.1 cm), not perfectly round, and sizes vary (9.8). This robot handles **POLLEN only** and does not handle NECTAR (3.6 in. red/blue balls).
- **Possession limit:** a robot may not control more than 4 scoring elements at once (G407). This robot stores at most 4, so it is at the limit by design.
- **Preload:** each robot starts the match touching 4 POLLEN (10.3.4).
- **HIVE:** in the center of the field. Each alliance's HIVE has 2 CELLS on a pivot 43.95 in. above the tiles. The upward-facing CELL opening is about 20 in. wide by 14 in. tall by 12 in. deep, with its bottom 53.5 in. and its top 65.6 in. above the tiles (Figure 9-10). The HIVE tips during the match, so which of the 2 CELLS faces up changes. Launching enough POLLEN into it tips the HIVE (20 pts in AUTO or TELEOP). Each CELL has a 4-tag AprilTag cluster (36h11, 3.25 in.) on its underside, facing down (9.6, 9.9).
- **FLOWERS:** 4 total, mounted on the perimeter wall, each staged with 4 POLLEN. POLLEN may only be removed through the Retrieval Opening at the bottom, about 3.55 in. tall by 3.57 in. deep (9.7, G418). Driving into a FLOWER and touching the POLLEN inside is allowed.
- **GARDEN:** 4 POLLEN staged in a line in each alliance's corner.
- **Size:** 18 x 18 x 18 in. (457 mm cube) at the start (R102). Once the match starts, it may expand to 18 x 24 in. (457 x 610 mm) and 29 in. (737 mm) tall, and the limit must be mechanical, not software (R105).
- **Actuator limit:** 8 motors and 8 servos max (R503). This robot uses **8 motors** (at the limit) and **4 servos**.
- **Match:** 30 s AUTO, 8 s transition, 2:00 TELEOP.

## Hardware summary

| Qty | Part | Device | Hub / port | Notes |
|-----|------|--------|------------|-------|
| 4 | Drivetrain | goBILDA 6000 RPM bare motor, 10:105 reduction | TBD | Mecanum, 104 mm GripForce wheels |
| 1 | Intake | goBILDA 6000 RPM bare motor, geared to about 4700 RPM | TBD | Intake end of the rail, 38.1 mm wheels |
| 1 | Transfer | goBILDA 6000 RPM bare motor, 20:32 reduction | TBD | Door end of the rail, 48 mm wheels |
| 2 | Shooter | goBILDA 6000 RPM bare motor, with encoder | TBD | Mechanically coupled, mounted opposite each other |
| 1 | Turret | Axon servo, CR mode | TBD (hub servo port, 5 V) | Infinite rotation, 1:2 gearing (turret turns 2x per servo turn) |
| 1 | Turret encoder | East Loop Components ELC Encoder V2 | TBD (analog port) | On the turret; analog absolute output used |
| 2 | Flower intake | SWYFT Slim servo | TBD (hub servo port, 5 V) | Mounted opposite each other, mirrored |
| 1 | Door | SWYFT Slim servo | TBD (hub servo port, 5 V) | End of the rail, before the shooter |
| 2 | Transfer break beams | Adafruit 3 mm IR break beam | TBD (digital port) | Slots 1 and 2 |
| 1 | Intake break beam | Adafruit 3 mm IR break beam | TBD (digital port) | Slot 4, trips only when all 4 are stored |
| 1 | Localization | goBILDA Pinpoint V2 | TBD (I2C) | goBILDA 4-bar odometry pods |

**Motors:** 8 of 8 allowed. **Servos:** 4 of 8 allowed.

## Control system

- REV Control Hub plus REV Expansion Hub.
- All servos run straight off hub servo ports at 5 V. There is no servo power module.
- Port assignments: TBD.

## Dimensions

| Measurement | Value |
|-------------|-------|
| Chassis width | 280 (CAD bottom view measures about 298 across the outer frame rails; check which one touches a wall) |
| Chassis length, including wedges | 318 |
| Wedge length (front only) | 27.19241 |
| Chassis length, without wedges | 290.80759 |
| Intake reach past the chassis front (CAD shows it is measured from the chassis front, not the wedge tips; about 31 past the wedge tips) | 57.55709 |
| Flower intake touchdown point past the front of the chassis (not the wedges) | 129.88894 |
| Turret position | Center of the chassis, wedges excluded: 145.4 behind the chassis front, 172.6 behind the wedge tips |
| Intake front to turret center (CAD) | about 203 |
| Flower intake touchdown to turret center | 275.3 |
| Overall length, flower intake down | 420.7 (limit 610) |
| Pinpoint pod offsets | see Localization (CAD estimate) |

## Mechanisms

### Drivetrain
- Mecanum, four goBILDA 6000 RPM bare motors.
- 10:105 reduction (10.5:1), so the wheels run about 571 RPM free speed.
- goBILDA 104 mm GripForce mecanum wheels.
- Free speed is about 3.1 m/s (10.2 ft/s) before losses.

### Localization
goBILDA Pinpoint V2 odometry computer with two goBILDA 4-bar odometry pods.

Pod positions from the turret center, measured off the CAD bottom view (scale 2.37 px/mm, checked against the 420.7 overall length, the 145.4 turret position and the 27 mm wedges). Accurate to a few mm. The bottom view is mirrored, so image right is robot left; if the CAD view wasn't mirrored, flip the left/right signs.

| Pod | Measures | Forward of center | Left of center |
|-----|----------|-------------------|----------------|
| Forward pod (X) | forward motion | -80 mm (-3.16 in.) | +64 mm (+2.54 in.) |
| Strafe pod (Y) | sideways motion | -39 mm (-1.55 in.) | -59 mm (-2.34 in.) |

In goBILDA's Pinpoint terms: X pod offset (forward pod, left positive) = **+2.54 in.**, Y pod offset (strafe pod, forward positive) = **-1.55 in.** Confirm with the PinpointTuner.

### Intake
A goBILDA 6000 RPM bare motor drives the intake end of the shared rail (slots 3 and 4) through gearing that gives about 4700 RPM at the 38.1 mm wheels. That is about 9.4 m/s surface speed at free RPM, matched to the transfer. At its furthest point the intake sits 57.55709 mm past the chassis front (about 31 mm past the wedge tips).

### Transfer
A goBILDA 6000 RPM bare motor drives the door end of the shared rail (slots 1 and 2) through a 20:32 reduction, giving 3750 RPM at the 48 mm wheels. That is about 9.4 m/s surface speed at free RPM, matched to the intake. It pushes balls into the shooter when the door opens.

### Door
SWYFT Slim servo at the end of the rail. Closed, it stops the slot 1 ball from reaching the shooter. Open, it lets balls feed into the shooter. Servo range is tuned on the robot.

### Shooter
- Two goBILDA 6000 RPM bare motors, mechanically coupled and mounted opposite each other, which is why they spin in opposite directions. Both have encoders.
- One 72 mm shooter wheel, driven 1:1 (6000 RPM free, about 22.6 m/s surface speed).
- 48 mm counter-roller geared off the same two motors to 9000 RPM (about 22.6 m/s surface speed, same as the wheel).
- Fixed launch angle (no adjustable hood). A variable hood is planned, but not soon. Passing (RT) waits for it.
- Full power is expected to be far more than needed, so the shooter should normally run well below its free speed.

### Turret
- Axon servo in continuous rotation (CR) mode, powered at 5 V from the hub.
- 1:2 gearing: the turret turns 2 revolutions for every 1 servo revolution (geared up, not down).
- Rotation is infinite, so there are no hard stops and no cable wind-up limit.
- Center of rotation is the center of the chassis (wedges excluded).
- An East Loop Components ELC Encoder V2 sits on the turret and reads the turret's own angle, not the servo's. It has two outputs on one JST-PH 4-pin connector:
  - an absolute angle as analog voltage, 0 to 3.3 V
  - quadrature at 4000 CPR
- Only the analog output is wired, to a hub analog port. It gives the turret's absolute angle at power-on with no homing step, and that holds even though the turret rotates infinitely.

### Flower intake
- Two SWYFT Slim servos mounted opposite each other, so they turn in mirrored directions.
- Two positions: up (stowed) and down. Servo ranges are tuned on the robot.
- When down, it touches the ground 129.88894 mm past the front of the chassis (not the wedges), and the robot drives into a FLOWER's bottom Retrieval Opening to pull POLLEN out.

## Ball path and storage

All storage is one straight linear rail with wheels. It holds 4 POLLEN in a single line. The intake motor drives the intake end of the rail and the transfer motor drives the door end. The wheel sizes differ (38.1 mm intake, 48 mm transfer), but each motor is geared so both ends run at about the same surface speed (about 9.4 m/s at free RPM), so balls move along the rail without bunching or gapping.

```
 FLOWER / floor
       │
       ▼
 [flower intake] ──> ┌───────────── linear rail ─────────────┐ ──> |door| ──> [shooter] ──> HIVE CELL
   2 servos          │ slot 4    slot 3  │  slot 2    slot 1 │      servo     2 motors
                     │   intake motor    │  transfer motor   │
                     └───────────────────────────────────────┘
                     beam at slot 4         beams at slots 2, 1
```

Storage holds 4 POLLEN max (the G407 limit). Slots are numbered from the door back toward the intake:

| Slot | Location | Driven by | Sensor |
|------|----------|-----------|--------|
| 1 | Against the closed door (first ball to shoot) | Transfer motor | Transfer beam A |
| 2 | Behind slot 1, toward the intake | Transfer motor | Transfer beam B |
| 3 | Behind slot 2 | Intake motor | None |
| 4 | Intake end of the rail | Intake motor | Intake beam (trips and stays tripped only when all 4 are stored) |

Break beams are Adafruit 3 mm IR break beam sensors (separate emitter and receiver). The receiver output is open collector and connects to a hub digital port. Which logic level means "broken" should be checked on the robot.

## Open hardware details (TBD)

- Hub and port assignment for every device
- Exact intake gear ratio (about 4700 RPM at the wheel)
- Pinpoint pod mounting offsets
