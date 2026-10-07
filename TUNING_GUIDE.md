# Tuning and measuring guide

Everything the code can't know until someone measures it on the robot or the field. Work top to bottom: later steps depend on earlier ones.

**The live checklist is `TODO_LIST.md`.** Every value below has a `// TODO(step): ...` comment right above it in the code, and the build collects them into `TODO_LIST.md`, sorted by step, with a link to each line. Android Studio's **TODO** tool window (View > Tool Windows > TODO) shows the same comments; click one to jump to it. When an item is done, delete its TODO comment. This guide explains how to do each step.

Tunable numbers are `public static` fields. Classes marked `@Configurable` show up in **Panels** (connect to the robot's Wi-Fi and open Panels in a browser; the address is in the Panels docs), where you can change them live. Live changes are lost when the robot app restarts, so **copy every final value back into the code**.

Each item says where its value goes: `Class.FIELD`.

## 1. Wiring and configuration

- [ ] Make the robot configuration on the Driver Station with the names in `robot/HardwareNames.java`. Write each port into the hardware table in `ROBOT.md`.
- [ ] Confirm all 8 motors and 4 servos show up (Driver Station, Configure Robot, Scan).

## 2. Drivetrain and localization (Pedro)

- [ ] Measure the Pinpoint pod offsets **from the turret center** (the robot's center in all our code). Write them in `ROBOT.md`.
- [ ] Run the tuners in order: `MecanumTuner`, `PinpointTuner`, `ForesightTuner`. They are registered in `pedro/TeamTuning.java`; `ForesightTuner` and `Tests` get added there once the first two results are pasted into `Constants` (see the TODO in that file).
- [ ] Paste their output into `pedro/Constants.create()`. Until this is done, Main TeleOp stops at init with an error saying so.
- [ ] Run the Pedro `Tests` procedure (Localization, Line, Curve). Push the robot 48 in. by hand and check the pose moves 48 in.
- [ ] Main TeleOp, red alliance: stick up drives **away from the driver**, stick left drives to the **driver's left**. Then the same on blue.
- [ ] Robot forward (heading 0) is the **intake end**. If Pedro's forward is the other end, the corner poses and the aiming are 180° off.
- [ ] `Drive.STICK_DEADBAND`, `Drive.DRIVE_CUBIC`, `Drive.TURN_CUBIC`: the stick curve. Higher cubic = finer control near the center, same top speed.
- [ ] `Drive.HOLD_WHEN_STOPPED`: drive around and let go. If holding feels jumpy, set it to false.

## 3. Storage and rail (Storage Test OpMode)

- [ ] `Storage.BLOCKED_READS_LOW`: block each beam by hand. Telemetry must show `true` when blocked. If it's backwards, flip this.
- [ ] `Rail.INTAKE_REVERSED`, `Rail.TRANSFER_REVERSED`: with Cross (collect) on, both ends must move balls **toward the door**.
- [ ] `Rail.TRANSFER_HOLD_POWER`: with a ball against the closed door, the lowest power that keeps it seated without the motor straining.
- [ ] `Rail.INTAKE_POWER`, `Rail.TRANSFER_INTAKE_POWER`: lower them if balls jam or bounce.
- [ ] `Storage.FULL_HOLD_MS`: feed balls one at a time. "Full" must only appear on the 4th. Raise it if a passing ball triggers full.
- [ ] `Storage.DEBOUNCE_LOOPS`: raise it if the count flickers.
- [ ] Watch the count go 0, 1, 2-3, 4 as balls come in.

## 4. Servos (Servo Position Finder OpMode)

- [ ] `Door.CLOSED_POSITION`, `Door.OPEN_POSITION`.
- [ ] `Door.OPEN_TIME_MS`: time from closed to fully open. Use slow-motion video, or start high and lower it until a ball catches the door.
- [ ] `Door.CLOSE_TIME_MS`: time from fully open to closed. The rail waits this long after a burst before pushing the next ball toward the door. Use slow-motion video, or start high and lower it until a ball reaches the door while it's still closing.
- [ ] `FlowerIntake.UP_POSITION`, `FlowerIntake.DOWN_POSITION`: find them on the **left** servo. The right one is reversed in code.
- [ ] `FlowerIntake.RIGHT_REVERSED`: in Main TeleOp, LB must move both servos the same way. Flip it if they fight.
- [ ] Check the flower intake reaches into a real FLOWER's Retrieval Opening (3.55 in. tall) and pulls POLLEN out.

## 5. Turret (Turret Test OpMode)

Manual mode first (Cross toggles; left stick X drives the servo):

- [ ] `Turret.SERVO_REVERSED`: stick **left** must turn the turret **counter-clockwise** (seen from above).
- [ ] `Turret.ENCODER_FULL_TURN_VOLTS`: turn the turret slowly through several full turns and watch "Encoder volts". Use the highest clean reading before it jumps back to 0 (Meta-Infinity found 3.2 V on the same encoder).
- [ ] `Turret.ENCODER_REVERSED`: the angle must **increase** when the turret turns counter-clockwise.
- [ ] `Turret.ENCODER_ZERO_DEG`: point the shooter straight out the robot's front and read the angle. Put that number here so the angle reads 0.
- [ ] `Turret.KS`: raise it from 0 in manual mode until the turret just starts to move. Use a little less.

Aiming mode (D-pad picks targets, bumpers nudge ±10°):

- [ ] `Turret.FAR_KP`, `Turret.FAR_KD`: big moves (D-pad 0 to 180). Fast, small overshoot.
- [ ] `Turret.CLOSE_KP`, `Turret.CLOSE_KD`: small moves (bumper nudges). Settles without buzzing.
- [ ] `Turret.CLOSE_ZONE_DEG`, `Turret.KS_RAMP_DEG`: adjust if it chatters near the target.
- [ ] `Turret.MAX_POWER`: lower it if the gearing or wiring complains.
- [ ] `Turret.ON_TARGET_DEG`: largest aim error that still scores. Find it with the shooter running (section 7).
- [ ] `Turret.KV`: in Main TeleOp, spin the robot in place. The turret should keep pointing at the cell. Raise KV until it stops lagging behind.

## 6. Shooter (Shooter Test OpMode)

**Before the first spin:** the two motors are geared together. If one runs the wrong way they fight. Test each motor alone at low power first.

- [ ] `Shooter.LEFT_REVERSED`, `Shooter.RIGHT_REVERSED`: both push the wheel in the shooting direction.
- [ ] `Shooter.ENCODER_REVERSED`: RPM reads positive while shooting.
- [ ] Check the encoder is on the **left** shooter motor's port. If it's on the right, change `encoderMotor` in `Shooter`.
- [ ] `Shooter.KV`: set KS and KP to 0. Raise KV until the wheel settles at the target at 2000, 3500 and 5000 RPM.
- [ ] `Shooter.KS`: raise it if low RPMs sit below target.
- [ ] `Shooter.KP`: raise it until it recovers quickly without oscillating.
- [ ] `Shooter.BOOST_BELOW_RPM`: fire 4 balls (RB) and watch RPM between balls. Recovery should be quick, with no big overshoot.
- [ ] `Shooter.RPM_TOLERANCE`: the largest RPM error that still scores at mid range.
- [ ] `FireControl.SHOT_DIP_RPM`: watch RPM as a ball goes through. Set it a bit under the smallest dip, or shots won't be counted and every ball will time out as a misfeed.
- [ ] `FireControl.RECOVER_MS`, `FireControl.READY_HOLD_MS`: lower for faster bursts, raise if back-to-back balls miss.

## 7. Shot tables (`shot/ShotTables.java`)

All values there now are placeholders.

- [ ] `SHOT_RPM`: put the robot at measured distances (turret center to the upward cell's center, along the floor) about every 10 in. from 20 to 110 in. At each one, find the RPM that scores reliably and write `{distance, rpm}`.
- [ ] Repeat a few distances with the **other** cell up to check one table works for both.
- [ ] `SHOT_FLIGHT_TIME`: slow-motion video of shots at near, mid and far distances. Count frames from leaving the wheel to entering the cell.
- [ ] `SHOT_FEED_POWER`: lower it at long range if back-to-back balls miss because RPM hasn't recovered.
- [ ] Check the robot can't reach a spot closer than the table's first row. If it can, add a row.

## 8. Field (`field/Field.java`)

- [ ] `Field.CENTER_TO_INTAKE_WALL`: push the intake flat against a wall and measure from the wall to the turret center (CAD says 7.99 in.). If the wedges or something else touches first, measure to that instead.
- [ ] `Field.CENTER_TO_SIDE_WALL`: the same for the robot's side (CAD says 5.87 in.). Check that nothing sticks out sideways past the chassis.
- [ ] Relocalize in all 4 corners (D-pad left/right, Square, Circle), then drive to a taped spot and check the pose.
- [ ] `Field.FAR_CELL_Y`, `Field.AUDIENCE_CELL_Y`: on a real field, measure from the audience wall to the center of the upward cell's opening, once with each cell up. These are computed from a drawing, not measured.
- [ ] `Field.RED_HIVE_X`: measure from the red wall to the red HIVE's center line.
- [ ] `Field.CELL_SWITCH_BAND`: drive across the middle of the field. The aim should switch cells once, not flicker.

## 9. Shooting while moving (`shot/ShotSolver.java`)

Only after stationary shots are reliable.

- [ ] With `ShotSolver.LEAD` and `ShotSolver.PREDICT` off: shooting while still must work everywhere.
- [ ] Turn `LEAD` on. Drive slowly past the HIVE at a steady speed and fire. A ball that lands ahead of the cell along the driving direction means the flight times are too short; one that lands behind means they are too long.
- [ ] Turn `PREDICT` on. Tune `ShotSolver.TURRET_LATENCY_SEC`: if the turret lags behind while driving, raise it; if it aims ahead, lower it.
- [ ] Try shooting while braking and while turning. Write down what misses for the next round of code.

## 10. Driver feedback and loop time

- [ ] `Feedback.FULL_RUMBLE_MS`, `Feedback.EMPTY_RUMBLE_MS`: long enough to feel, short enough not to annoy.
- [ ] The gamepad light only works on PlayStation-style controllers. Check yours.
- [ ] Watch "Loop" in telemetry with everything running. Goal: under 15 ms. Then turn telemetry off (gamepad2 Options) and check again.

## 11. Before competition

- [ ] Copy every value changed in Panels back into the code.
- [ ] Practice: a full match with both drivers, including relocalizing and switching cells.
- [ ] Check the ROBOT.md TBDs are all filled in.

## 12. Autonomous (`opmodes/auto/AutoTemplate.java`)

Every pose in an auto has a `TODO(12)`. Copy `AutoTemplate` for each real auto, fill in the poses, and delete each TODO once its value is measured. The building blocks are in `robot/RobotCommands.java`.

- [ ] Measure each start pose, with the robot sitting where it starts. Write it for red; the robot flips it for blue. Run it once on blue to check the flip.
- [ ] Test the commands one at a time on the field, in this order:
  - [ ] `driveTo` a short distance.
  - [ ] `shoot()` with 1 ball.
  - [ ] `shootAll()`.
  - [ ] `shootContinuous()` standing still first. Moving needs section 9 done.
- [ ] `RobotCommands.SHOOT_ONE_TIMEOUT_MS`, `RobotCommands.SHOOT_ALL_TIMEOUT_MS`: time a normal 1-ball shot and a full burst, then add margin. Long enough that a good shot never times out, short enough that a misfeed doesn't eat the auto.
- [ ] `shoot()` relies on `FireControl.SHOT_DIP_RPM` from section 6. If dips aren't detected, one "shot" can push more than one ball.
- [ ] After auto, start Main TeleOp and check there's no "POSE UNKNOWN" warning.
