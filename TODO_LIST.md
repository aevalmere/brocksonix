# TODO list

Generated from the `// TODO(step): ...` comments in TeamCode. Don't edit this file. Do the item, delete its TODO comment, and the list updates on the next build (or run `./gradlew todoList`). How to do each step is in TUNING_GUIDE.md.

**70 left.**

## 1. Wiring and configuration

- [ ] [HardwareNames.java:10](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/HardwareNames.java#L10) `FRONT_LEFT`: Make the robot configuration on the Driver Station with these names. Write each port in ROBOT.md.

## 2. Drivetrain and localization

- [ ] [Constants.java:7](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L7) `create()`: Pinpoint pod offsets from CAD are in ROBOT.md (X pod +2.54 in. left, Y pod -1.55 in. forward). Confirm with PinpointTuner.
- [ ] [Constants.java:8](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L8) `create()`: Run the tuners in TeamTuning.java (MecanumTuner, PinpointTuner, then ForesightTuner) and paste the results here.
- [ ] [Constants.java:9](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L9) `create()`: Robot forward (heading 0) must be the intake end. Corner poses and aiming depend on it.
- [ ] [Constants.java:10](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L10) `create()`: Run the Pedro Tests procedure. Push the robot 48 in. by hand and check the pose moves 48 in.
- [ ] [TeamTuning.java:22](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/TeamTuning.java#L22): After pasting the Mecanum and Pinpoint tuner output into Constants, add ForesightTuner here. Add Tests after pasting the Foresight output too.
- [ ] [Drive.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L24) `STICK_DEADBAND`: Raise if the robot creeps with the sticks let go; lower if small moves feel dead.
- [ ] [Drive.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L26) `DRIVE_CUBIC`: Drive and adjust DRIVE_CUBIC and TURN_CUBIC to taste. Higher = gentler near center, same top speed.
- [ ] [Drive.java:29](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L29) `HOLD_WHEN_STOPPED`: Drive around and let go. If holding feels jumpy, set false.
- [ ] [Drive.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L31) `HOLD_BELOW_SPEED`: Drive around and let go. Raise if it rolls on before holding; lower if the hold feels jumpy.
- [ ] [Drive.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L41) `drive()`: Main TeleOp, red: stick up drives away from the driver, stick left to the driver's left. Check blue too.

## 3. Storage and rail

- [ ] [Rail.java:19](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Rail.java#L19) `INTAKE_REVERSED`: Storage Test, Cross on: both ends must move balls toward the door.
- [ ] [Rail.java:23](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Rail.java#L23) `INTAKE_POWER`: Lower the collect powers if balls jam or bounce.
- [ ] [Rail.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Rail.java#L26) `TRANSFER_HOLD_POWER`: Lowest power that keeps a ball seated on the closed door without the motor straining.
- [ ] [Storage.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Storage.java#L21) `BLOCKED_READS_LOW`: Storage Test: each beam must show true when blocked by hand. Flip if backwards.
- [ ] [Storage.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Storage.java#L24) `DEBOUNCE_MS`: Raise (in ms) if the ball count flickers.
- [ ] [Storage.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Storage.java#L26) `FULL_HOLD_MS`: Feed balls one at a time. Full must appear only on the 4th.

## 4. Servos

- [ ] [Door.java:15](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Door.java#L15) `OPEN_POSITION`: Find open and closed with Servo Position Finder.
- [ ] [Door.java:18](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Door.java#L18) `OPEN_TIME_MS`: Closed to fully open. Slow-motion video, or lower it until a ball catches the door.
- [ ] [Door.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Door.java#L21) `CLOSE_TIME_MS`: Fully open to closed. Slow-motion video, or lower it until a ball reaches the door while it's still closing.
- [ ] [FlowerIntake.java:17](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlowerIntake.java#L17) `UP_POSITION`: Find up and down with Servo Position Finder, on the left servo.
- [ ] [FlowerIntake.java:19](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlowerIntake.java#L19) `DOWN_POSITION`: Check it reaches into a real FLOWER's Retrieval Opening and pulls POLLEN out.
- [ ] [FlowerIntake.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlowerIntake.java#L21) `RIGHT_REVERSED`: LB in Main TeleOp must move both servos the same way. Flip if they fight.

## 5. Turret

- [ ] [Turret.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L39) `ENCODER_FULL_TURN_VOLTS`: Turret Test, manual: turn slowly through full turns. Use the highest clean volts before it drops to 0.
- [ ] [Turret.java:42](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L42) `ENCODER_ZERO_DEG`: Point the shooter straight out the front, read the angle, put it here.
- [ ] [Turret.java:45](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L45) `ENCODER_REVERSED`: The angle must increase when the turret turns counter-clockwise.
- [ ] [Turret.java:47](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L47) `SERVO_REVERSED`: Turret Test, manual: stick left must turn the turret counter-clockwise (seen from above).
- [ ] [Turret.java:50](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L50) `FAR_KP`: Aiming mode, D-pad 0 to 180: tune FAR_KP and FAR_KD for fast moves with a small overshoot.
- [ ] [Turret.java:53](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L53) `CLOSE_KP`: Aiming mode, bumper nudges: tune CLOSE_KP and CLOSE_KD so it settles without buzzing.
- [ ] [Turret.java:56](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L56) `CLOSE_ZONE_DEG`: Aiming mode: if it chatters near the target, adjust this.
- [ ] [Turret.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L61) `KS`: Manual mode: raise until the turret just starts to move, then use a bit less.
- [ ] [Turret.java:63](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L63) `KS_RAMP_DEG`: Aiming mode: if it chatters near the target, adjust this. Higher = KS fades in over a wider zone.
- [ ] [Turret.java:65](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L65) `KV`: Main TeleOp: spin the robot in place. Raise until the turret stops lagging behind the cell.
- [ ] [Turret.java:68](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L68) `MAX_POWER`: Lower if the gearing or wiring complains.
- [ ] [Turret.java:71](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L71) `ON_TARGET_DEG`: Largest aim error that still scores. Find it once the shooter works.
- [ ] [Turret.java:74](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L74) `WATCHDOG_MS`: Aiming mode, D-pad 0 to 180: if a normal big move trips the watchdog, raise this time.

## 6. Shooter

- [ ] [FireControl.java:35](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L35) `READY_HOLD_MS`: Raise if shots go early while the turret is still settling.
- [ ] [FireControl.java:37](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L37) `SHOT_DIP_RPM`: Shooter Test: watch RPM as a ball goes through. Set a bit under the smallest dip you see.
- [ ] [FireControl.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L39) `FEED_TIMEOUT_MS`: 400 is a guess. Watch how long a normal feed takes to show the RPM dip, then set this a bit above the slowest.
- [ ] [FireControl.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L41) `RECOVER_MS`: Lower for faster bursts; raise if back-to-back balls still miss.
- [ ] [Shooter.java:33](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L33) `LEFT_REVERSED`: The motors are geared together: test each alone at low power first (Shooter Test, hold Square or Circle). Both must push the shooting direction.
- [ ] [Shooter.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L36) `ENCODER_REVERSED`: RPM must read positive while shooting. The encoder must be on the left motor's port. This flag alone sets the RPM sign, LEFT_REVERSED does not change it.
- [ ] [Shooter.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L39) `KS`: Raise if low RPMs sit below target.
- [ ] [Shooter.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L41) `KV`: Shooter Test with KS and KP at 0: raise until the wheel settles on target at 2000, 3500 and 5000 RPM.
- [ ] [Shooter.java:44](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L44) `KP`: Raise until it recovers fast without oscillating.
- [ ] [Shooter.java:47](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L47) `RPM_TOLERANCE`: Largest RPM error that still scores at mid range.
- [ ] [Shooter.java:50](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L50) `BOOST_BELOW_RPM`: Fire 4 balls: quick recovery between balls, no big overshoot.
- [ ] [Shooter.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L61) `WATCHDOG_MS`: Only if the watchdog trips on a normal spin-up: raise this, or lower WATCHDOG_MIN_RPM.
- [ ] [Shooter.java:98](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L98) `setZeroPowerBehavior()`: After a trim-down the wheel coasts (FLOAT) and atSpeed() blocks shots until it slows. Time a 100 to 200 RPM coast-down; if it blocks shots, tell the programmers to add braking or a small reverse power.

## 7. Shot tables

- [ ] [ShotTables.java:22](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotTables.java#L22) `SHOT_RPM`: Shooter Test at measured distances, about every 10 in. from 20 to 110. Check a few with the other cell up.
- [ ] [ShotTables.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotTables.java#L31) `SHOT_FLIGHT_TIME`: Slow-motion video of shots at near, mid and far distances.
- [ ] [ShotTables.java:38](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotTables.java#L38) `SHOT_FEED_POWER`: Lower at long range if back-to-back balls miss because RPM hasn't recovered.

## 8. Field

- [ ] [Field.java:20](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L20) `RED_HIVE_X`: On a real field, measure from the red wall to the red HIVE's center line.
- [ ] [Field.java:23](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L23) `FAR_CELL_Y`: On a real field, measure from the audience wall to the upward cell's opening center, once with each cell up.
- [ ] [Field.java:29](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L29) `CELL_SWITCH_BAND`: Drive across the middle. Raise it if the turret flips between cells too eagerly.
- [ ] [Field.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L41) `CENTER_TO_INTAKE_WALL`: CAD says 203 mm. Check with a tape: intake flat on a wall, wall to turret center. Is the stowed flower intake further out?
- [ ] [Field.java:44](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L44) `CENTER_TO_SIDE_WALL`: CAD outer frame rails say 149 mm (298 wide, not 280). Check with a tape.
- [ ] [Field.java:70](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L70) `cornerPose()`: Relocalize in all 4 corners, drive to a taped spot, and check the pose.

## 9. Shooting while moving

- [ ] [ShotSolver.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotSolver.java#L26) `LEAD`: Only after stationary shots work. Turn on, drive steadily past the HIVE and fire. Lands ahead of the cell = flight times too short; behind = too long.
- [ ] [ShotSolver.java:28](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotSolver.java#L28) `PREDICT`: Turn on after LEAD works.
- [ ] [ShotSolver.java:30](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotSolver.java#L30) `TURRET_LATENCY_SEC`: With PREDICT on: if the turret lags while driving, raise it; if it aims ahead, lower it.

## 10. Driver feedback and loop time

- [ ] [MainTeleOp.java:32](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MainTeleOp.java#L32) `TELEMETRY_INTERVAL_MS`: Watch Loop ms with everything running, goal under 15. Check again with telemetry off (gamepad2 Options).
- [ ] [Feedback.java:27](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Feedback.java#L27) `WARNING_STRENGTH`: Warning buzz: strong and unmistakable, long enough to feel, short enough not to annoy. Raise the cooldown if a flickering warning still annoys.
- [ ] [Feedback.java:35](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Feedback.java#L35) `FULL_STRENGTH`: Full rumble: soft enough to drive with, strong enough to notice. The gamepad light only works on PlayStation controllers.

## 12. Autonomous

- [ ] [AutoTemplate.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L31): Copy this file for a real auto, fill in the poses and the routine, and delete the @Disabled line.
- [ ] [AutoTemplate.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L39) `START_X`: Measure where the robot starts on the field. Center of the turret, with the robot sitting in place.
- [ ] [AutoTemplate.java:43](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L43) `SHOOT_X`: Pick where to shoot from and check the distance to the cell is inside the ShotTables range.
- [ ] [AutoTemplate.java:81](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L81) `schedule()`: Replace this placeholder with the real routine.
- [ ] [RobotCommands.java:32](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotCommands.java#L32) `SHOOT_ONE_TIMEOUT_MS`: Time a normal 1-ball shot, then add margin. Long enough that a good shot never times out, short enough that a misfeed doesn't eat the auto.
- [ ] [RobotCommands.java:34](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotCommands.java#L34) `SHOOT_ALL_TIMEOUT_MS`: Time a full burst the same way.
