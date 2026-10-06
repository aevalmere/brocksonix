# TODO list

Generated from the `// TODO(step): ...` comments in TeamCode. Don't edit this file. Do the item, delete its TODO comment, and the list updates on the next build (or run `./gradlew todoList`). How to do each step is in TUNING_GUIDE.md.

**52 left.**

## 1. Wiring and configuration

- [ ] [HardwareNames.java:10](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/HardwareNames.java#L10): Make the robot configuration on the Driver Station with these names. Write each port in ROBOT.md.

## 2. Drivetrain and localization

- [ ] [Constants.java:7](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L7) `create()`: Pinpoint pod offsets from CAD are in ROBOT.md (X pod +2.54 in. left, Y pod -1.55 in. forward). Confirm with PinpointTuner.
- [ ] [Constants.java:8](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L8) `create()`: Register MecanumTuner, PinpointTuner, ForesightTuner in Tuning.java, run them, paste the results here.
- [ ] [Constants.java:9](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L9) `create()`: Robot forward (heading 0) must be the intake end. Corner poses and aiming depend on it.
- [ ] [Constants.java:10](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L10) `create()`: Run the Pedro Tests procedure. Push the robot 48 in. by hand and check the pose moves 48 in.
- [ ] [Drive.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L24) `STICK_DEADBAND`: Raise if the robot creeps with the sticks let go; lower if small moves feel dead.
- [ ] [Drive.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L26) `DRIVE_CUBIC`: Drive and adjust to taste. Higher = gentler near center, same top speed.
- [ ] [Drive.java:29](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L29) `HOLD_WHEN_STOPPED`: Drive around and let go. If holding feels jumpy, set false.
- [ ] [Drive.java:40](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java#L40) `drive()`: Main TeleOp, red: stick up drives away from the driver, stick left to the driver's left. Check blue too.

## 3. Storage and rail

- [ ] [Rail.java:20](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Rail.java#L20) `INTAKE_REVERSED`: Storage Test, Cross on: both ends must move balls toward the door.
- [ ] [Rail.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Rail.java#L24) `INTAKE_POWER`: Lower the collect powers if balls jam or bounce.
- [ ] [Rail.java:27](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Rail.java#L27) `TRANSFER_HOLD_POWER`: Lowest power that keeps a ball seated on the closed door without the motor straining.
- [ ] [Storage.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Storage.java#L21) `BLOCKED_READS_LOW`: Storage Test: each beam must show true when blocked by hand. Flip if backwards.
- [ ] [Storage.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Storage.java#L24) `DEBOUNCE_LOOPS`: Raise if the ball count flickers.
- [ ] [Storage.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Storage.java#L26) `FULL_HOLD_MS`: Feed balls one at a time. Full must appear only on the 4th.

## 4. Servos

- [ ] [Door.java:15](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Door.java#L15) `OPEN_POSITION`: Find open and closed with Servo Position Finder.
- [ ] [Door.java:18](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Door.java#L18) `OPEN_TIME_MS`: Closed to fully open. Slow-motion video, or lower it until a ball catches the door.
- [ ] [FlowerIntake.java:17](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlowerIntake.java#L17) `UP_POSITION`: Find up and down with Servo Position Finder, on the left servo.
- [ ] [FlowerIntake.java:19](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlowerIntake.java#L19) `DOWN_POSITION`: Check it reaches into a real FLOWER's Retrieval Opening and pulls POLLEN out.
- [ ] [FlowerIntake.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlowerIntake.java#L21) `RIGHT_REVERSED`: LB in Main TeleOp must move both servos the same way. Flip if they fight.

## 5. Turret

- [ ] [Turret.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L31) `ENCODER_FULL_TURN_VOLTS`: Turret Test, manual: turn slowly through full turns. Use the highest clean volts before it drops to 0.
- [ ] [Turret.java:34](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L34) `ENCODER_ZERO_DEG`: Point the shooter straight out the front, read the angle, put it here.
- [ ] [Turret.java:37](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L37) `ENCODER_REVERSED`: The angle must increase when the turret turns counter-clockwise.
- [ ] [Turret.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L39) `SERVO_REVERSED`: Turret Test, manual: stick left must turn the turret counter-clockwise (seen from above).
- [ ] [Turret.java:42](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L42) `FAR_KP`: Aiming mode, D-pad 0 to 180: fast with a small overshoot.
- [ ] [Turret.java:45](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L45) `CLOSE_KP`: Aiming mode, bumper nudges: settles without buzzing.
- [ ] [Turret.java:52](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L52) `KS`: Manual mode: raise until the turret just starts to move, then use a bit less.
- [ ] [Turret.java:55](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L55) `KV`: Main TeleOp: spin the robot in place. Raise until the turret stops lagging behind the cell.
- [ ] [Turret.java:60](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java#L60) `ON_TARGET_DEG`: Largest aim error that still scores. Find it once the shooter works.

## 6. Shooter

- [ ] [FireControl.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L31) `READY_HOLD_MS`: Raise if shots go early while the turret is still settling.
- [ ] [FireControl.java:33](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L33) `SHOT_DIP_RPM`: Shooter Test: watch RPM as a ball goes through. Set a bit under the smallest dip you see.
- [ ] [FireControl.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L36) `RECOVER_MS`: Lower for faster bursts; raise if back-to-back balls still miss.
- [ ] [Shooter.java:25](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L25) `LEFT_REVERSED`: The motors are geared together: test each alone at low power first. Both must push the shooting direction.
- [ ] [Shooter.java:28](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L28) `ENCODER_REVERSED`: RPM must read positive while shooting. The encoder must be on the left motor's port.
- [ ] [Shooter.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L31) `KS`: Raise if low RPMs sit below target.
- [ ] [Shooter.java:33](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L33) `KV`: Shooter Test with KS and KP at 0: raise until the wheel settles on target at 2000, 3500 and 5000 RPM.
- [ ] [Shooter.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L36) `KP`: Raise until it recovers fast without oscillating.
- [ ] [Shooter.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L39) `RPM_TOLERANCE`: Largest RPM error that still scores at mid range.
- [ ] [Shooter.java:42](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java#L42) `BOOST_BELOW_RPM`: Fire 4 balls: quick recovery between balls, no big overshoot.

## 7. Shot tables

- [ ] [ShotTables.java:11](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotTables.java#L11) `SHOT_RPM`: Shooter Test at measured distances, about every 10 in. from 20 to 110. Check a few with the other cell up.
- [ ] [ShotTables.java:20](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotTables.java#L20) `SHOT_FLIGHT_TIME`: Slow-motion video of shots at near, mid and far distances.
- [ ] [ShotTables.java:27](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotTables.java#L27) `SHOT_FEED_POWER`: Lower at long range if back-to-back balls miss because RPM hasn't recovered.

## 8. Field

- [ ] [Field.java:20](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L20) `RED_HIVE_X`: On a real field, measure from the red wall to the red HIVE's center line.
- [ ] [Field.java:23](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L23) `FAR_CELL_Y`: On a real field, measure from the audience wall to the upward cell's opening center, once with each cell up.
- [ ] [Field.java:29](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L29) `CELL_SWITCH_BAND`: Drive across the middle. Raise it if the turret flips between cells too eagerly.
- [ ] [Field.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L41) `CENTER_TO_INTAKE_WALL`: CAD says 203 mm. Check with a tape: intake flat on a wall, wall to turret center. Is the stowed flower intake further out?
- [ ] [Field.java:44](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L44) `CENTER_TO_SIDE_WALL`: CAD outer frame rails say 149 mm (298 wide, not 280). Check with a tape.
- [ ] [Field.java:67](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L67) `cornerPose()`: Relocalize in all 4 corners, drive to a taped spot, and check the pose.

## 9. Shooting while moving

- [ ] [ShotSolver.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotSolver.java#L26) `LEAD`: Only after stationary shots work. Drive steadily past the HIVE and fire. Misses behind the motion = flight times too short.
- [ ] [ShotSolver.java:29](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shot/ShotSolver.java#L29) `TURRET_LATENCY_SEC`: With PREDICT on: if the turret lags while driving, raise it; if it aims ahead, lower it.

## 10. Driver feedback and loop time

- [ ] [MainTeleOp.java:30](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MainTeleOp.java#L30): Watch Loop ms with everything running, goal under 15. Check again with telemetry off (gamepad2 Options).
- [ ] [Feedback.java:17](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Feedback.java#L17) `FULL_RUMBLE_MS`: Long enough to feel, short enough not to annoy. The light only works on PlayStation controllers.
