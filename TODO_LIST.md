# TODO list

Generated from the `// TODO(step): ...` comments in TeamCode. Don't edit this file. Do the item, delete its TODO comment, and the list updates on the next build (or run `./gradlew todoList`). How to do each step is in TUNING_GUIDE.md.

Most important first: work from the top down. Phase 1 gets the robot playing TeleOp, phase 2 adds a scoring auto, phase 3 makes both better, phase 4 adds AgateFlow and auto pickups.

**97 left.**

## Phase 1: TeleOp, bare minimum (52 left)

### 1. Wiring and configuration

- [ ] [HardwareNames.java:10](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/hardware/HardwareNames.java#L10) `FRONT_LEFT`: Make the robot configuration on the Driver Station with these names. Write each port in ROBOT.md.

### 2. Drivetrain and localization

- [ ] [DriveModel.java:51](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/DriveModel.java#L51) `TUNED_VOLTS`: Write down the battery voltage while ForesightTuner runs. It is for AgateFlow (step 13), but now is the only time to read it without rerunning the tuner.
- [ ] [Constants.java:7](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L7) `create()`: Pinpoint pod offsets from CAD are in ROBOT.md (X pod +2.54 in. left, Y pod -1.55 in. forward). Confirm with PinpointTuner.
- [ ] [Constants.java:8](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L8) `create()`: Run the tuners in TeamTuning.java (MecanumTuner, PinpointTuner, then ForesightTuner) and paste the results here.
- [ ] [Constants.java:9](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L9) `create()`: Robot forward (heading 0) must be the intake end. Corner poses and aiming depend on it.
- [ ] [Constants.java:10](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java#L10) `create()`: Run the Pedro Tests procedure. Push the robot 48 in. by hand and check the pose moves 48 in.
- [ ] [TeamTuning.java:22](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/TeamTuning.java#L22): After pasting the Mecanum and Pinpoint tuner output into Constants, add ForesightTuner here. Add Tests after pasting the Foresight output too.
- [ ] [Drive.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Drive.java#L24) `STICK_DEADBAND`: Raise if the robot creeps with the sticks let go; lower if small moves feel dead.
- [ ] [Drive.java:29](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Drive.java#L29) `HOLD_WHEN_STOPPED`: Drive around and let go. If holding feels jumpy, set false.
- [ ] [Drive.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Drive.java#L31) `HOLD_BELOW_SPEED`: Drive around and let go. Raise if it rolls on before holding; lower if the hold feels jumpy.
- [ ] [Drive.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Drive.java#L41) `drive()`: Main TeleOp, red: stick up drives away from the driver, stick left to the driver's left. Check blue too.

### 3. Storage and rail

- [ ] [Door.java:15](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Door.java#L15) `OPEN_POSITION`: Find open and closed with Servo Position Finder, before Storage Test (it holds the door closed).
- [ ] [Rail.java:19](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Rail.java#L19) `INTAKE_REVERSED`: Storage Test, Cross on: both ends must move balls toward the door.
- [ ] [Rail.java:23](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Rail.java#L23) `INTAKE_POWER`: Lower the collect powers if balls jam or bounce.
- [ ] [Rail.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Rail.java#L26) `TRANSFER_HOLD_POWER`: Lowest power that keeps a ball seated on the closed door without the motor straining.
- [ ] [Storage.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Storage.java#L21) `BLOCKED_READS_LOW`: Storage Test: each beam must show true when blocked by hand. Flip if backwards.
- [ ] [Storage.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Storage.java#L24) `DEBOUNCE_MS`: Raise (in ms) if the ball count flickers.
- [ ] [Storage.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Storage.java#L26) `FULL_HOLD_MS`: Feed balls one at a time. Full must appear only on the 4th.

### 4. Servos

- [ ] [Door.java:18](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Door.java#L18) `OPEN_TIME_MS`: Closed to fully open. Slow-motion video in Shooter Test (wheel off, hold RB), or after step 6 lower it until a ball catches the door.
- [ ] [Door.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Door.java#L21) `CLOSE_TIME_MS`: Fully open to closed. Slow-motion video in Shooter Test (let go of RB), or after step 6 lower it in Main TeleOp until a ball reaches the door while it's still closing.
- [ ] [FlowerIntake.java:17](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/FlowerIntake.java#L17) `UP_POSITION`: Find up and down with Servo Position Finder, on the left servo.
- [ ] [FlowerIntake.java:19](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/FlowerIntake.java#L19) `DOWN_POSITION`: Check it reaches into a real FLOWER's Retrieval Opening and pulls POLLEN out.
- [ ] [FlowerIntake.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/FlowerIntake.java#L21) `RIGHT_REVERSED`: LB in Main TeleOp must move both servos the same way. Flip if they fight.

### 5. Turret

- [ ] [Turret.java:58](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L58) `ENCODER_FULL_TURN_VOLTS`: Turret Test, manual: turn slowly through full turns. Use the highest clean volts before it drops to 0.
- [ ] [Turret.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L61) `ENCODER_ZERO_DEG`: Point the shooter straight out the front, read the angle, put it here.
- [ ] [Turret.java:64](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L64) `ENCODER_REVERSED`: The angle must increase when the turret turns counter-clockwise.
- [ ] [Turret.java:66](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L66) `SERVO_REVERSED`: Turret Test, manual: stick left must turn the turret counter-clockwise (seen from above).
- [ ] [Turret.java:69](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L69) `FAR_KP`: Aiming mode, D-pad 0 to 180: tune FAR_KP and FAR_KD for fast moves with a small overshoot.
- [ ] [Turret.java:72](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L72) `CLOSE_KP`: Aiming mode, bumper nudges: tune CLOSE_KP and CLOSE_KD so it settles without buzzing.
- [ ] [Turret.java:75](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L75) `CLOSE_ZONE_DEG`: Aiming mode: if it chatters near the target, adjust this.
- [ ] [Turret.java:80](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L80) `KS`: Manual mode (KS isn't used there): push the stick slowly, note the power where the turret starts to move, set KS a bit below it.
- [ ] [Turret.java:82](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L82) `KS_RAMP_DEG`: Aiming mode: if it chatters near the target, adjust this. Higher = KS fades in over a wider zone.
- [ ] [Turret.java:87](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L87) `MAX_POWER`: Lower if the gearing or wiring complains.
- [ ] [Turret.java:90](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L90) `ON_TARGET_DEG`: Largest aim error that still scores. Find it once the shooter works.
- [ ] [Turret.java:93](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L93) `WATCHDOG_MS`: Aiming mode, D-pad 0 to 180: if a normal big move trips the watchdog, raise this time.

### 6. Shooter

- [ ] [FireControl.java:35](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L35) `READY_HOLD_MS`: Raise if shots go early while the turret is still settling.
- [ ] [FireControl.java:37](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L37) `SHOT_DIP_RPM`: Shooter Test: watch RPM as a ball goes through. Set a bit under the smallest dip you see.
- [ ] [FireControl.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L39) `FEED_TIMEOUT_MS`: 400 is a guess. Watch how long a normal feed takes to show the RPM dip, then set this a bit above the slowest.
- [ ] [FireControl.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L41) `RECOVER_MS`: Lower for faster bursts; raise if back-to-back balls still miss.
- [ ] [Shooter.java:33](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L33) `LEFT_REVERSED`: The motors are geared together: test each alone at low power first (Shooter Test, hold Square or Circle). Both must push the shooting direction.
- [ ] [Shooter.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L36) `ENCODER_REVERSED`: RPM must read positive while shooting. The encoder must be on the left motor's port. This flag alone sets the RPM sign, LEFT_REVERSED does not change it.
- [ ] [Shooter.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L39) `KS`: Raise if low RPMs sit below target.
- [ ] [Shooter.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L41) `KV`: Shooter Test with KS and KP at 0: raise until the wheel settles on target at 2000, 3500 and 5000 RPM.
- [ ] [Shooter.java:44](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L44) `KP`: Raise until it recovers fast without oscillating.
- [ ] [Shooter.java:47](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L47) `RPM_TOLERANCE`: Largest RPM error that still scores at mid range.
- [ ] [Shooter.java:50](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L50) `BOOST_BELOW_RPM`: Fire 4 balls: quick recovery between balls, no big overshoot.
- [ ] [Shooter.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L61) `WATCHDOG_MS`: Only if the watchdog trips on a normal spin-up: raise this, or lower WATCHDOG_MIN_RPM.

### 7. Shot tables

- [ ] [ShotTables.java:22](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L22) `SHOT_RPM`: Shooter Test at measured distances, about every 10 in. from 20 to 110. Check a few with the other cell up.
- [ ] [ShotTables.java:38](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L38) `SHOT_FEED_POWER`: Lower at long range if back-to-back balls miss because RPM hasn't recovered.

### 8. Field and relocalizing

- [ ] [Field.java:60](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L60) `CENTER_TO_INTAKE_WALL`: CAD says 203 mm. Check with a tape: intake flat on a wall, wall to turret center. Is the stowed flower intake further out?
- [ ] [Field.java:63](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L63) `CENTER_TO_SIDE_WALL`: CAD outer frame rails say 149 mm (298 wide, not 280). Check with a tape.
- [ ] [Field.java:89](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L89) `cornerPose()`: Relocalize in all 4 corners, drive to a taped spot, and check the pose.

## Phase 2: Autonomous and competition (9 left)

### 9. First autonomous

- [ ] [AutoTemplate.java:32](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L32): Copy this file for a real auto, fill in the poses and the routine, and delete the @Disabled line.
- [ ] [AutoTemplate.java:40](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L40) `START_X`: Measure where the robot starts on the field. Center of the turret, with the robot sitting in place.
- [ ] [AutoTemplate.java:44](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L44) `SHOOT_X`: Pick where to shoot from and check the distance to the cell is inside the ShotTables range.
- [ ] [AutoTemplate.java:83](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L83) `schedule()`: Replace this placeholder with the real routine.
- [ ] [RobotCommands.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L41) `SHOOT_ONE_TIMEOUT_MS`: Time a normal 1-ball shot, then add margin. Long enough that a good shot never times out, short enough that a misfeed doesn't eat the auto.
- [ ] [RobotCommands.java:43](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L43) `SHOOT_ALL_TIMEOUT_MS`: Time a full burst the same way.
- [ ] [RobotCommands.java:45](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L45) `RETRY_UNJAM_MS`: Jam a ball on purpose and run shootAllWithRetry. Raise until the retry frees it, but balls must not come out of the intake.
- [ ] [RobotCommands.java:47](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L47) `SHOOT_RETRIES`: Each retry can cost RETRY_UNJAM_MS + SHOOT_ALL_TIMEOUT_MS. Set to 0 if retries never fix anything.

### 10. Before competition

- [ ] [MainTeleOp.java:37](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L37) `FULL_TELEMETRY`: Set to false before competition.

## Phase 3: Make it better (11 left)

### 11. Shooting while moving

- [ ] [ShotSolver.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L26) `LEAD`: Only after stationary shots work. Turn on, drive steadily past the HIVE and fire. Lands ahead of the cell = flight times too short; behind = too long.
- [ ] [ShotSolver.java:28](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L28) `PREDICT`: Turn on after LEAD works.
- [ ] [ShotSolver.java:30](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L30) `TURRET_LATENCY_SEC`: With PREDICT on: if the turret lags while driving, raise it; if it aims ahead, lower it.
- [ ] [ShotTables.java:31](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L31) `SHOT_FLIGHT_TIME`: Slow-motion video of shots at near, mid and far distances.
- [ ] [Turret.java:84](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L84) `KV`: Main TeleOp: spin the robot in place. Raise until the turret stops lagging behind the cell.

### 12. Driver feel, feedback and loop time

- [ ] [Feedback.java:28](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/driver/Feedback.java#L28) `WARNING_STRENGTH`: Warning buzz: strong and unmistakable, long enough to feel, short enough not to annoy. Raise the cooldown if a flickering warning still annoys.
- [ ] [Feedback.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/driver/Feedback.java#L36) `FULL_STRENGTH`: Full rumble: soft enough to drive with, strong enough to notice. The gamepad light only works on PlayStation controllers.
- [ ] [Field.java:45](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L45) `CELL_SWITCH_BAND`: Drive across the middle. Raise it if the turret flips between cells too eagerly.
- [ ] [MainTeleOp.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L39) `TELEMETRY_INTERVAL_MS`: Watch Loop ms with everything running, goal under 15. Check again with FULL_TELEMETRY false.
- [ ] [Drive.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Drive.java#L26) `DRIVE_CUBIC`: Drive and adjust DRIVE_CUBIC and TURN_CUBIC to taste. Higher = gentler near center, same top speed.
- [ ] [Shooter.java:98](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L98) `setZeroPowerBehavior()`: After a trim-down the wheel coasts (FLOAT) and atSpeed() blocks shots until it slows. Time a 100 to 200 RPM coast-down; if it blocks shots, tell the programmers to add braking or a small reverse power.

## Phase 4: AgateFlow and auto pickups (25 left)

### 13. AgateFlow setup

- [ ] [AgateFlow.java:70](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L70) `ROBOT_RADIUS`: Measure the farthest point of the robot from the turret center (wedge tips, intake). A circle this big must cover the whole robot.
- [ ] [AgateFlow.java:73](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L73) `ROBOT_FRONT`: Copy the measured Field.CENTER_TO_INTAKE_WALL into ROBOT_FRONT and CENTER_TO_SIDE_WALL into ROBOT_HALF_WIDTH; tape the turret center to the chassis back for ROBOT_BACK.
- [ ] [AgateFlow.java:110](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L110) `CONTACT_RADIUS`: The smallest distance from the turret center to the robot's outside (the back is about 5.7 in.).
- [ ] [DriveModel.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/DriveModel.java#L36) `FORWARD_SPEED_PER_VOLT`: Run DriveModelTuner (after ForesightTuner) and paste its output over these.
- [ ] [DriveModel.java:57](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/DriveModel.java#L57) `TRACTION_ACCEL`: Drive full power forward from a stop on the field tiles. If the wheels spin, lower this until the estimate matches.
- [ ] [TeamTuning.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/TeamTuning.java#L36): Add DriveModelTuner the same way as ForesightTuner (code below). It measures the numbers agateflow/DriveModel needs.

### 14. Pickups: FLOWERS, GARDEN and floor POLLEN

- [ ] [AgateFlow.java:140](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L140) `FLOWER_INTAKE_RAISE_RUN`: Time how long the flower intake takes to come up, and set this to how far the robot backs away from a FLOWER in that time.
- [ ] [AgateFlow.java:170](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L170) `PICKUP_SPEED`: Drive into a POLLEN at rising speeds; set this to the fastest that still picks it up every time.
- [ ] [AgateFlow.java:175](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L175) `INTAKE_DEPTH`: Drive slowly over a POLLEN and measure how far past the intake's front edge it is when the intake grabs it.
- [ ] [IntakePoses.java:30](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L30) `FLOWER_INTAKE_REACH`: Measure on the robot, flower intake down: turret center to where it touches the tiles (ROBOT.md says 275.3 mm).
- [ ] [IntakePoses.java:33](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L33) `FLOWER_REACH_INTO_OPENING`: Drive into a real FLOWER. Raise this if the flower intake doesn't reach the POLLEN, lower it if the robot's front hits the FLOWER.
- [ ] [IntakePoses.java:40](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L40) `FLOWER_APPROACH`: Shorten it if there's no room to line up, lengthen it if the flower intake catches the FLOWER's side.
- [ ] [IntakePoses.java:49](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L49) `GARDEN_RUN_UP`: Raise it if the robot isn't lined up yet when the intake reaches the line.
- [ ] [IntakePoses.java:52](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L52) `GARDEN_END_GAP`: Lower it if the POLLEN in the corner is left behind.
- [ ] [IntakePoses.java:55](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L55) `GARDEN_WALL_GAP`: Raise it if the robot rubs the audience wall on the GARDEN run.
- [ ] [RobotCommands.java:49](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L49) `FLOWER_RAISE_MS`: Time how long the flower intake takes to come all the way up (film it). Set this a little longer.

### 15. AgateFlow on the field

- [ ] [AgateFlow.java:92](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L92) `WALL_GAP`: Raise it if rubbing a wall slows the robot down or drags its heading round.
- [ ] [AgateFlow.java:100](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L100) `WALL_CONTACT_SPEED`: Lower it if the odometry pods skip or the robot jolts when it reaches a wall; raise it if wall runs are slow.
- [ ] [AgateFlow.java:107](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L107) `SAFETY_MARGIN`: Raise if the robot brushes zones on planned paths, lower if paths go needlessly wide.
- [ ] [AgateFlowCommands.java:40](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L40) `TIMEOUT_SCALE`: Watch a few autos: raise if a drive times out close to its target, lower if a stuck robot waits too long.
- [ ] [AgateFlowCommands.java:46](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L46) `TIMEOUT_EXTRA_SECONDS`: Check together with TIMEOUT_SCALE on the robot.
- [ ] [AgateFlowCommands.java:48](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L48) `REPLAN_EXTRA_SECONDS`: Check on the robot with something pushing it off its path: the drive should still end.
- [ ] [AgateFlowCommands.java:66](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L66) `STOP_BRAKING_POWER`: In a test OpMode, add a vision zone 15 in. in front of the robot while it drives at full speed. It should stop short and straight, and the battery reading shouldn't drop under 9 V. Lower this if the wheels skid or the hub browns out.
- [ ] [EtaCalibration.java:16](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/EtaCalibration.java#L16) `TIME_SCALE`: After a few autos with AgateFlow, copy the learned TIME_SCALE from Panels into the code here.
- [ ] [ParkGuard.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/ParkGuard.java#L41) `MARGIN_SECONDS`: Run an auto that runs long and check it ends parked with a little time to spare. Raise if it's late, lower if it parks too early.
