# TODO list

Generated from the `// TODO(step): ...` comments in TeamCode. Don't edit this file. Do the item, delete its TODO comment, and the list updates on the next build (or run `./gradlew todoList`). How to do each step is in TUNING_GUIDE.md.

Most important first: work from the top down. Phase 1 gets the robot playing TeleOp, phase 2 adds a scoring auto, phase 3 makes both better, phase 4 adds AgateFlow and auto pickups, phase 5 is the coax hood once it is on the robot.

**162 left.**

## Phase 1: TeleOp, bare minimum (61 left)

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
- [ ] [Rail.java:29](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Rail.java#L29) `REVERSE_POWER`: Storage Test, hold Circle with balls loaded: they must back out without flying out of the intake. Lower it (toward 0) if they do.
- [ ] [Storage.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Storage.java#L21) `BLOCKED_READS_LOW`: Storage Test: each beam must show true when blocked by hand. Flip if backwards.
- [ ] [Storage.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Storage.java#L24) `DEBOUNCE_MS`: Raise (in ms) if the ball count flickers.
- [ ] [Storage.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Storage.java#L26) `FULL_HOLD_MS`: Feed balls one at a time. Full must appear only on the 4th.

### 4. Servos

- [ ] [Door.java:18](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Door.java#L18) `OPEN_TIME_MS`: Closed to fully open. Slow-motion video in Shooter Test (wheel off, hold RB), or after step 6 lower it until a ball catches the door.
- [ ] [Door.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Door.java#L21) `CLOSE_TIME_MS`: Fully open to closed. Slow-motion video in Shooter Test (let go of RB), or after step 6 lower it in Main TeleOp until a ball reaches the door while it's still closing.
- [ ] [FlowerIntake.java:20](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/FlowerIntake.java#L20) `UP_POSITION`: Find up and down with Servo Position Finder, on the left servo.
- [ ] [FlowerIntake.java:22](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/FlowerIntake.java#L22) `DOWN_POSITION`: Check it reaches into a real FLOWER's Retrieval Opening and pulls POLLEN out.
- [ ] [FlowerIntake.java:24](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/FlowerIntake.java#L24) `RIGHT_REVERSED`: LB in Main TeleOp must move both servos the same way. Flip if they fight.

### 5. Turret

- [ ] [Turret.java:72](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L72) `ENCODER_FULL_TURN_VOLTS`: Turret Test, manual: turn slowly through full turns. Use the highest clean volts before it drops to 0.
- [ ] [Turret.java:75](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L75) `ENCODER_MIN_VOLTS`: Turret Test, manual, same slow turns: the lowest clean volts just after it drops. Leave 0 if it reads 0.000 there.
- [ ] [Turret.java:84](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L84) `ENCODER_ZERO_DEG`: Point the shooter straight out the front, read the angle, put it here.
- [ ] [Turret.java:87](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L87) `ENCODER_REVERSED`: The angle must increase when the turret turns counter-clockwise.
- [ ] [Turret.java:89](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L89) `SERVO_REVERSED`: Turret Test, manual: stick left must turn the turret counter-clockwise (seen from above).
- [ ] [Turret.java:92](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L92) `FAR_KP`: Aiming mode, D-pad 0 to 180: tune FAR_KP and FAR_KD for fast moves with a small overshoot.
- [ ] [Turret.java:95](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L95) `CLOSE_KP`: Aiming mode, bumper nudges: tune CLOSE_KP and CLOSE_KD so it settles without buzzing.
- [ ] [Turret.java:98](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L98) `CLOSE_ZONE_DEG`: Aiming mode: if it chatters near the target, adjust this.
- [ ] [Turret.java:103](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L103) `KI`: Aiming mode, bumper nudges: if it stops a degree or two short of the target and stays there, raise KI until it closes the gap without overshoot. If it never stops short, leave it at 0.
- [ ] [Turret.java:106](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L106) `I_ZONE_DEG`: Aiming mode, with KI on: set this a little above the biggest gap where it used to stop short, so KI only works near the target.
- [ ] [Turret.java:109](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L109) `I_MAX_POWER`: Aiming mode, with KI on: start a little above KS. Lower it if KI makes the turret jump past the target.
- [ ] [Turret.java:112](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L112) `D_FILTER`: Aiming mode: only if the turret buzzes or "Turret P / I / D / F / KS" shows D jittering while it holds still, raise this toward 0.5. Too high and it overshoots more.
- [ ] [Turret.java:116](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L116) `KS`: Manual mode (KS isn't used there): push the stick slowly, note the power where the turret starts to move, set KS a bit below it.
- [ ] [Turret.java:118](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L118) `KS_RAMP_DEG`: Aiming mode: if it chatters near the target, adjust this. Higher = KS fades in over a wider zone.
- [ ] [Turret.java:129](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L129) `MAX_POWER`: Lower if the gearing or wiring complains.
- [ ] [Turret.java:132](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L132) `ON_TARGET_DEG`: Largest aim error that still scores. Find it once the shooter works.
- [ ] [Turret.java:135](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L135) `WATCHDOG_MS`: Aiming mode, D-pad 0 to 180: if a normal big move trips the watchdog, raise this time.

### 6. Shooter

- [ ] [FireControl.java:58](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L58) `READY_HOLD_MS`: Main TeleOp: raise if bursts start while the turret is still settling.
- [ ] [FireControl.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L61) `BURST_RPM_FLOOR`: Main TeleOp, hold RB with 4 balls: watch the lowest RPM during the burst. Set this well past that sag, so a normal burst never pauses; lower it if the last balls of a burst fall short. 0 turns the check off.
- [ ] [FireControl.java:64](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L64) `DOOR_CLOSE_DELAY_MS`: Main TeleOp: let go of RB in the middle of a burst. Raise it if the door closes on a ball; lower it if the door stays open longer than it needs to.
- [ ] [FireControl.java:67](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L67) `SHOT_DIP_RPM`: Shooter Test: watch RPM as a ball goes through. Set a bit under the smallest dip you see.
- [ ] [FireControl.java:75](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L75) `DIP_WINDOW_MS`: Shooter Test: longer than a ball's dip takes to reach SHOT_DIP_RPM (watch the RPM as a ball goes through), and far shorter than the wheel takes to coast down SHOT_DIP_RPM with the power off (Cross turns the wheel off: do it at full speed and time it).
- [ ] [FireControl.java:78](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L78) `NO_SHOT_WARN_MS`: Main TeleOp, hold RB with balls loaded: a bit longer than the slowest time from the rail starting to the first shot counted.
- [ ] [Shooter.java:40](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L40) `LEFT_REVERSED`: The motors are geared together: test each alone at low power first (Shooter Test, hold Square or Circle). Both must push the shooting direction.
- [ ] [Shooter.java:43](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L43) `ENCODER_REVERSED`: RPM must read positive while shooting. The encoder must be on the left motor's port. This flag alone sets the RPM sign, LEFT_REVERSED does not change it.
- [ ] [Shooter.java:46](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L46) `KS`: Raise if low RPMs sit below target.
- [ ] [Shooter.java:48](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L48) `KV`: Shooter Test with KS and KP at 0: raise until the wheel settles on target at 2000, 3500 and 5000 RPM.
- [ ] [Shooter.java:51](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L51) `KP`: Raise until it recovers fast without oscillating.
- [ ] [Shooter.java:67](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L67) `RPM_TOLERANCE`: Largest RPM error that still scores at mid range. Not under 90: the RPM reading moves in steps of about 43.
- [ ] [Shooter.java:74](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L74) `BOOST_BELOW_RPM`: Fire 4 balls: quick recovery between balls, no big overshoot.
- [ ] [Shooter.java:85](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L85) `WATCHDOG_MS`: Only if the watchdog trips on a normal spin-up: raise this, or lower WATCHDOG_MIN_RPM.
- [ ] [Shooter.java:138](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L138) `setZeroPowerBehavior()`: Shooter Test, wheel on: D-pad down once or twice and time the 100 to 200 RPM coast-down. Driving toward the HIVE and trimming down both lower the target, and atSpeed() holds off a burst until the wheel gets there. If that takes too long, tell the programmers to add braking or a small reverse power.

### 7. Shot tables

- [ ] [ShotTables.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L26) `SHOT_RPM`: Shooter Test at measured distances, about every 10 in. from 20 to 110. Check a few with the other cell up.
- [ ] [ShotTables.java:42](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L42) `SHOT_FEED_POWER`: Lower at long range if back-to-back balls miss because RPM hasn't recovered.

### 8. Field and relocalizing

- [ ] [Field.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L61) `CENTER_TO_INTAKE_WALL`: CAD says 203 mm. Check with a tape: intake flat on a wall, wall to turret center. Is the stowed flower intake further out?
- [ ] [Field.java:64](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L64) `CENTER_TO_SIDE_WALL`: CAD outer frame rails say 149 mm (298 wide, not 280). Check with a tape.
- [ ] [Field.java:90](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L90) `cornerPose()`: Relocalize in all 4 corners, drive to a taped spot, and check the pose.

## Phase 2: Autonomous and competition (12 left)

### 9. First autonomous

- [ ] [AutoTemplate.java:32](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L32): Copy this file for a real auto, fill in the poses and the routine, and delete the @Disabled line.
- [ ] [AutoTemplate.java:40](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L40) `START_X`: Measure where the robot starts on the field. Center of the turret, with the robot sitting in place.
- [ ] [AutoTemplate.java:44](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L44) `SHOOT_X`: Pick where to shoot from and check the distance to the cell is inside the ShotTables range.
- [ ] [AutoTemplate.java:48](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L48) `DRIVE_TIMEOUT_MS`: Time the drive to the shooting spot, then add margin. Pedro never gives up on a path by itself, so without a limit a blocked robot waits there all auto.
- [ ] [AutoTemplate.java:85](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoTemplate.java#L85) `schedule()`: Replace this placeholder with the real routine.
- [ ] [RobotCommands.java:42](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L42) `SHOOT_ONE_TIMEOUT_MS`: Time a normal 1-ball shot, then add margin. Long enough that a good shot never times out, short enough that a misfeed doesn't eat the auto.
- [ ] [RobotCommands.java:44](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L44) `SHOOT_ALL_TIMEOUT_MS`: Time a full burst the same way.
- [ ] [RobotCommands.java:46](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L46) `LAST_BALL_MS`: shootAll with 4 balls: watch the last ball. Raise it until the last ball always reaches the wheel before the rail stops; lower it to end bursts sooner.
- [ ] [RobotCommands.java:52](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L52) `RETRY_UNJAM_MS`: Jam a ball on purpose and run shootAllWithRetry. Raise until the retry frees it, but balls must not come out of the intake.
- [ ] [RobotCommands.java:54](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L54) `SHOOT_RETRIES`: Each retry can cost RETRY_UNJAM_MS + SHOOT_ALL_TIMEOUT_MS. Set to 0 if retries never fix anything.

### 10. Before competition

- [ ] [MainTeleOp.java:37](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L37) `FULL_TELEMETRY`: Set to false before competition.
- [ ] [PanelsSettings.java:21](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/PanelsSettings.java#L21) `EVENT_BUILD`: set true for event builds so Panels never starts (R704.D).

## Phase 3: Make it better (23 left)

### 11. Shooting while moving

- [ ] [ShotSolver.java:51](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L51) `LEAD`: Only after stationary shots work. Turn on, drive steadily past the HIVE and fire. Lands ahead of the cell = flight times too short; behind = too long.
- [ ] [ShotSolver.java:53](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L53) `PREDICT`: Turn on after LEAD works.
- [ ] [ShotSolver.java:55](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L55) `TURRET_LATENCY_SEC`: With PREDICT on: if the turret lags while driving, raise it; if it aims ahead, lower it.
- [ ] [ShotSolver.java:58](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L58) `RELEASE_LATENCY_SEC`: About one loop (Loop ms in telemetry, 0.02 to 0.04 s). Not the feed time: the aim keeps updating while the ball is fed.
- [ ] [ShotSolver.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L61) `VELOCITY_LATENCY_SEC`: With LEAD and PREDICT on, drive past the HIVE, brake hard and fire as it stops. Lands behind the cell (along the way it drove) = raise it; ahead = lower it.
- [ ] [ShotSolver.java:64](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L64) `USE_ACCEL`: Only if the aim twitches while driving at a steady speed: raise ACCEL_FILTER toward 0.9, or set this false.
- [ ] [ShotSolver.java:69](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L69) `MAX_ACCEL`: Only if hard brakes still land behind the cell after VELOCITY_LATENCY_SEC: raise it toward DriveModel.TRACTION_ACCEL.
- [ ] [ShotSolver.java:72](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotSolver.java#L72) `MAX_ANGULAR_ACCEL`: Only if PREDICT's turret aim lags when the robot starts or stops spinning hard: raise it. If the aim twitches while turning steadily, lower it.
- [ ] [ShotTables.java:35](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L35) `SHOT_FLIGHT_TIME`: Slow-motion video of shots at near, mid and far distances.
- [ ] [Shooter.java:53](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L53) `KA`: Main TeleOp: drive straight toward and away from the HIVE while watching Shooter RPM. Raise from 0 until the wheel keeps up with the target without overshooting it.
- [ ] [Shooter.java:56](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L56) `TARGET_RATE_FILTER`: Raise (smoother) if the kA part jumps around; lower if it reacts too late.
- [ ] [Shooter.java:59](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L59) `MAX_TARGET_RATE`: With KA on, drive full speed straight at the HIVE and watch "target changing" in telemetry. Set this a little above the biggest number you see.
- [ ] [Turret.java:120](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L120) `KV`: Main TeleOp: spin the robot in place. Raise until the turret stops lagging behind the cell.
- [ ] [Turret.java:123](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L123) `TARGET_RATE_FILTER`: Main TeleOp, while tuning KV: if the turret jerks when the robot starts or stops turning, raise this toward 0.8. 0 = no smoothing.
- [ ] [Turret.java:126](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Turret.java#L126) `MAX_TARGET_RATE_DEG_PER_SEC`: Time a full-stick spin in place (degrees divided by seconds) and set this a little above it.

### 12. Driver feel, feedback and loop time

- [ ] [Feedback.java:28](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/driver/Feedback.java#L28) `WARNING_STRENGTH`: Warning buzz: strong and unmistakable, long enough to feel, short enough not to annoy. Raise the cooldown if a flickering warning still annoys.
- [ ] [Feedback.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/driver/Feedback.java#L36) `FULL_STRENGTH`: Full rumble: soft enough to drive with, strong enough to notice. The gamepad light only works on PlayStation controllers.
- [ ] [Field.java:45](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L45) `CELL_SWITCH_BAND`: Drive across the middle. Raise it if the turret flips between cells too eagerly.
- [ ] [MainTeleOp.java:39](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L39) `TELEMETRY_INTERVAL_MS`: Watch Loop ms with everything running, goal under 15. Check again with FULL_TELEMETRY false.
- [ ] [MainTeleOp.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L41) `TURRET_TRIM_STEP_DEG`: Driver 2 practice: one press should fix a typical miss. Raise if it takes several presses, lower if one press overshoots.
- [ ] [MainTeleOp.java:43](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L43) `RPM_TRIM_STEP`: Same as the turret trim, for shots that land short or long (static hood).
- [ ] [MainTeleOp.java:48](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L48) `TRIGGER_THRESHOLD`: Raise it if a resting finger fires RT or LT; lower it if a full pull doesn't register.
- [ ] [Drive.java:26](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Drive.java#L26) `DRIVE_CUBIC`: Drive and adjust DRIVE_CUBIC and TURN_CUBIC to taste. Higher = gentler near center, same top speed.

## Phase 4: AgateFlow and auto pickups (36 left)

### 13. AgateFlow setup

- [ ] [AgateFlow.java:69](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L69) `ROBOT_RADIUS`: Measure the farthest point of the robot from the turret center (wedge tips, intake). A circle this big must cover the whole robot.
- [ ] [AgateFlow.java:72](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L72) `ROBOT_FRONT`: Confirm ROBOT_FRONT and ROBOT_HALF_WIDTH equal the measured Field.CENTER_TO_INTAKE_WALL and CENTER_TO_SIDE_WALL (step 8), and tape the turret center to the chassis back for ROBOT_BACK.
- [ ] [AgateFlow.java:113](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L113) `CONTACT_RADIUS`: The smallest distance from the turret center to the robot's outside (the back is about 5.7 in.).
- [ ] [AgateFlow.java:167](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L167) `PLAN_TIME_LIMIT_MS`: Run the AgateFlow Benchmark on the Control Hub. Raise this if many plans hit it and come out slower than on the desktop; lower it if replans come back too late while driving.
- [ ] [DriveModel.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/DriveModel.java#L36) `FORWARD_SPEED_PER_VOLT`: Run DriveModelTuner (after ForesightTuner) and paste its output over these.
- [ ] [DriveModel.java:57](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/DriveModel.java#L57) `TRACTION_ACCEL`: Drive full power forward from a stop on the field tiles. If the wheels spin, lower this until the estimate matches.
- [ ] [DriveModel.java:60](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/DriveModel.java#L60) `SETTLE_SECONDS`: Drive a few short paths and time from the robot reaching the end to the follower saying it is done. Set this to that.
- [ ] [TeamTuning.java:36](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/TeamTuning.java#L36): Add DriveModelTuner the same way as ForesightTuner (code below). It measures the numbers agateflow/DriveModel needs.

### 14. Pickups: FLOWERS, GARDEN and floor POLLEN

- [ ] [AgateFlow.java:143](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L143) `FLOWER_INTAKE_RAISE_RUN`: Time how long the flower intake takes to come up, and set this to how far the robot backs away from a FLOWER in that time.
- [ ] [AgateFlow.java:174](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L174) `PICKUP_SPEED`: Drive into a POLLEN at rising speeds; set this to the fastest that still picks it up every time.
- [ ] [AgateFlow.java:177](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L177) `INTAKE_HALF_WIDTH`: Measure the intake's opening across (where a POLLEN gets grabbed) and set this to half of it.
- [ ] [AgateFlow.java:180](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L180) `INTAKE_DEPTH`: Drive slowly over a POLLEN and measure how far past the intake's front edge it is when the intake grabs it.
- [ ] [AgateFlow.java:185](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L185) `PICKUP_RUN`: Pick up POLLEN that sits at an angle to the robot's way in. Raise this if the intake hits them crooked and misses, lower it if pickUp routes are slow.
- [ ] [AgateFlow.java:188](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L188) `PICKUP_EXIT`: Watch a pickUp route: raise this if the robot speeds up before the POLLEN is all the way in, lower it if it stays slow for long after.
- [ ] [AgateFlow.java:191](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L191) `PICKUP_SIDE_MARGIN`: Run pickUp routes on the field: raise this if POLLEN gets missed at the intake's edges, lower it if Plan.skipped() leaves out ones the intake could get.
- [ ] [IntakePoses.java:30](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L30) `FLOWER_INTAKE_REACH`: Measure on the robot, flower intake down: turret center to where it touches the tiles (ROBOT.md says 275.3 mm).
- [ ] [IntakePoses.java:33](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L33) `FLOWER_REACH_INTO_OPENING`: Drive into a real FLOWER. Raise this if the flower intake doesn't reach the POLLEN, lower it if the robot's front hits the FLOWER.
- [ ] [IntakePoses.java:40](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L40) `FLOWER_APPROACH`: Shorten it if there's no room to line up, lengthen it if the flower intake catches the FLOWER's side.
- [ ] [IntakePoses.java:49](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L49) `GARDEN_RUN_UP`: Raise it if the robot isn't lined up yet when the intake reaches the line.
- [ ] [IntakePoses.java:52](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L52) `GARDEN_END_GAP`: Lower it if the POLLEN in the corner is left behind.
- [ ] [IntakePoses.java:55](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/IntakePoses.java#L55) `GARDEN_WALL_GAP`: Raise it if the robot rubs the audience wall on the GARDEN run.
- [ ] [RobotCommands.java:56](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/commands/RobotCommands.java#L56) `FLOWER_RAISE_MS`: Time how long the flower intake takes to come all the way up (film it). Set this a little longer.

### 15. AgateFlow on the field

- [ ] [AgateFlow.java:95](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L95) `WALL_GAP`: Raise it if rubbing a wall slows the robot down or drags its heading round.
- [ ] [AgateFlow.java:103](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L103) `WALL_CONTACT_SPEED`: Lower it if the odometry pods skip or the robot jolts when it reaches a wall; raise it if wall runs are slow.
- [ ] [AgateFlow.java:110](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlow.java#L110) `SAFETY_MARGIN`: Raise if the robot brushes zones on planned paths, lower if paths go needlessly wide.
- [ ] [AgateFlowCommands.java:41](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L41) `TIMEOUT_SCALE`: Watch a few autos: raise if a drive times out close to its target, lower if a stuck robot waits too long.
- [ ] [AgateFlowCommands.java:47](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L47) `TIMEOUT_EXTRA_SECONDS`: Check together with TIMEOUT_SCALE on the robot.
- [ ] [AgateFlowCommands.java:49](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L49) `REPLAN_EXTRA_SECONDS`: Check on the robot with something pushing it off its path: the drive should still end.
- [ ] [AgateFlowCommands.java:55](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L55) `MAX_DETOURS`: In a test OpMode, drop a vision zone in the robot's way a few times in one drive. It should go round each one and still end.
- [ ] [AgateFlowCommands.java:63](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L63) `MAX_OVERTIME_SECONDS`: Watch drives that run past their deadline: raise this if one still closing in on its stop gives up, lower it if a crawling robot holds up the auto.
- [ ] [AgateFlowCommands.java:69](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L69) `STOP_BRAKING_POWER`: In a test OpMode, add a vision zone 15 in. in front of the robot while it drives at full speed. It should stop short and straight, and the battery reading shouldn't drop under 9 V. Lower this if the wheels skid or the hub browns out.
- [ ] [AgateFlowCommands.java:81](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/AgateFlowCommands.java#L81) `REPLAN_OFF_PATH`: Push the robot off its path by hand while it drives: it should replan once it is this far off, and not from normal wobble.
- [ ] [EtaCalibration.java:19](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/EtaCalibration.java#L19) `TIME_SCALE`: After a few autos with AgateFlow, copy the learned TIME_SCALE from Panels into the code here.
- [ ] [ParkGuard.java:48](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/ParkGuard.java#L48) `MARGIN_SECONDS`: Run an auto that runs long and check it ends parked with a little time to spare. Raise if it's late, lower if it parks too early.
- [ ] [ParkGuard.java:51](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/ParkGuard.java#L51) `REFRESH_MS`: Run an auto that replans a lot. Raise this if replans come back late (they share the planning thread with park estimates), lower it if parking starts much earlier than needed.
- [ ] [ParkGuard.java:54](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/agateflow/ParkGuard.java#L54) `FALLBACK_SECONDS`: Block the park route with a vision zone in a test auto and check the robot still parks in time once it is clear.

## Phase 5: Coax hood (once it is on the robot) (30 left)

### 16. Coax hood

- [ ] [Field.java:53](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/Field.java#L53) `PASS_TARGET_RED_X`: Once passing is on (FireControl.PASS_ENABLED): pick where passes should land in our LOADING ZONE, set X and Y, and check real passes land there.
- [ ] [MainTeleOp.java:45](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java#L45) `HOOD_TRIM_STEP_DEG`: Main TeleOp, COAX: one press should fix a typical high or low miss. Raise if it takes several presses, lower if one overshoots.
- [ ] [FireControl.java:50](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L50) `PASS_ENABLED`: Once coax shots work, turn on and pass from around the field. Check no pass can leave the field (G405).
- [ ] [FireControl.java:68](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FireControl.java#L68) `SHOT_DIP_RPM`: Coax Hood Test, physics mode: check it again with the wheel anywhere in the 300 RPM ready window (Robot.COAX_RPM_TOLERANCE).
- [ ] [Robot.java:46](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java#L46) `HOOD`: Switch to COAX once the coax hood is on the robot and its servo is in the configuration.
- [ ] [Robot.java:49](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java#L49) `COAX_RPM_TOLERANCE`: Main TeleOp, hold RB with 4 balls: raise for faster bursts while the hood makes up for the dip; lower if balls fired during recovery miss.
- [ ] [HardwareNames.java:27](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/hardware/HardwareNames.java#L27) `HOOD_SERVO`: When the coax hood goes on, add a servo with this name to the configuration ("Full Range Servo" for goBILDA or Axon) and write its port in ROBOT.md.
- [ ] [HoodSolver.java:77](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/HoodSolver.java#L77) `LAUNCH_HEIGHT`: Measure the ball center's height above the tiles at the moment it leaves the wheel.
- [ ] [HoodSolver.java:82](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/HoodSolver.java#L82) `TARGET_BAND`: Smaller = pickier, more accurate shots; larger = shoots in more situations. Keep it above about 2 while RPM_MARGIN is 50 (the margin moves the crossing 1 to 2 in.), or lower both together.
- [ ] [HoodSolver.java:87](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/HoodSolver.java#L87) `LIP_HEIGHT`: Only if shots clip the CELL's bottom edge: raise it a little, and check LIP_BEFORE against the real CELL.
- [ ] [HoodSolver.java:95](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/HoodSolver.java#L95) `RPM_MARGIN`: Raise it if shots fired while the wheel recovers miss; lower it if the hood refuses shots it would make.
- [ ] [HoodSolver.java:102](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/HoodSolver.java#L102) `SEED_RPM_WINDOW`: Keep it equal to Robot.COAX_RPM_TOLERANCE (both 300 now): the seeds leave the hood room for exactly the RPM error a burst may start with.
- [ ] [ShotTables.java:54](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L54) `PASS_RPM`: Once passing is on (FireControl.PASS_ENABLED): fill it from real passes like SHOT_RPM. Its first and last distances also limit where coax passes fire (G405): check a pass from the far end can't leave the field.
- [ ] [ShotTables.java:60](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L60) `PASS_FEED_POWER`: Once passing is on: lower it if back-to-back passes land short or scatter.
- [ ] [ShotTables.java:67](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L67) `COAX_RPM`: Seeds from physics. On the field: if shots from a distance are too steep and slow, raise the RPM there; if too flat, lower it.
- [ ] [ShotTables.java:89](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L89) `COAX_EXIT_SPEED`: Slow-motion video of a shot at a known hood angle and RPM: (forward in/s) / cos(angle) / RPM. Or in the Coax Hood Test: balls land high at an RPM = they're faster than this says, so raise it at that RPM; low = lower it.
- [ ] [ShotTables.java:99](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/shot/ShotTables.java#L99) `COAX_EXIT_SPEED_ANGLE`: Calibrate COAX_EXIT_SPEED at one angle first. Then fire at a flat and a steep angle at the same RPM: balls land high at an angle = raise that angle's multiplier; low = lower it.
- [ ] [CoaxHood.java:52](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L52) `GEAR_RATIO`: The gearing isn't built yet. Set this when it is: teeth on the hood's gear divided by teeth on the servo's gear.
- [ ] [CoaxHood.java:61](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L61) `ANCHOR_DEG`: First. Coax Hood Test, angle mode, Circle (raw servo): nudge the hood until it just touches its flat stop. Measure the launch angle there (phone level on the hood's exit lip) and put it here.
- [ ] [CoaxHood.java:64](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L64) `SERVO_AT_ANCHOR`: First, same spot as ANCHOR_DEG: the raw servo position the test shows there.
- [ ] [CoaxHood.java:67](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L67) `REVERSED`: Second. Coax Hood Test, angle mode: D-pad up must make the hood steeper. If it gets flatter (or pushes into the flat stop), flip this.
- [ ] [CoaxHood.java:70](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L70) `SERVO_RANGE_DEG`: Third. Start from the servo's spec. Then angle mode: command a steep angle short of the steep stop and measure it. Steeper than commanded: raise this. Flatter: lower it (or GEAR_RATIO is off).
- [ ] [CoaxHood.java:79](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L79) `MIN_LAUNCH_DEG`: Fourth. A degree or two steeper than the flat stop's angle (ANCHOR_DEG, if the anchor is at the stop), so the servo never pushes into it.
- [ ] [CoaxHood.java:82](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L82) `MAX_LAUNCH_DEG`: Fourth. Raw servo (Circle): move the hood to its steep stop and measure the angle there. Set this a degree or two flatter.
- [ ] [CoaxHood.java:85](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L85) `SPEED_DEG_PER_SEC`: Fifth, after MIN and MAX. Coax Hood Test, angle mode: Triangle jumps between MIN and MAX. Time it with slow-motion video, (MAX - MIN) / seconds. Round down, so atTarget() is never early.
- [ ] [CoaxHood.java:94](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L94) `ON_ANGLE_DEG`: Main TeleOp, COAX: raise it if bursts wait on "moving" while the hood angle barely changes; lower it if shots fire before the hood has settled.
- [ ] [CoaxHood.java:98](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L98) `STATIC_LAUNCH_DEG`: Measure the static hood's launch angle (phone level on its exit lip).
- [ ] [CoaxHood.java:101](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L101) `IDLE_LAUNCH_DEG`: Main TeleOp, COAX: set it near the angle most shots use, so the hood has the least to move when a shot comes up.
- [ ] [CoaxHood.java:104](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/CoaxHood.java#L104) `PASS_LAUNCH_DEG`: Once passing is on (FireControl.PASS_ENABLED): higher lobs over robots, lower ones get there sooner.
- [ ] [Shooter.java:63](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java#L63) `AVERAGE_SAMPLES`: Coax hood: if the hood shakes with the wheel's speed reading, raise it. Each sample is one loop; more = steadier but later.
