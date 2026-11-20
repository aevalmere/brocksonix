package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.shot.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.shot.ShotSolver;
import org.firstinspires.ftc.teamcode.robot.shot.ShotTables;
import org.firstinspires.ftc.teamcode.robot.shot.TurnPredictor;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.firstinspires.ftc.teamcode.util.PIDF;

/**
 * Continuous shooting with full auto aim (turret and RPM from the pose), using the match
 * subsystems (Intake, Transfer, Shooter, Door, Turret). It needs no Foresight tuning, so it
 * runs before Main TeleOp can. No shooting on the move yet.
 *
 * Break beams: one line per beam, in the order a ball passes them (slot 4, slot 3, slot 1),
 * says BLOCKED or clear (Storage.BLOCKED_READS_LOW) and the raw level, live. Each beam is
 * optional: one missing from the configuration shows "not configured". Breakbeam Test shows
 * more per beam (pass counts, how long the last block lasted), with nothing else running.
 *
 * Rail, the whole ball path as Robot.update() runs it, with the real Storage (its debounce
 * and full rule). First match wins:
 * - LT: intake and transfer reverse, door closed, whatever else is held.
 * - RB held with the wheel on, door fully open: both feed at ShooterTest.FEED_POWER.
 * - RB held but the door still opening, or the door still closing after a shot: both stop,
 *   so no ball reaches a half-open door (Robot does the same).
 * - Otherwise they collect. The intake runs until storage is full (Storage.isFull(): all
 *   three beams blocked), then stops, and runs again as soon as any beam clears. The transfer runs until the slot 1 beam sees a ball, then
 *   holds it: with the door, at Transfer.HOLD_POWER against it (Transfer.collect, as in the
 *   match; set HOLD_POWER to 0 for a dead stop); with ShooterTest.USE_DOOR off (no door),
 *   it stops, since nothing else keeps the ball out of the wheel. When slot 1 clears (a
 *   ball was shot), it runs again and brings the next ball up.
 * Storage needs all three beams. With one missing the rail runs as before the beams were
 * used, and the Intake and Transfer lines say which beam is missing: the intake always
 * collects, and the transfer holds at HOLD_POWER against the door, or with no door stays
 * stopped (so balls only move up while RB feeds).
 * The "Intake" and "Transfer" lines say what each motor is doing and why.
 *
 * Wheel: Cross turns it on and off. It runs closed loop at the static hood's RPM for the angle
 * and distance the robot sits at from the cell (ShotTables.staticShotRpm, through ShotSolver), with the
 * boost while firing, like Shooter Test. The turret aims at the cell shifted toward the HIVE
 * wall that faces the robot (ShotTables.AIM_SHIFT_IN). Outside SHOT_MIN_DISTANCE to
 * SHOT_MAX_DISTANCE the distance line says so (Main TeleOp would not fire there). D-pad up/down
 * trims by ShooterTest.RPM_STEP_BIG on top, to check a spot: stand on it, trim until it scores,
 * and put the pose and RPM + trim into the nearest of ShotTables' seven spots.
 * Check each shooter motor alone at low power in Shooter Test before the first run.
 *
 * Turret tracking: the pose comes from TestPose. It carries over from the last OpMode, and
 * driver 2 relocalizes with Main TeleOp's buttons. Square turns tracking on/off: on, the
 * turret keeps pointing at the cell (FAR_CELL; D-pad left/right picks the cell on the
 * driver's left or right) as the robot drives and turns. Off, the turret servo stops.
 * Tracking is predictive through the same TurnPredictor.aim() Main TeleOp uses (Triangle
 * turns the prediction off and on): the turret leads the turn the stick asks for, and the
 * predicted turn rate goes in as its feedforward (Turret.KV). Compare "Yaw rate predicted /
 * measured" to tune TurnPredictor (Panels page "TurnPredictor"). Tune the turret in Turret
 * Test first (step 5).
 *
 * Driving: Drive Test's stick code, field-centric (stick up = away from the driver's alliance
 * wall, like Main TeleOp); Circle switches to robot-centric and back. With no heading
 * (NaN pose) it drives robot-centric. RT (hold) is slow mode.
 *
 * Pinpoint check: the "Pinpoint" line shows the device status, its loop time and firmware.
 * If the heading freezes, read it before power cycling: READY with a loop time means the
 * Pinpoint runs but its IMU stopped; FAULT_BAD_READ or NOT_READY means the whole device
 * stopped answering (its cable or its power). Share, with the robot still, resets the
 * Pinpoint and its IMU and puts the pose back where it was half a second later.
 *
 * Panels field (practice only): the robot outline, the goal, a yellow line to the goal and
 * an orange line where the turret points. On target, the two lines lie on top of each other.
 */
@Configurable
@TeleOp(name = "Continuous Test", group = "Test")
public class ContinuousTest extends LinearOpMode {
    /** Which cell the turret tracks: true = FAR cell, false = AUDIENCE cell. */
    public static boolean FAR_CELL = false;
    /** Smooths "Yaw rate measured" (only telemetry): 0 = none, closer to 1 = smoother. */
    public static double YAW_RATE_FILTER = 0.5;
    /** How long Share's Pinpoint reset waits before setting the pose (the IMU takes about 0.25 s). */
    public static double PINPOINT_RESET_WAIT_SEC = 0.5;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);
        // Moves nothing in init: Mecanum only sets directions and float mode, the Pinpoint and
        // the turret only read, and the other subsystems write only when told to.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        TestPose testPose = new TestPose(hardwareMap, gamepad1, gamepad2);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, voltage);
        // Null while ShooterTest.USE_DOOR is off: the door servo isn't even looked up.
        Door door = ShooterTest.USE_DOOR ? new Door(hardwareMap) : null;
        Turret turret = new Turret(hardwareMap, voltage);
        // The same driver object the localizer uses, only for its status and Share's reset.
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        int pinpointFirmware = pinpoint.getDeviceVersion();
        // Break beams in ball-travel order: slot 4 (intake end) first, slot 1 (door) last. No slot 2 beam.
        DigitalChannel[] beams = TestBeams.find(hardwareMap);
        // The match's Storage runs the rail, but only with all three beams (it would throw otherwise).
        String missingBeams = Storage.missingBeams(hardwareMap);
        Storage storage = missingBeams.isEmpty() ? new Storage(hardwareMap) : null;

        screen.addLine(testPose.startDescription());
        for (int i = 0; i < beams.length; i++) {
            screen.addData(TestBeams.label(i), beams[i] == null ? "not configured" : "found");
        }
        if (storage == null) screen.addLine("!! Missing " + missingBeams + ": the rail won't stop by itself at slot 1 or when full.");
        screen.addLine("Ready. Cross: wheel on. Hold RB: fire. Square: turret tracking.");
        screen.update();

        waitForStart();
        gamepad1.resetEdgeDetection();
        testPose.start();
        Alliance alliance = TestPose.alliance();
        boolean wheelOn = false;
        boolean tracking = false;
        boolean fieldCentric = true;
        ShotSolver solver = new ShotSolver();
        TurnPredictor turnPredictor = new TurnPredictor();
        ElapsedTime loopTimer = new ElapsedTime();
        double lastHeading = 0;
        boolean hadHeading = false;
        double measuredYawRate = 0;
        double tableRpm = 0; // the last good one, kept if the pose goes NaN
        double rpmTrim = 0;

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Read: everything this loop decides from is this loop's.
            bulkReads.clear();
            if (storage != null) storage.update();
            shooter.readSensors();
            turret.readSensors();
            // Robot must be still: the Pinpoint zeroes its gyro while it calibrates.
            if (gamepad1.shareWasPressed()) testPose.resetPinpoint(pinpoint, PINPOINT_RESET_WAIT_SEC);
            Pose pose = testPose.update();
            boolean poseOk = !Double.isNaN(pose.x()) && !Double.isNaN(pose.y()) && !Double.isNaN(pose.heading());

            // Buttons.
            if (gamepad1.crossWasPressed()) wheelOn = !wheelOn;
            if (gamepad1.squareWasPressed()) tracking = !tracking;
            if (gamepad1.triangleWasPressed()) TurnPredictor.ENABLED = !TurnPredictor.ENABLED;
            if (gamepad1.circleWasPressed()) fieldCentric = !fieldCentric;
            if (gamepad1.dpadLeftWasPressed()) FAR_CELL = Field.cellFromDriver(alliance, true) == Cell.FAR;
            if (gamepad1.dpadRightWasPressed()) FAR_CELL = Field.cellFromDriver(alliance, false) == Cell.FAR;
            double trimStep = 0;
            if (gamepad1.dpadUpWasPressed()) trimStep = ShooterTest.RPM_STEP_BIG;
            if (gamepad1.dpadDownWasPressed()) trimStep = -ShooterTest.RPM_STEP_BIG;
            if (trimStep != 0) {
                shooter.addTrim(trimStep);
                rpmTrim += trimStep;
            }
            boolean reversing = gamepad1.left_trigger > MainTeleOp.TRIGGER_THRESHOLD;
            boolean fire = wheelOn && gamepad1.right_bumper && !reversing;

            // Drive first, so the turn predictor knows the turn power going out this loop.
            DrivePowers powers = DriveTest.stickPowers(gamepad1);
            // A NaN heading would make every power NaN, so field-centric waits for a real one.
            if (fieldCentric && poseOk) powers = Drive.driverToField(powers, pose.heading(), alliance);
            drivetrain.drive(powers, true);
            turnPredictor.update(powers, dt);
            if (poseOk) {
                // The real turn rate, from the Pinpoint, to check the predictor against.
                double turned = Angles.wrapDegrees(Math.toDegrees(pose.heading() - lastHeading));
                if (hadHeading && dt > 0) measuredYawRate = PIDF.smooth(measuredYawRate, turned / dt, YAW_RATE_FILTER);
                lastHeading = pose.heading();
            }
            hadHeading = poseOk;

            // Aim and RPM from ShotSolver, as in Main TeleOp with the static hood.
            Pose cell = Field.cellTarget(alliance, FAR_CELL ? Cell.FAR : Cell.AUDIENCE);
            // Zero velocity is how "no shooting on the move" is done: LEAD and PREDICT have nothing to act on.
            ShotSolution shot = poseOk ? solver.solveShot(pose, new Velocity(0, 0, 0), cell) : null;
            if (shot != null) tableRpm = shot.rpm;

            // Turret: the solved aim, led by the predicted turn, the same way Robot aims in Main TeleOp.
            if (tracking && shot != null) {
                turret.setTestPower(null);
                turnPredictor.aim(turret, shot.turretDeg, true);
            } else {
                // Tracking off, or no pose to aim from: the servo stops where it is.
                turret.setTestPower(0.0);
            }

            // Wheel: closed loop, so its watchdog runs too.
            shooter.setTargetRpm(wheelOn ? tableRpm : 0);
            shooter.setBoost(fire);

            // Door and rail. Feeding waits for the door to be fully open.
            if (door != null) {
                if (fire) door.open();
                else door.close();
            }
            // No door: nothing to wait for.
            boolean doorFullyOpen = door == null || door.isFullyOpen();
            boolean doorFullyClosed = door == null || door.isFullyClosed();
            // What each rail motor is doing and why, for the Intake and Transfer lines.
            String intakeWhy, transferWhy;
            if (reversing) {
                intake.reverse();
                transfer.reverse();
                intakeWhy = transferWhy = "reversing (LT)";
            } else if (fire && doorFullyOpen) {
                intake.feed(ShooterTest.FEED_POWER);
                transfer.feed(ShooterTest.FEED_POWER);
                intakeWhy = transferWhy = "feeding (RB)";
            } else if (fire || !doorFullyClosed) {
                // As in Robot.update(): stopped while the door moves, so no ball meets it half open.
                intake.stop();
                transfer.stop();
                intakeWhy = transferWhy = fire ? "stopped, door opening" : "stopped, door closing";
            } else if (storage == null) {
                // No Storage: the rail can't see the balls, so nothing stops by itself.
                intake.collect(false);
                intakeWhy = "collecting, never stops (missing " + missingBeams + ")";
                if (door == null) {
                    // No door to hold against: the transfer stops, so balls don't creep into the wheel.
                    transfer.stop();
                    transferWhy = "stopped, no door and can't see slot 1 (missing " + missingBeams + ")";
                } else {
                    transfer.collect(true);
                    transferWhy = "holding against the door, no auto-stop (missing " + missingBeams + ")";
                }
            } else {
                // The match's rule: the intake fills storage, the transfer brings a ball up to slot 1.
                boolean full = storage.isFull();
                boolean ballAtDoor = storage.slot1Occupied();
                intake.collect(full);
                intakeWhy = full ? "stopped, full (all 3 beams blocked)" : "collecting";
                if (door == null && ballAtDoor) {
                    // No door: stopping the transfer is the only thing that keeps the ball out of the wheel.
                    transfer.stop();
                    transferWhy = "stopped, slot 1 blocked (no door)";
                } else {
                    transfer.collect(ballAtDoor);
                    transferWhy = ballAtDoor
                            ? String.format("holding at %.2f, slot 1 blocked", Transfer.HOLD_POWER)
                            : "collecting, until slot 1 sees a ball";
                }
            }

            // Write.
            shooter.update();
            if (door != null) door.update();
            turret.update();

            // Panels field: robot, goal, the line to it (yellow) and where the turret points (orange).
            ShooterTest.drawField(poseOk ? pose : null, cell,
                    pose.heading() + Math.toRadians(turret.angleDeg()));

            String tableProblem = ShotTables.orderProblem();
            if (tableProblem != null) screen.addLine("!! " + tableProblem + " (fix in Panels, ShotTables)");
            screen.addData("Wheel (Cross, D-pad up/down: trim)", wheelOn
                    ? String.format("on, table %.0f, trim %+.0f, target %.0f rpm%s", tableRpm, rpmTrim,
                            shooter.effectiveTargetRpm(), shooter.atSpeed() ? ", at speed" : "")
                    : String.format("off (table %.0f, trim %+.0f rpm)", tableRpm, rpmTrim));
            screen.addData("Fire (hold RB)", reversing ? "reversing (LT)"
                    : fire ? (doorFullyOpen ? "feeding" : "door opening")
                    : wheelOn ? "not firing" : "not firing, wheel off");
            screen.addData("Intake", intakeWhy);
            screen.addData("Transfer", transferWhy);
            if (storage != null) screen.addData("Balls (Storage)", storage.countLabel());
            for (int i = 0; i < beams.length; i++) {
                screen.addData(TestBeams.label(i), TestBeams.text(beams[i]));
            }
            if (door != null) door.addTelemetry(screen);
            else screen.addData("Door", "off (ShooterTest.USE_DOOR), not moved");
            screen.addData("Drive (Circle)", !fieldCentric ? "robot-centric"
                    : poseOk ? "field-centric" : "field-centric, no heading: robot-centric for now");
            screen.addData("Slow mode (RT)", DriveTest.slowHeld(gamepad1) ? "on" : "off");
            screen.addData("Turn prediction (Triangle)", TurnPredictor.ENABLED
                    ? String.format("on, lead %.1f deg, feedforward %.0f deg/s",
                            turnPredictor.headingAheadDeg(), turnPredictor.feedforwardRateDegPerSec()) : "off");
            screen.addData("Yaw rate predicted / measured", "%.0f / %.0f deg/s",
                    turnPredictor.rateDegPerSec(), measuredYawRate);
            screen.addData("Goal (D-pad left/right)", FAR_CELL ? "FAR cell" : "AUDIENCE cell");
            screen.addData("Turret tracking (Square)", tracking
                    ? (turret.onTarget() ? "on, on target" : "on, turning") : "off, servo stopped");
            testPose.addTelemetry(screen);
            if (shot != null) {
                screen.addData("Pose", "x %.1f  y %.1f  heading %.0f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                double angle = ShotTables.angleOffFront(pose.x(), pose.y(), cell);
                String tableText = shot.valid
                        ? String.format("%.0f deg off the front, %.0f rpm", angle, shot.rpm)
                        : String.format("%.0f deg off the front, %.0f rpm, but outside %.1f to %.1f in. (Main TeleOp wouldn't fire)",
                                angle, shot.rpm, ShotTables.SHOT_MIN_DISTANCE, ShotTables.SHOT_MAX_DISTANCE);
                screen.addData("Distance to " + (FAR_CELL ? "FAR" : "AUDIENCE") + " cell", "%.1f in., %s", shot.distance, tableText);
            } else {
                screen.addLine("!! Pose is NaN: the Pinpoint is not reading, so the turret can't track. Check its cable and pods.");
            }
            GoBildaPinpointDriver.DeviceStatus pinpointStatus = pinpoint.getDeviceStatus();
            screen.addData("Pinpoint (Share: reset, robot still)", "%s, loop %d us, firmware v%d%s",
                    pinpointStatus, pinpoint.getLoopTime(), pinpointFirmware,
                    testPose.resetting() ? ", resetting" : "");
            if (pinpointStatus != GoBildaPinpointDriver.DeviceStatus.READY && !testPose.resetting()) {
                screen.addLine("!! Pinpoint is not READY: the pose may be stale. Note this status before power cycling.");
            }
            shooter.addTelemetry(screen);
            turret.addTelemetry(screen);
            screen.addLine("Sticks: drive. Circle: field/robot-centric. RT: slow. Cross: wheel on/off (RPM from the table). D-pad up/down: RPM trim. RB (hold, wheel on): fire. LT: reverse rail. Square: turret tracking. Triangle: turn prediction. D-pad left/right: goal. Share: reset Pinpoint, keeps the pose. Driver 2: relocalize.");
            screen.update();
        }

        drivetrain.stop();
        intake.stop();
        transfer.stop();
        shooter.setTargetRpm(0);
        shooter.update();
        if (door != null) {
            door.close();
            door.update();
        }
        turret.setTestPower(0.0);
        turret.update();
    }
}
