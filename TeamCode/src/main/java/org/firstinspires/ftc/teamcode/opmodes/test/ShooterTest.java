package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.ShootingZone;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.shot.ShotTables;
import org.firstinspires.ftc.teamcode.robot.subsystems.CoaxHood;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.LookupTable;

/**
 * Flywheel tuning and filling the shot tables. The Pinpoint tracks the robot:
 * at Start the pose is set to the NEAR_RIGHT corner (red's GARDEN corner, intake
 * flat on the alliance wall, side flat on the audience wall), so start the robot
 * there. Options sets it there again. Drive to a spot with the sticks (Drive
 * Test's stick code, RT slow), and the screen shows the pose and the distance to
 * the cell; Panels' field page draws the robot and the cell. Find the RPM that
 * scores, put the pair into ShotTables (also live in Panels). Tune KS/KV/KP live
 * in Panels (Shooter). Red field coordinates throughout.
 *
 * No break beams needed. Cross: wheel on/off. The target is TARGET_RPM (Panels);
 * D-pad up/down changes it by a big step, left/right by a small one.
 * Continuous fire, no button: while the wheel is on, the door opens and, once it
 * is fully open, intake and transfer feed together nonstop, at any wheel speed.
 * With the wheel off the door closes, the intake keeps collecting and the
 * transfer holds balls against the door at Transfer.HOLD_POWER.
 *
 * Door positions: with the wheel off, Triangle swings the door open/closed with the
 * rail stopped, so no ball goes through. Edit Door.OPEN_POSITION and
 * Door.CLOSED_POSITION in Panels (page "Door"); the servo follows live.
 *
 * Before the first spin, with the wheel off: hold Square to run only the left
 * motor, Circle to run only the right one, at low power. Each must push the
 * shooting direction; if not, flip its REVERSED flag in Panels (works live).
 * With the left motor running, RPM must read positive, or flip ENCODER_REVERSED.
 *
 * With the coax hood on the robot, it holds CoaxHood.STATIC_LAUNCH_DEG, like Main
 * TeleOp with Robot.HOOD = STATIC, so the SHOT_RPM rows found here fit the static tables.
 */
@Configurable
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {
    /** Power for the one-motor check (Square / Circle). Keep it low. */
    public static double MOTOR_TEST_POWER = 0.2;
    /** D-pad up/down changes the target RPM by this much. */
    public static double RPM_STEP_BIG = 100;
    /** D-pad left/right changes the target RPM by this much. */
    public static double RPM_STEP_SMALL = 25;
    /** Rail power while feeding (right bumper). Compare with ShotTables.SHOT_FEED_POWER. */
    public static double FEED_POWER = 1.0;
    /** Wheel target while it is on (Cross). Type it in Panels, or step it with the D-pad. */
    public static double TARGET_RPM = 3000;
    /** Which cell the distance is measured to: true = FAR cell, false = AUDIENCE cell. */
    public static boolean FAR_CELL = true;

    /** Panels' PEDRO_PATHING picture is 144 across; ours is Field.SIZE (see PlanDrawing). Drawing only. */
    private static final double TO_PANELS = 144 / Field.SIZE;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, voltage);
        Door door = new Door(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        // Moves nothing in init: Mecanum only sets directions and float mode, the Pinpoint only reads.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        // Optional, like in Robot: null when the configuration has no hood servo.
        CoaxHood hood = CoaxHood.isConfigured(hardwareMap) ? new CoaxHood(hardwareMap) : null;
        boolean on = false;
        boolean doorTestOpen = false;

        waitForStart();
        // "WasPressed" remembers presses made during init: Cross would turn the wheel on at start.
        gamepad1.resetEdgeDetection();
        Pose startPose = Field.cornerPose(Alliance.RED, Corner.NEAR_RIGHT);
        localizer.setPose(startPose);
        while (opModeIsActive()) {
            bulkReads.clear();
            shooter.readSensors();
            if (gamepad1.optionsWasPressed()) localizer.setPose(startPose);
            localizer.update();
            Pose pose = localizer.pose();
            drivetrain.drive(DriveTest.stickPowers(gamepad1), true);

            if (gamepad1.crossWasPressed()) on = !on;
            // Door test only with the wheel off; turning the wheel on ends it.
            if (gamepad1.triangleWasPressed() && !on) doorTestOpen = !doorTestOpen;
            if (on) doorTestOpen = false;
            if (gamepad1.dpadUpWasPressed()) TARGET_RPM += RPM_STEP_BIG;
            if (gamepad1.dpadDownWasPressed()) TARGET_RPM -= RPM_STEP_BIG;
            if (gamepad1.dpadRightWasPressed()) TARGET_RPM += RPM_STEP_SMALL;
            if (gamepad1.dpadLeftWasPressed()) TARGET_RPM -= RPM_STEP_SMALL;
            if (TARGET_RPM < 0) TARGET_RPM = 0;

            // Continuous fire: feeding whenever the wheel is on.
            boolean fire = on;
            // One motor alone, open loop. Only with the wheel off, so it can't fight the closed loop.
            boolean testLeft = !on && gamepad1.square;
            boolean testRight = !on && gamepad1.circle;
            if (testLeft || testRight) {
                shooter.setTestPowers(testLeft ? MOTOR_TEST_POWER : 0, testRight ? MOTOR_TEST_POWER : 0);
            } else {
                shooter.setTargetRpm(on ? TARGET_RPM : 0);
            }
            shooter.setBoost(fire);
            if (fire || doorTestOpen) door.open();
            else door.close();

            if (doorTestOpen) {
                // Door test: the rail stays still, so balls don't roll into the wheel.
                intake.stop();
                transfer.stop();
            } else if (fire && door.isFullyOpen()) {
                intake.feed(FEED_POWER);
                transfer.feed(FEED_POWER);
            } else if (fire) {
                intake.stop();
                transfer.stop();
            } else {
                // No beams: the intake always collects, and the transfer holds gently against the closed door.
                intake.collect(false);
                transfer.collect(true);
            }

            shooter.update();
            door.update();
            if (hood != null) {
                // Every loop, so a Panels edit to STATIC_LAUNCH_DEG moves it live.
                hood.setTargetDeg(CoaxHood.STATIC_LAUNCH_DEG);
                hood.update();
            }

            String tableProblem = ShotTables.orderProblem();
            if (tableProblem != null) screen.addLine("!! " + tableProblem + " (fix in Panels, ShotTables)");
            screen.addData("Wheel (Cross)", on ? "on, firing nonstop" : "off");
            screen.addData("One motor (hold, wheel off)", "Square = left, Circle = right, %.2f power", MOTOR_TEST_POWER);
            screen.addData("Target RPM (Panels TARGET_RPM, D-pad)", "%.0f", TARGET_RPM);
            screen.addData("Door test (Triangle, wheel off)", doorTestOpen ? "door OPEN, rail stopped" : "off");
            screen.addData("Door positions (Panels, Door)", "open %.3f  closed %.3f",
                    Door.OPEN_POSITION, Door.CLOSED_POSITION);
            door.addTelemetry(screen);
            screen.addData("At speed", shooter.atSpeed());
            Pose cell = Field.cellTarget(Alliance.RED, FAR_CELL ? Cell.FAR : Cell.AUDIENCE);
            boolean poseOk = !Double.isNaN(pose.x()) && !Double.isNaN(pose.y()) && !Double.isNaN(pose.heading());
            if (poseOk) {
                double distance = Math.hypot(cell.x() - pose.x(), cell.y() - pose.y());
                screen.addData("Pose (Options: back to the GARDEN corner)", "x %.1f  y %.1f  heading %.0f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                screen.addData("Distance to " + (FAR_CELL ? "FAR" : "AUDIENCE") + " cell",
                        "%.1f in. SHOT_RPM there now: %.0f",
                        distance, LookupTable.at(ShotTables.SHOT_RPM, distance));
            } else {
                screen.addLine("!! Pose is NaN: the Pinpoint is not reading. Check its cable and pods.");
            }
            drawField(poseOk ? pose : null, cell);
            shooter.addTelemetry(screen);
            intake.addTelemetry(screen);
            transfer.addTelemetry(screen);
            if (hood != null) {
                screen.addLine("Coax hood: holding STATIC_LAUNCH_DEG. Wait for \"at target\" before firing.");
                hood.addTelemetry(screen);
            }
            screen.addData("Battery", "%.2f V", voltage.get());
            screen.addData("Slow mode (RT)", DriveTest.slowHeld(gamepad1) ? "on" : "off");
            screen.update();
        }
        drivetrain.stop();
    }

    /**
     * Panels only: the robot's outline (null pose: not drawn), the cell, and a line between
     * them. Panels sends at most every 100 ms, so this does nothing between sends.
     */
    private static void drawField(Pose pose, Pose cell) {
        drawField(pose, cell, null);
    }

    /**
     * The same, plus (when turretFieldRad isn't null) an orange line where the turret really
     * points, as a field angle: robot heading + turret angle. Continuous Test uses it.
     */
    static void drawField(Pose pose, Pose cell, Double turretFieldRad) {
        FieldManager field = PanelsField.INSTANCE.getField();
        if (!field.getShouldUpdateCanvas()) return;
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        field.setStyle("red", "red", 0.8 * TO_PANELS);
        field.moveCursor(cell.x() * TO_PANELS, cell.y() * TO_PANELS);
        field.circle(2 * TO_PANELS);
        if (pose != null) {
            double[][] outline = ShootingZone.robotOutline(pose);
            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "limegreen", 0.8 * TO_PANELS);
            for (int i = 0; i < 4; i++) {
                int next = (i + 1) % 4;
                field.moveCursor(outline[0][i] * TO_PANELS, outline[1][i] * TO_PANELS);
                field.line(outline[0][next] * TO_PANELS, outline[1][next] * TO_PANELS);
            }
            // The intake end: a short line from the center toward the front.
            field.moveCursor(pose.x() * TO_PANELS, pose.y() * TO_PANELS);
            field.line((pose.x() + 6 * Math.cos(pose.heading())) * TO_PANELS, (pose.y() + 6 * Math.sin(pose.heading())) * TO_PANELS);
            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "yellow", 0.5 * TO_PANELS);
            field.moveCursor(pose.x() * TO_PANELS, pose.y() * TO_PANELS);
            field.line(cell.x() * TO_PANELS, cell.y() * TO_PANELS);
            if (turretFieldRad != null) {
                field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "orange", 0.8 * TO_PANELS);
                field.moveCursor(pose.x() * TO_PANELS, pose.y() * TO_PANELS);
                field.line((pose.x() + 18 * Math.cos(turretFieldRad)) * TO_PANELS, (pose.y() + 18 * Math.sin(turretFieldRad)) * TO_PANELS);
            }
        }
        field.update();
    }
}
