package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driver.AllianceSelector;
import org.firstinspires.ftc.teamcode.driver.Feedback;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.ShootingZone;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.robot.Warning;
import org.firstinspires.ftc.teamcode.util.DriveCurve;

/**
 * Driver 1 (gamepad1) drives, fires, intakes and turns zone fire on and off.
 * RB fires anywhere: the robot shoots or passes by itself (Robot's class comment).
 * With zone fire on (Triangle), being in the shooting zone fires too, once the
 * shot is ready. Parked in the zone the rail keeps feeding, so balls picked up
 * there go straight through.
 *
 * Driver 2 (gamepad2) handles what is easy to watch from the side: which of our
 * HIVE's cells is up (LB = the cell on the driver's left, RB = right),
 * relocalizing, and aim trims on the sticks. For relocalizing, the left hand
 * covers the left corners and the right hand the right ones, and the button
 * points the way the intake faces: toward the driver, away, or at the side wall.
 *
 * The full button map is in TELEOP_PLAN.md.
 */
@Configurable
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    // TODO(10): Set to false before competition.
    /**
     * true: every value, every loop, so Panels shows live values while tuning, and the
     * shooting zone is drawn on the Panels field page.
     * false: only the warnings, the goal and zone fire lines and loop time, every
     * TELEMETRY_INTERVAL_MS. Autos use this too.
     */
    public static boolean FULL_TELEMETRY = true;
    // TODO(12): Watch Loop ms with everything running, goal under 15. Check again with FULL_TELEMETRY false.
    public static double TELEMETRY_INTERVAL_MS = 200;
    // TODO(12): Driver 2 practice: a typical miss should take about a second of full stick to fix. Raise it if fixing takes too long, lower it if the trim is hard to stop in the right place.
    /** Driver 2's left stick, full left or right: how fast the turret trim moves, degrees per second. */
    public static double TURRET_TRIM_RATE_DEG_PER_SEC = 4;
    // TODO(12): Same as the turret trim, for shots that land short or long (static hood).
    /** Driver 2's right stick, full up or down: how fast the RPM trim moves, RPM per second. */
    public static double RPM_TRIM_RATE_PER_SEC = 75;
    // TODO(16): Main TeleOp, COAX: same as the turret trim, for shots that land high or low.
    /** Coax hood: the right stick trims the hood angle instead of the RPM, since the hood re-solves for any RPM. Degrees per second. */
    public static double HOOD_TRIM_RATE_DEG_PER_SEC = 1;
    // TODO(12): Raise it if a resting stick still moves a trim; lower it if a small push does nothing.
    /** Driver 2's trim sticks count as 0 under this, so a resting stick never moves a trim. */
    public static double TRIM_STICK_DEADBAND = 0.15;
    // TODO(12): Raise it if a resting finger fires LT; lower it if a full pull doesn't register.
    /** How far a trigger (LT, unjam) must be pulled to count as pressed. Test OpModes use it too. */
    public static double TRIGGER_THRESHOLD = 0.5;

    /** A slow loop counts as at most this long for the trims, so one stalled loop can't jump a trim. */
    private static final double MAX_TRIM_LOOP_SEC = 0.1;
    /** Panels' PEDRO_PATHING picture is 144 across; ours is Field.SIZE (see PlanDrawing). Drawing only. */
    private static final double TO_PANELS = 144 / Field.SIZE;

    private Robot robot;
    private Feedback feedback;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        feedback = new Feedback(gamepad1, gamepad2);

        AllianceSelector allianceSelector = new AllianceSelector(gamepad1, gamepad2);
        Pose savedPose = RobotState.recentPose();
        Alliance savedAlliance = RobotState.alliance;

        while (opModeInInit()) {
            allianceSelector.update(screen);
            // The saved pose is in the old alliance's frame, so switching drops it.
            if (RobotState.alliance != savedAlliance) savedPose = null;
            // Without a saved pose, field-centric driving uses a guessed heading: intake facing the driver.
            screen.addData("Start pose", savedPose == null
                    ? "unknown: set the robot down with the intake facing you, and relocalize in a corner before shooting"
                    : "from the last OpMode " + savedPose);
            screen.update();
        }
        // "WasPressed" remembers presses made during init, which would fire on the first loop
        // (a relocalize, an intake toggle, zone fire). Start with none.
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();

        if (savedPose != null) {
            robot.setPose(savedPose, true);
        } else if (RobotState.alliance != null) {
            robot.setPose(Field.cornerPose(RobotState.alliance, Corner.NEAR_RIGHT), false);
        }

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime sinceTelemetry = new ElapsedTime();
        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            // Same order as auto: buttons, then commands, then update applies both in this loop.
            driverOne(gamepad1);
            driverTwo(gamepad2, Math.min(loopMs / 1000, MAX_TRIM_LOOP_SEC));
            Scheduler.execute();
            robot.update();
            feedback.update(robot.storage, robot.warnings());

            if (FULL_TELEMETRY || sinceTelemetry.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                sinceTelemetry.reset();
                robot.addWarnings(screen);
                // Driver Station lines the drivers need in a match, so they show with FULL_TELEMETRY off too.
                screen.addData("Goal", robot.goalStatus());
                screen.addData("Zone fire", robot.zoneFireStatus());
                screen.addData("Loop", "%.1f ms", loopMs);
                if (FULL_TELEMETRY) {
                    robot.addTelemetry(screen);
                    drawShootingZone();
                }
                screen.update();
            }
        }
        feedback.stop();
        robot.stop();
    }

    private void driverOne(Gamepad pad) {
        // Before an alliance is picked, drive as red: driving still works, and nothing fires without an alliance.
        robot.drive.drive(
                -pad.left_stick_y,
                -pad.left_stick_x,
                -pad.right_stick_x,
                RobotState.alliance != null ? RobotState.alliance : Alliance.RED);

        // These are set from the buttons every loop. A macro that sets one of them too
        // has to be OR-ed in here, or the button overwrites it on the next loop.
        // RB fires anywhere, zone fire or not. Robot picks shoot or pass.
        robot.shootHeld = pad.right_bumper;
        robot.unjamHeld = pad.left_trigger > TRIGGER_THRESHOLD;

        if (pad.leftBumperWasPressed()) robot.flowerIntake.toggle();
        if (pad.triangleWasPressed()) robot.zoneFire = !robot.zoneFire;
    }

    private void driverTwo(Gamepad pad, double loopSec) {
        // The cell that is up, as driver 2 sees it from the ALLIANCE AREA. Needs an alliance.
        if (pad.leftBumperWasPressed() && RobotState.alliance != null) {
            robot.upCell = Field.cellFromDriver(RobotState.alliance, true);
        }
        if (pad.rightBumperWasPressed() && RobotState.alliance != null) {
            robot.upCell = Field.cellFromDriver(RobotState.alliance, false);
        }

        // Relocalize, with the robot pushed into the corner (or its intake flat on the side wall).
        // Left hand, left side: D-pad down = near corner, up = far corner, left = heading only.
        // Right hand, right side: Cross = near corner, Triangle = far corner, Circle = heading only.
        // Both drivers feel one buzz when it happened.
        if (pad.dpadDownWasPressed() && robot.relocalize(Corner.NEAR_LEFT)) feedback.relocalized();
        if (pad.dpadUpWasPressed() && robot.relocalize(Corner.FAR_LEFT)) feedback.relocalized();
        if (pad.dpadLeftWasPressed() && robot.relocalizeHeading(true)) feedback.relocalized();
        if (pad.crossWasPressed() && robot.relocalize(Corner.NEAR_RIGHT)) feedback.relocalized();
        if (pad.triangleWasPressed() && robot.relocalize(Corner.FAR_RIGHT)) feedback.relocalized();
        if (pad.circleWasPressed() && robot.relocalizeHeading(false)) feedback.relocalized();

        // Trims: holding a stick moves the trim slowly, at a rate per second, so it is the
        // same at any loop speed. The SDK reads stick left and stick up as -1, hence the minus.
        // Left stick left: turret counter-clockwise (positive).
        double turretStick = DriveCurve.shape(-pad.left_stick_x, TRIM_STICK_DEADBAND, 0);
        robot.turret.addTrim(turretStick * TURRET_TRIM_RATE_DEG_PER_SEC * loopSec);
        // Right stick up: more RPM. With the coax hood an RPM trim does nothing useful (the hood
        // just re-solves for the new speed), so it trims the hood there. Steeper mostly means higher at the cell.
        double upStick = DriveCurve.shape(-pad.right_stick_y, TRIM_STICK_DEADBAND, 0);
        if (robot.usingCoaxHood()) {
            robot.coaxHood.addTrim(upStick * HOOD_TRIM_RATE_DEG_PER_SEC * loopSec);
        } else {
            robot.shooter.addTrim(upStick * RPM_TRIM_RATE_PER_SEC * loopSec);
        }
    }

    /**
     * Practice aid, Panels only: the shooting zone that counts now, and the robot's outline,
     * green while it is in the zone and red outside. Panels sends at most every 100 ms, so
     * this does nothing between sends. Nothing else draws on the field page in this OpMode.
     */
    private void drawShootingZone() {
        if (RobotState.alliance == null) return;
        FieldManager field = PanelsField.INSTANCE.getField();
        if (!field.getShouldUpdateCanvas()) return;
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        drawShape(field, ShootingZone.corners(RobotState.alliance, robot.shootingZoneCell()), "yellow");
        if (!robot.warnings().contains(Warning.POSE_NAN)) {
            drawShape(field, ShootingZone.robotOutline(robot.drive.pose()), robot.inShootingZone() ? "limegreen" : "red");
        }
        field.update();
    }

    /** Draws a closed shape (x of each corner in [0], y in [1], field inches) in Panels inches. */
    private static void drawShape(FieldManager field, double[][] shape, String color) {
        double[] xs = shape[0];
        double[] ys = shape[1];
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), color, 0.8 * TO_PANELS);
        for (int i = 0; i < xs.length; i++) {
            int next = (i + 1) % xs.length;
            field.moveCursor(xs[i] * TO_PANELS, ys[i] * TO_PANELS);
            field.line(xs[next] * TO_PANELS, ys[next] * TO_PANELS);
        }
    }
}
