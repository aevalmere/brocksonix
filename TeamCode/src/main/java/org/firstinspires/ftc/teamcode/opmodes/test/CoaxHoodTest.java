package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.shot.HoodSolver;
import org.firstinspires.ftc.teamcode.robot.shot.ShotTables;
import org.firstinspires.ftc.teamcode.robot.subsystems.CoaxHood;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.LookupTable;

import java.util.ArrayList;
import java.util.List;

/**
 * Coax hood setup, then a check of the hood physics with real shots. Change
 * CoaxHood in Panels (works live). The hood moves to IDLE_LAUNCH_DEG at start.
 *
 * Cross: switch between angle mode and physics mode.
 *
 * Angle mode, for finding the CoaxHood numbers. D-pad up/down: target ±1°,
 * bumpers: ±5°. Triangle: jump between MIN and MAX (time it for SPEED_DEG_PER_SEC).
 * Measure the real launch angle with a phone level on the hood's exit lip and
 * compare it with the commanded angle. The wheel stays off while you measure.
 * Hold Square to spin it at ANGLE_MODE_RPM.
 * Circle: raw servo on/off. Raw servo ignores the angle: D-pad up/down moves the
 * servo position ±0.005, bumpers ±0.05. Use it to find SERVO_AT_ANCHOR and
 * ANCHOR_DEG (the order is in CoaxHood). Back in angles, the target starts at the
 * angle the raw position maps to, so once those two are set for it the hood stays
 * put (it only moves if that angle is past MIN_LAUNCH_DEG or MAX_LAUNCH_DEG).
 *
 * Physics mode. Put the robot at a taped distance from the cell (turret center to
 * the center of the cell's opening, along the floor) and set the same distance
 * with D-pad left/right (±5 in.). The wheel runs at COAX_RPM for that distance
 * plus an offset, D-pad up/down (±100 RPM). The hood follows HoodSolver's angle
 * for the RPM the wheel really has. Right bumper (hold): once the hood has an angle
 * for this RPM and is there, the door opens and the rail feeds. While RB stays held
 * the door stays open, and the rail pauses whenever the hood isn't ready, so every
 * ball leaves at the angle the physics picked. Unlike a Main TeleOp burst, nothing
 * checks the wheel speed, there is no READY_HOLD_MS and no turret, and letting go
 * closes the door at once.
 * Firing from one spot at two or three offsets checks ShotTables.COAX_EXIT_SPEED:
 * the distance stays the real one while the RPM changes.
 *
 * The seed table is GNCE Ditto's LutTuningSolver idea, with our own rule for
 * picking the RPM (HoodSolver.seedRpm):
 * https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/LutTuningSolver.java
 *
 * In init the screen shows the physics seeds for COAX_RPM, every 10 in. from 20 to
 * 110, written so the rows can be copied into ShotTables. Init again to recompute.
 */
@Configurable
@TeleOp(name = "Coax Hood Test", group = "Test")
public class CoaxHoodTest extends LinearOpMode {
    /** Highest RPM the seeds in init may use. */
    public static double SEED_MAX_RPM = 5500;
    /** Angle mode: D-pad up/down changes the hood target by this much. */
    public static double ANGLE_STEP_SMALL = 1;
    /** Angle mode: bumpers change the hood target by this much. */
    public static double ANGLE_STEP_BIG = 5;
    /** Raw servo: D-pad up/down changes the servo position by this much. */
    public static double RAW_STEP_SMALL = 0.005;
    /** Raw servo: bumpers change the servo position by this much. */
    public static double RAW_STEP_BIG = 0.05;
    /** Angle mode: wheel speed while Square is held. */
    public static double ANGLE_MODE_RPM = 3000;
    /** Physics mode: D-pad left/right changes the test distance by this much (inches). */
    public static double DISTANCE_STEP = 5;
    /** Physics mode: D-pad up/down changes the RPM offset by this much. */
    public static double RPM_STEP = 100;
    /** Rail power while feeding (right bumper). Compare with ShotTables.SHOT_FEED_POWER. */
    public static double FEED_POWER = 1.0;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, voltage);
        Door door = new Door(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Storage storage = new Storage(hardwareMap);
        CoaxHood hood = new CoaxHood(hardwareMap);

        boolean physics = false;
        boolean raw = false;
        double rawPosition = CoaxHood.SERVO_AT_ANCHOR;
        double angleTargetDeg = CoaxHood.IDLE_LAUNCH_DEG;
        double distance = 50;
        double rpmOffset = 0;
        boolean doorWanted = false;

        screen.addLine(String.format("Physics seeds for ShotTables.COAX_RPM (hood %.0f to %.0f deg, at most %.0f RPM):",
                CoaxHood.MIN_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG, SEED_MAX_RPM));
        for (String row : seedRows()) screen.addLine(row);
        screen.addLine("Seeds only: check every distance with real shots in physics mode.");
        screen.update();

        waitForStart();
        // "WasPressed" remembers presses made during init, which would fire on the first loop.
        gamepad1.resetEdgeDetection();
        while (opModeIsActive()) {
            bulkReads.clear();
            shooter.readSensors();
            storage.update();

            if (gamepad1.crossWasPressed()) {
                physics = !physics;
                raw = false;
            }
            // "WasPressed" means since the last time it was asked. Ask for every button every
            // loop, in both modes, so a press in one mode doesn't show up later in the other.
            boolean up = gamepad1.dpadUpWasPressed();
            boolean down = gamepad1.dpadDownWasPressed();
            boolean left = gamepad1.dpadLeftWasPressed();
            boolean right = gamepad1.dpadRightWasPressed();
            boolean rightBumper = gamepad1.rightBumperWasPressed();
            boolean leftBumper = gamepad1.leftBumperWasPressed();
            boolean triangle = gamepad1.triangleWasPressed();
            boolean circle = gamepad1.circleWasPressed();

            double tableRpm = 0;
            double solvedDeg = Double.NaN;
            if (physics) {
                if (right) distance += DISTANCE_STEP;
                if (left) distance = Math.max(0, distance - DISTANCE_STEP);
                if (up) rpmOffset += RPM_STEP;
                if (down) rpmOffset -= RPM_STEP;

                tableRpm = LookupTable.at(ShotTables.COAX_RPM, distance);
                shooter.setTargetRpm(tableRpm + rpmOffset);
                // From the RPM the wheel really has. NaN (no shot, or still spinning up) is
                // ignored by setTargetDeg, so the hood keeps its last target.
                solvedDeg = HoodSolver.launchDeg(distance, shooter.averageRpm(), CoaxHood.MIN_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG);
                hood.setTargetDeg(solvedDeg);
            } else {
                if (circle) {
                    raw = !raw;
                    if (raw) {
                        // Start from where the servo is, so the hood doesn't jump.
                        if (!Double.isNaN(hood.servoPosition())) rawPosition = hood.servoPosition();
                    } else {
                        // Back to angles from the angle the raw position maps to, so the hood
                        // doesn't jump either (once SERVO_AT_ANCHOR and ANCHOR_DEG are set).
                        double fromRaw = CoaxHood.launchDegAt(rawPosition);
                        if (Double.isFinite(fromRaw)) angleTargetDeg = fromRaw;
                    }
                }
                if (raw) {
                    if (up) rawPosition += RAW_STEP_SMALL;
                    if (down) rawPosition -= RAW_STEP_SMALL;
                    if (rightBumper) rawPosition += RAW_STEP_BIG;
                    if (leftBumper) rawPosition -= RAW_STEP_BIG;
                    rawPosition = Math.max(0, Math.min(1, rawPosition));
                } else {
                    if (up) angleTargetDeg += ANGLE_STEP_SMALL;
                    if (down) angleTargetDeg -= ANGLE_STEP_SMALL;
                    if (rightBumper) angleTargetDeg += ANGLE_STEP_BIG;
                    if (leftBumper) angleTargetDeg -= ANGLE_STEP_BIG;
                    if (triangle) {
                        // To whichever end is farther, so the move covers the whole range.
                        double middle = (CoaxHood.MIN_LAUNCH_DEG + CoaxHood.MAX_LAUNCH_DEG) / 2;
                        angleTargetDeg = hood.commandedDeg() < middle ? CoaxHood.MAX_LAUNCH_DEG : CoaxHood.MIN_LAUNCH_DEG;
                    }
                    // The target may go past MIN or MAX (the hood stops there), so raising MAX in Panels moves it live.
                    angleTargetDeg = Math.max(0, Math.min(90, angleTargetDeg));
                    hood.setTargetDeg(angleTargetDeg);
                }
                shooter.setTargetRpm(gamepad1.square ? ANGLE_MODE_RPM : 0);
            }
            hood.setTestPosition(raw ? (Double) rawPosition : null);
            // Written here, before the fire check, so atTarget() counts this loop's move (like Robot).
            hood.update();

            // Physics mode: a ball only goes once the hood has an angle for this RPM and is there.
            // Fired at the old angle instead, it would tune COAX_EXIT_SPEED from a shot the physics
            // never aimed. The door opens the first time that is true and stays open while RB is held.
            boolean hoodReady = !Double.isNaN(solvedDeg) && hood.atTarget();
            if (!physics || !gamepad1.right_bumper) doorWanted = false;
            else if (hoodReady) doorWanted = true;

            shooter.setBoost(doorWanted);
            if (doorWanted) door.open();
            else door.close();

            if (doorWanted && hoodReady && door.isFullyOpen()) {
                intake.feed(FEED_POWER);
                transfer.feed(FEED_POWER);
            } else if (doorWanted) {
                intake.stop();
                transfer.stop();
            } else {
                intake.collect(storage.isFull());
                transfer.collect(storage.slot1Occupied());
            }

            shooter.update();
            door.update();

            String tableProblem = ShotTables.orderProblem();
            if (tableProblem != null) screen.addLine("!! " + tableProblem + " (fix in Panels, ShotTables)");
            screen.addData("Mode (Cross)", physics ? "physics" : "angle");
            if (physics) {
                screen.addData("Distance (D-pad left/right)", "%.0f in.", distance);
                screen.addData("Target RPM (table + offset)", "%.0f + %.0f = %.0f (D-pad up/down)", tableRpm, rpmOffset, tableRpm + rpmOffset);
                screen.addData("Measured RPM", "%.0f (average %.0f, what the hood solves with)", shooter.rpm(), shooter.averageRpm());
                if (Double.isNaN(solvedDeg)) {
                    screen.addData("Solved angle", "no shot at this RPM (hood stays at %.1f deg)", hood.commandedDeg());
                } else {
                    double flightTime = HoodSolver.flightTime(distance, shooter.averageRpm(), solvedDeg);
                    screen.addData("Solved angle", "%.1f deg", solvedDeg);
                    screen.addData("Flight time", Double.isNaN(flightTime) ? "-" : String.format("%.2f s", flightTime));
                }
                screen.addData("Hood at target", hood.atTarget());
                if (gamepad1.right_bumper && !hoodReady) screen.addLine("RB: waiting for the hood (no angle at this RPM, or still moving)");
                screen.addData("At speed", shooter.atSpeed());
            } else if (raw) {
                screen.addData("Raw servo (Circle)", "%.3f: copy this into SERVO_AT_ANCHOR, and the launch angle you measure here into ANCHOR_DEG", rawPosition);
                screen.addData("Controls", "D-pad up/down ±%.3f, bumpers ±%.2f, hold Square: wheel at %.0f RPM",
                        RAW_STEP_SMALL, RAW_STEP_BIG, ANGLE_MODE_RPM);
            } else {
                screen.addData("Controls", "D-pad up/down ±%.0f, bumpers ±%.0f, Triangle MIN/MAX, Circle raw servo, hold Square: wheel at %.0f RPM",
                        ANGLE_STEP_SMALL, ANGLE_STEP_BIG, ANGLE_MODE_RPM);
                screen.addLine("Measure the hood's launch angle and compare it with the commanded angle (the second number).");
            }
            hood.addTelemetry(screen);
            shooter.addTelemetry(screen);
            storage.addTelemetry(screen);
            screen.addData("Battery", "%.2f V", voltage.get());
            screen.update();
        }
    }

    /** One line per distance: "{distance, rpm},  // launch angle", ready to copy into COAX_RPM. */
    private static List<String> seedRows() {
        List<String> rows = new ArrayList<>();
        for (int distance = 20; distance <= 110; distance += 10) {
            double rpm = HoodSolver.seedRpm(distance, CoaxHood.MIN_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG, SEED_MAX_RPM);
            if (Double.isNaN(rpm)) {
                rows.add(String.format("%d in.: no shot under %.0f RPM", distance, SEED_MAX_RPM));
            } else {
                // From the seed RPM directly (seedLaunchDeg would run the whole seed scan again).
                double launchDeg = HoodSolver.launchDeg(distance, rpm, CoaxHood.MIN_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG);
                rows.add(String.format("{%d, %.0f},  // %.1f deg", distance, rpm, launchDeg));
            }
        }
        return rows;
    }
}
