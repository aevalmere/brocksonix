package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.driver.AllianceSelector;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.horizon36596.zenith.ivy.AutoFromFile;

/**
 * What every Zenith auto OpMode extends (zenith.json, deploy.opModeBase). zenith deploy
 * writes one small subclass per auto file that names the file and TeamZenithRobot.
 * Zenith's AutoFromFile reads the file and runs it; this class adds what our autos need:
 *
 * - The alliance is picked in init with AllianceSelector, like in every match OpMode.
 *   Zenith flips the routine for blue when it builds it, at init, so picking the
 *   other alliance builds it again. The Robot is built once and kept.
 * - alliance() moves every pose in the file onto our Pedro field (ZenithFrame) and
 *   then flips it for blue, once, with TeamZenithRobot.mirror. Nothing flips it again.
 * - With no alliance picked at start, the robot sits still, as in AutoTemplate.
 *
 * The loop is Zenith's: Scheduler.execute(), then TeamZenithRobot.afterLoop runs
 * robot.update(). See TeamZenithRobot for the rest.
 */
public abstract class ZenithAuto extends AutoFromFile {
    private Robot sharedRobot;

    /** The one Robot this OpMode builds. It is kept when the routine is built again for the other alliance. */
    Robot sharedRobot() {
        if (sharedRobot == null) sharedRobot = buildRobot();
        return sharedRobot;
    }

    /** Builds that Robot. The headless sim overrides this to give Robot a follower, since Constants.create() has none yet. */
    protected Robot buildRobot() {
        return new Robot(hardwareMap);
    }

    /**
     * Zenith calls this for every pose in the file (the start, every waypoint and control
     * point, the point a facePoint heading looks at). It moves the pose onto our field, then
     * flips it if the file was written for the other alliance.
     */
    @Override
    public Pose alliance(Pose filePose) {
        Pose onOurField = ZenithFrame.toPedro(filePose);
        return spec.mirrorsInto(robot.runningAlliance()) ? robot.mirror(onOurField) : onOurField;
    }

    /** AutoFromFile's own runOpMode, plus picking the alliance and building the routine again for it. */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            AllianceSelector allianceSelector = new AllianceSelector(gamepad1, gamepad2);
            initialize();
            Alliance builtFor = RobotState.alliance;

            while (opModeInInit()) {
                allianceSelector.update(telemetry);
                if (RobotState.alliance != builtFor) {
                    // A fast rebuild: the file is read again, the hardware isn't.
                    initialize();
                    builtFor = RobotState.alliance;
                }
                if (RobotState.alliance == null) {
                    telemetry.addLine("!! PICK AN ALLIANCE (D-pad up). Auto does nothing without one.");
                }
                // Adds the file's name and any Zenith warnings, then sends the telemetry.
                initialize_loop();
            }
            if (!opModeIsActive()) return;

            // Shooting at the wrong goal is worse than sitting still.
            if (RobotState.alliance == null) {
                while (opModeIsActive()) {
                    telemetry.addLine("!! NO ALLIANCE PICKED: auto is doing nothing. Stop, restart the OpMode and pick one in init.");
                    telemetry.update();
                    sleep((long) MainTeleOp.TELEMETRY_INTERVAL_MS);
                }
                return;
            }

            startRoutine();
            while (opModeIsActive()) {
                run();
            }
        } finally {
            // TeamZenithRobot.onStop (robot.stop()), then Scheduler.reset(). Runs however the OpMode ends.
            reset();
        }
    }
}
