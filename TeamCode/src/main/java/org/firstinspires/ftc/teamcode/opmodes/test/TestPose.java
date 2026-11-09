package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driver.Feedback;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotState;

/**
 * The robot's pose in the test OpModes. One pose follows the robot from test to
 * test, and only driver 2's relocalize buttons change it (the same buttons as
 * Main TeleOp). It is saved in RobotState every loop, so it lives until the app
 * restarts, and Main TeleOp picks it up if it is trusted and recent.
 *
 * Pedro's PinpointLocalizer constructor calls reset() in every OpMode. With the
 * resetMode in Constants.localizerConfig (the default, RECALIBRATE_IMU) that
 * recalibrates the IMU, and RESET_AND_RECALIBRATE_IMU would zero the pose as
 * well. So the robot must be still during init, and start() always puts the
 * carried pose back instead of counting on the Pinpoint to still have it.
 *
 * Build it before waitForStart, call start() right after it, and update() once
 * per loop.
 */
final class TestPose {
    private final PinpointLocalizer localizer;
    private final Gamepad driver1, driver2;
    private final ElapsedTime resetTimer = new ElapsedTime();

    private Pose lastGood;
    private boolean trusted = false;
    private String source = "";
    private boolean resetting = false;
    private double resetWaitSec = 0;

    TestPose(HardwareMap hardwareMap, Gamepad driver1, Gamepad driver2) {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        this.driver1 = driver1;
        this.driver2 = driver2;
    }

    /** The alliance last picked in a match OpMode, or red if there is none. Tests never write RobotState.alliance. */
    static Alliance alliance() {
        return RobotState.alliance != null ? RobotState.alliance : Alliance.RED;
    }

    /** One line for the init screen: where the pose will start from. */
    String startDescription() {
        Pose saved = RobotState.lastPose();
        if (saved == null) {
            return "Start pose: none saved, so put the robot in the NEAR_RIGHT corner or relocalize with driver 2.";
        }
        return String.format("Start pose: from the last OpMode, x %.1f  y %.1f  heading %.0f deg (%s).",
                saved.x(), saved.y(), Math.toDegrees(saved.heading()),
                RobotState.lastPoseTrusted() ? "trusted" : "not trusted");
    }

    /** Call right after waitForStart: puts the carried pose back, or a guess if there is none. */
    void start() {
        // "WasPressed" remembers presses made during init, which would relocalize on the first loop.
        driver2.resetEdgeDetection();
        Pose saved = RobotState.lastPose();
        if (saved != null) {
            lastGood = saved;
            trusted = RobotState.lastPoseTrusted();
            source = "last OpMode";
        } else {
            lastGood = Field.cornerPose(alliance(), Corner.NEAR_RIGHT);
            trusted = false;
            source = "guess, NEAR_RIGHT corner";
        }
        localizer.setPose(lastGood);
    }

    /** Call once per loop. The pose this loop: NaN if the Pinpoint reads NaN, never the zeros of a reset. */
    Pose update() {
        relocalizeButtons();

        if (resetting && resetTimer.seconds() >= resetWaitSec) {
            localizer.setPose(lastGood);
            resetting = false;
        }

        localizer.update();
        Pose pose = localizer.pose();
        // Mid-reset it reads zeros, which must not reach the turret or RobotState.
        if (resetting) return lastGood;
        if (!isNaN(pose)) {
            lastGood = pose;
            RobotState.savePose(pose, trusted);
        }
        return pose;
    }

    /** Zeroes the Pinpoint and recalibrates its IMU, robot still. The pose comes back after waitSec. */
    void resetPinpoint(GoBildaPinpointDriver pinpoint, double waitSec) {
        pinpoint.resetPosAndIMU();
        resetTimer.reset();
        resetWaitSec = waitSec;
        resetting = true;
    }

    boolean resetting() {
        return resetting;
    }

    void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Pose from", "%s, %s. Driver 2: D-pad down/up, Cross, Triangle = corners; D-pad left, Circle = heading",
                source, trusted ? "trusted" : "NOT trusted");
    }

    /** Main TeleOp's map: left hand = left side, right hand = right side (see MainTeleOp.driverTwo). */
    private void relocalizeButtons() {
        if (driver2.dpadDownWasPressed()) toCorner(Corner.NEAR_LEFT);
        if (driver2.dpadUpWasPressed()) toCorner(Corner.FAR_LEFT);
        if (driver2.dpadLeftWasPressed()) toWallHeading(true);
        if (driver2.crossWasPressed()) toCorner(Corner.NEAR_RIGHT);
        if (driver2.triangleWasPressed()) toCorner(Corner.FAR_RIGHT);
        if (driver2.circleWasPressed()) toWallHeading(false);
    }

    private void toCorner(Corner corner) {
        relocalized(Field.cornerPose(alliance(), corner), true, "relocalized " + corner);
    }

    /** Heading only: x, y and trust stay, like Robot.relocalizeHeading. Refused while the pose is NaN. */
    private void toWallHeading(boolean leftWall) {
        Pose now = resetting ? lastGood : localizer.pose();
        if (isNaN(now)) return;
        relocalized(new Pose(now.x(), now.y(), Field.sideWallHeading(alliance(), leftWall)), trusted, "heading from a side wall");
    }

    private void relocalized(Pose pose, boolean nowTrusted, String from) {
        localizer.setPose(pose);
        lastGood = pose; // so a relocalize during a Pinpoint reset is the pose it comes back to
        trusted = nowTrusted;
        source = from;
        if (Feedback.RELOCALIZE_MS > 0) {
            driver1.rumble(Feedback.RELOCALIZE_STRENGTH, Feedback.RELOCALIZE_STRENGTH, Feedback.RELOCALIZE_MS);
            driver2.rumble(Feedback.RELOCALIZE_STRENGTH, Feedback.RELOCALIZE_STRENGTH, Feedback.RELOCALIZE_MS);
        }
    }

    private static boolean isNaN(Pose pose) {
        return Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading());
    }
}
