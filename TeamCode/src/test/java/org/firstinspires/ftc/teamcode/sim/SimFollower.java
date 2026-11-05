package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.agateflow.TestRobot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The follower the sim drives with, built the way Constants.create() will build it:
 * Pedro's real PinpointLocalizer and Mecanum from Constants' configs, on RobotSim's
 * fakes. Constants.create() still returns null (no ForesightTuner output yet), so
 * Foresight gets TestRobot's settings, the ones the AgateFlow tests drive with.
 */
final class SimFollower {
    private SimFollower() {}

    static Follower create(HardwareMap hardwareMap) {
        // TODO(2): Once Constants has foresightConfig(), use it here too, so the sim follows the way the robot does.
        return new Follower(
                new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                new Mecanum(hardwareMap, Constants.drivetrainConfig),
                new Foresight(TestRobot.config()));
    }
}
