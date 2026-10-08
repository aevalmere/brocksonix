package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * DriveModel reads Foresight's translational and heading gains from the follower's
 * own controllers (a real ForesightConfig, piecewise and PID ones too), falls back to
 * its static fields when a gain isn't simply proportional, and never touches the
 * controllers while the follower is following a path or holding a pose.
 */
public class DriveModelTest {
    private static final double EPS = 1e-9;
    private double heading, forwardNear, forwardFar, strafeNear, strafeFar;

    @Before
    public void fallbacksThatStandOut() {
        heading = DriveModel.HEADING_GAIN;
        forwardNear = DriveModel.FORWARD_KP_NEAR;
        forwardFar = DriveModel.FORWARD_KP_FAR;
        strafeNear = DriveModel.STRAFE_KP_NEAR;
        strafeFar = DriveModel.STRAFE_KP_FAR;
        DriveModel.HEADING_GAIN = 9.1;
        DriveModel.FORWARD_KP_NEAR = 9.2;
        DriveModel.FORWARD_KP_FAR = 9.3;
        DriveModel.STRAFE_KP_NEAR = 9.4;
        DriveModel.STRAFE_KP_FAR = 9.5;
    }

    @After
    public void putBack() {
        DriveModel.HEADING_GAIN = heading;
        DriveModel.FORWARD_KP_NEAR = forwardNear;
        DriveModel.FORWARD_KP_FAR = forwardFar;
        DriveModel.STRAFE_KP_NEAR = strafeNear;
        DriveModel.STRAFE_KP_FAR = strafeFar;
    }

    /** TestRobot's Foresight setup with these three controllers. */
    private static Follower follower(final Controller forward, final Controller strafe, final Controller heading) {
        ForesightConfig base = TestRobot.config();
        ForesightConfig config = new ForesightConfig(c -> {
            c.forwardTranslational.set(forward);
            c.strafeTranslational.set(strafe);
            c.headingFeedback.set(heading);
            c.coast.set(base.coast.get());
            c.brake.set(base.brake.get());
            c.headingBrakeCoefficients.set(base.headingBrakeCoefficients.get());
            c.linearBrakeCoefficients.set(base.linearBrakeCoefficients.get());
            c.quadraticBrakeCoefficients.set(base.quadraticBrakeCoefficients.get());
            c.maxAchievableForwardVelocity.set(base.maxAchievableForwardVelocity.get());
            c.maxAchievableStrafeVelocity.set(base.maxAchievableStrafeVelocity.get());
            c.naturalForwardDeceleration.set(base.naturalForwardDeceleration.get());
            c.naturalStrafeDeceleration.set(base.naturalStrafeDeceleration.get());
        });
        TestRobot.Wheels wheels = new TestRobot.Wheels();
        return new Follower(new TestRobot.Body(wheels), wheels, new Foresight(config));
    }

    /** Counts every call, so a test can see that nothing was touched. Proportional, gain 0.4. */
    private static final class Counting implements Controller {
        int calls;

        @Override
        public double calculate(double target, double error) {
            calls++;
            return 0.4 * error;
        }

        @Override
        public void reset() {
            calls++;
        }
    }

    @Test
    public void readsPiecewiseAndPidGains() {
        Follower follower = follower(
                Controller.piecewise(Controller.proportional(0.21)).put(2.5, Controller.proportional(0.61)),
                // PIDs with a derivative (and one with an integral): read right after a reset, only kP shows.
                Controller.piecewise(Controller.pid(0.3, 0, 0.01)).put(2.5, Controller.pid(0.8, 0.05, 0.02)),
                Controller.pid(3.1, 0, 0.05));
        DriveModel model = DriveModel.capture(follower);
        assertEquals(0.21, model.forwardGainNear, EPS);
        assertEquals(0.61, model.forwardGainFar, EPS);
        // A PID's integral could add kI x (a millisecond or two) if the computer stalls between reset and probe.
        assertEquals(0.3, model.strafeGainNear, 1e-3);
        assertEquals(0.8, model.strafeGainFar, 1e-3);
        assertEquals(3.1, model.headingGain, 1e-3);
    }

    @Test
    public void fallsBackWhenAGainIsNotSimplyProportional() {
        Follower follower = follower(
                // A static part: output / error changes with the error.
                Controller.sum(Controller.proportional(0.2), Controller.staticFeedforward(0.05)),
                // Zero under 2.5 in., fine past it.
                Controller.piecewise(Controller.proportional(0)).put(2.5, Controller.proportional(0.7)),
                (target, error) -> error * error);
        DriveModel model = DriveModel.capture(follower);
        assertEquals(9.2, model.forwardGainNear, EPS);
        assertEquals(9.3, model.forwardGainFar, EPS);
        assertEquals(9.4, model.strafeGainNear, EPS);
        assertEquals(0.7, model.strafeGainFar, EPS);
        assertEquals(9.1, model.headingGain, EPS);
    }

    @Test
    public void neverProbesWhileHoldingButKeepsWhatItRead() {
        Counting forward = new Counting(), strafe = new Counting(), heading = new Counting();
        Follower follower = follower(forward, strafe, heading);

        // Holding a pose (Foresight uses its controllers, and hold() doesn't reset them): not touched.
        follower.hold(new Pose(30, 30, 0));
        DriveModel holding = DriveModel.capture(follower);
        assertEquals(0, forward.calls + strafe.calls + heading.calls);
        assertEquals(9.2, holding.forwardGainNear, EPS);
        assertEquals(9.1, holding.headingGain, EPS);

        // Driven by hand: Foresight isn't using them, so they are read.
        follower.manual(DrivePowers.zero());
        DriveModel manual = DriveModel.capture(follower);
        assertEquals(0.4, manual.forwardGainNear, EPS);
        assertEquals(0.4, manual.strafeGainFar, EPS);
        assertEquals(0.4, manual.headingGain, EPS);

        // Holding again: what was read is kept, and nothing is called.
        int before = forward.calls + strafe.calls + heading.calls;
        follower.hold(new Pose(30, 30, 0));
        DriveModel later = DriveModel.capture(follower);
        assertEquals(before, forward.calls + strafe.calls + heading.calls);
        assertEquals(0.4, later.forwardGainFar, EPS);
        assertEquals(0.4, later.headingGain, EPS);
    }

    @Test
    public void testRobotGainsMatchWhatTheTunerWouldGive() {
        // TestRobot's Foresight is set up the way ForesightTuner would: the read gains are its numbers.
        // TestRobot's heading controller uses HEADING_GAIN as it is when TestRobot is made: make it 2.5, then change the field.
        DriveModel.HEADING_GAIN = 2.5;
        TestRobot robot = new TestRobot();
        DriveModel.HEADING_GAIN = 7.7;
        DriveModel model = DriveModel.capture(robot.follower);
        assertEquals(TestRobot.TAU_F * 6.2 * 6.2 / TestRobot.FORWARD, model.forwardGainNear, 1e-12);
        assertEquals(TestRobot.TAU_S * 10.2 * 10.2 / TestRobot.STRAFE, model.strafeGainFar, 1e-12);
        assertEquals(2.5, model.headingGain, 1e-12);
    }
}
