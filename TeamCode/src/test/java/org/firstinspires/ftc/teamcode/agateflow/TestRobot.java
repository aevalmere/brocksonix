package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldZones;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A simulated mecanum robot for the tests: Pedro's real Follower and Foresight
 * driving a drivetrain with the same physics DriveModel assumes (top speeds, time
 * constants, battery sag, grip, momentum), on a virtual clock (10 ms steps, as fast
 * as the computer goes, so the tests are repeatable). Foresight is set up the way
 * ForesightTuner would set it up on this drivetrain.
 */
final class TestRobot {
    static final double VOLTS = 12.5, FORWARD = 70, STRAFE = 55, STEP = 0.01;
    static final double TURN = DriveModel.TURN_SPEED_PER_VOLT * VOLTS, TAU_F = DriveModel.FORWARD_TIME_CONSTANT,
            TAU_S = DriveModel.STRAFE_TIME_CONSTANT, TAU_T = DriveModel.TURN_TIME_CONSTANT;
    static final double SAG = DriveModel.BATTERY_OHMS * DriveModel.DRIVE_MOTORS / DriveModel.MOTOR_OHMS;

    final Wheels wheels = new Wheels();
    final Body body = new Body(wheels);
    final Follower follower;

    TestRobot() {
        // AgateFlow works the translational kP out the way ForesightTuner does (0 = not copied in).
        DriveModel.FORWARD_KP_NEAR = 0;
        DriveModel.FORWARD_KP_FAR = 0;
        DriveModel.STRAFE_KP_NEAR = 0;
        DriveModel.STRAFE_KP_FAR = 0;
        follower = new Follower(body, wheels, new Foresight(config()));
    }

    static ForesightConfig config() {
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.piecewise(Controller.proportional(TAU_F * 6.2 * 6.2 / FORWARD))
                    .put(2.5, Controller.proportional(TAU_F * 10.2 * 10.2 / FORWARD)));
            c.strafeTranslational.set(Controller.piecewise(Controller.proportional(TAU_S * 6.2 * 6.2 / STRAFE))
                    .put(2.5, Controller.proportional(TAU_S * 10.2 * 10.2 / STRAFE)));
            c.coast.set(Controller.proportionalFeedforward(1.0 / FORWARD));
            c.brake.set(Controller.proportionalFeedforward(0.85 / FORWARD));
            c.headingFeedback.set(Controller.proportional(DriveModel.HEADING_GAIN));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(TAU_T, 1e-6));
            c.linearBrakeCoefficients.set(Matrix.diag(TAU_F, TAU_S));
            c.quadraticBrakeCoefficients.set(Matrix.diag(1e-6, 1e-6));
            c.maxAchievableForwardVelocity.set(FORWARD);
            c.maxAchievableStrafeVelocity.set(STRAFE);
            c.naturalForwardDeceleration.set(FORWARD / TAU_F);
            c.naturalStrafeDeceleration.set(STRAFE / TAU_S);
        });
    }

    /** An AgateFlow on the real field (FieldZones) with this robot's follower. */
    AgateFlow agateFlow() {
        FieldMap map = new FieldMap();
        map.setZones("field", FieldZones.zones());
        return new AgateFlow(map, follower);
    }

    Plan plan(AgateFlow agateFlow, Pose start, Route route, Alliance alliance) {
        return agateFlow.plan(start, new Velocity(0, 0, 0), route, alliance, VOLTS, DriveModel.capture(follower));
    }

    void place(Pose pose) {
        body.vf = 0;
        body.vs = 0;
        body.w = 0;
        wheels.command = DrivePowers.zero();
        follower.setPose(pose);
    }

    void step() {
        follower.update(STEP);
    }

    Pose pose() {
        return follower.pose();
    }

    double speed() {
        return Math.hypot(body.vf, body.vs);
    }

    /** What one closed-loop drive saw. */
    static final class Drive {
        double seconds;
        boolean finished;
        /** Least room to the zones by the true rule (TrueRule) along the way, inches. */
        double worstRoom = Double.POSITIVE_INFINITY;
    }

    /**
     * Drives a plan's legs from {@code start}: each leg ends when the robot is at its
     * stop (MotionModel.ARRIVED_*) or settleSeconds + 0.5 s after the handoff, like DriveRoute.
     * {@code points} are the route points the true rule relaxes near.
     */
    Drive drive(Pose start, Plan plan, List<Zone> zones, double radius, double[] points) {
        Drive d = new Drive();
        place(start);
        double t = 0;
        for (Leg leg : plan.legs) {
            Path path = leg.path;
            if (path != null) follower.follow(path);
            else follower.hold(leg.end);
            double legStart = t, handoff = Double.NaN;
            while (t < 60) {
                step();
                t += STEP;
                Pose p = pose();
                d.worstRoom = Math.min(d.worstRoom, TrueRule.room(zones, radius, points, start.heading(), p.x(), p.y(), p.heading()));
                boolean there = Math.hypot(p.x() - leg.end.x(), p.y() - leg.end.y()) < MotionModel.ARRIVED_INCHES
                        && Math.abs(org.firstinspires.ftc.teamcode.util.Angles.wrapRadians(p.heading() - leg.end.heading()))
                        < Math.toRadians(MotionModel.ARRIVED_DEGREES)
                        && speed() < MotionModel.ARRIVED_SPEED;
                if (path != null && !follower.following() && Double.isNaN(handoff)) handoff = t;
                if (path != null && !Double.isNaN(handoff) && (there || t - handoff > leg.settleSeconds() + 0.5)) break;
                if (path == null && t - legStart > leg.seconds + 0.5) break;
            }
        }
        d.seconds = t;
        d.finished = t < 60;
        return d;
    }

    // ---- The simulated drivetrain ----

    /** Takes Foresight's powers, scaled so no wheel goes past 1, like Pedro's Mecanum. */
    static final class Wheels implements Drivetrain {
        DrivePowers command = DrivePowers.zero();

        @Override
        public void drive(DrivePowers p, boolean manual) {
            double fl = p.forward() - p.strafe() - p.turn(), fr = p.forward() + p.strafe() + p.turn();
            double bl = p.forward() + p.strafe() - p.turn(), br = p.forward() - p.strafe() + p.turn();
            double most = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
            command = new DrivePowers(p.forward() / most, p.strafe() / most, p.turn() / most);
        }

        @Override
        public double maxScaling(DrivePowers current, DrivePowers delta) {
            double scale = 1.0;
            double[] c = wheels(current), d = wheels(delta);
            for (int i = 0; i < 4; i++) {
                if (Math.abs(d[i]) < 1e-9) continue;
                double up = (1.0 - c[i]) / d[i], down = (-1.0 - c[i]) / d[i];
                if (up >= 0 && up < scale) scale = up;
                if (down >= 0 && down < scale) scale = down;
            }
            return Math.max(0, Math.min(1, scale));
        }

        static double[] wheels(DrivePowers p) {
            return new double[]{p.forward() - p.strafe() - p.turn(), p.forward() + p.strafe() + p.turn(),
                    p.forward() + p.strafe() - p.turn(), p.forward() - p.strafe() + p.turn()};
        }

        @Override
        public void stop() {
            command = DrivePowers.zero();
        }

        @Override
        public void stop(boolean brake) {
            command = DrivePowers.zero();
        }

        @Override
        public Map<String, Object> debug() {
            return new HashMap<>();
        }

        @Override
        public double interpolateVelocity(double x, double y, double theta) {
            return 1.0 / (Math.abs(Math.cos(theta)) / x + Math.abs(Math.sin(theta)) / y);
        }
    }

    /** The robot's motion (the localizer): forward, strafe and turning, each a lag toward power x volts x top speed. */
    static final class Body implements Localizer {
        final Wheels wheels;
        double x, y, h, vf, vs, w;
        MotionState state = MotionState.zero();

        Body(Wheels wheels) {
            this.wheels = wheels;
        }

        @Override
        public void setPose(Pose p) {
            x = p.x();
            y = p.y();
            h = p.heading();
            rebuild();
        }

        @Override
        public MotionState state() {
            return state;
        }

        @Override
        public void update() {
            DrivePowers c = wheels.command;
            // Battery sag like DriveModel.motorVolts: the volts drop while the motors push hard at low speed.
            double power = Math.min(Math.max(Math.abs(c.forward()) + Math.abs(c.strafe()) + Math.abs(c.turn()), 1e-9), 1);
            double back = Math.abs(vf) / FORWARD + Math.abs(vs) / STRAFE + Math.abs(w) / TURN;
            double volts = (1 + SAG * back) / (1 + SAG * power);
            if (power * volts < back) volts = 1;
            double af = (c.forward() * FORWARD * volts - vf) / TAU_F, as = (c.strafe() * STRAFE * volts - vs) / TAU_S;
            double a = Math.hypot(af, as);
            if (a > DriveModel.TRACTION_ACCEL) {
                af *= DriveModel.TRACTION_ACCEL / a;
                as *= DriveModel.TRACTION_ACCEL / a;
            }
            vf += af * STEP;
            vs += as * STEP;
            // Momentum stays put in the field while the robot turns (the omega x v terms).
            double f = vf, s = vs;
            vf += w * s * STEP;
            vs -= w * f * STEP;
            w += (c.turn() * TURN * volts - w) / TAU_T * STEP;
            double cos = Math.cos(h), sin = Math.sin(h);
            x += (vf * cos - vs * sin) * STEP;
            y += (vf * sin + vs * cos) * STEP;
            h += w * STEP;
            rebuild();
        }

        void rebuild() {
            state = MotionState.ofTwist(new Pose(x, y, h), new Twist(vf, vs, w));
        }

        @Override
        public void reset() {
        }
    }
}
