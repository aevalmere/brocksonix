package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.fakehardware.FakeMotor;

/**
 * The shooter wheel: two motors on one flywheel, mounted opposite each other, so
 * one spins its shaft backward to push the wheel forward. The wheel's speed
 * follows the motors' average push with a first-order lag, faster on a fuller
 * battery, and every ball it launches takes a dip out of it. Both motors' encoders
 * are then told how fast the wheel really turns, so Shooter reads this speed, and
 * FireControl's shot counter sees each dip.
 */
final class FlywheelPlant {
    /** SPEC: a goBILDA 6000 RPM bare motor's free speed, wheel driven 1:1 (ROBOT.md). */
    static final double FREE_RPM = 6000;
    /** SPEC: the voltage a motor's free speed is rated at. */
    static final double RATED_VOLTS = 12;
    // TODO(6): Shooter Test, step from 0 to 3500 RPM: the time to reach 63% of the way there.
    /** PLACEHOLDER: seconds for the wheel to close 63% of the gap to a new speed. */
    static final double TIME_CONSTANT = 0.6;
    // TODO(6): Shooter Test, fire one ball at 3000 RPM: how far the RPM drops.
    /** PLACEHOLDER: RPM each ball takes out of the wheel. */
    static final double DIP_PER_BALL_RPM = 400;
    // TODO(6): The slowest wheel that still throws a ball out instead of jamming it.
    /** PLACEHOLDER: below this RPM a ball at the wheel stays there. */
    static final double MIN_LAUNCH_RPM = 1000;

    private final FakeMotor left, right;
    /** +1 or -1: which way each motor's shaft turns when the wheel turns forward. */
    private final double leftMount, rightMount;
    private double rpm = 0;

    FlywheelPlant(FakeMotor left, double leftMount, FakeMotor right, double rightMount) {
        this.left = left;
        this.right = right;
        this.leftMount = leftMount;
        this.rightMount = rightMount;
    }

    /** One tick: spin toward what the motors push for, then lose a dip for each ball launched this tick. */
    void update(double dt, double batteryVolts, int ballsLaunched) {
        double push = (left.getAppliedPhysicalPower() * leftMount + right.getAppliedPhysicalPower() * rightMount) / 2;
        double target = push * batteryVolts / RATED_VOLTS * FREE_RPM;
        rpm += (target - rpm) * (1 - StrictMath.exp(-dt / TIME_CONSTANT));
        for (int i = 0; i < ballsLaunched; i++) rpm = Math.max(rpm - DIP_PER_BALL_RPM, 0);

        // What each encoder counts: the wheel's speed, in that motor's own direction.
        double fraction = Math.max(-1, Math.min(1, rpm / FREE_RPM));
        left.setMeasuredShaftSpeedFraction(fraction * leftMount);
        right.setMeasuredShaftSpeedFraction(fraction * rightMount);
    }

    boolean canLaunch() {
        return rpm >= MIN_LAUNCH_RPM;
    }
}
