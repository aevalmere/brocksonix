package org.firstinspires.ftc.teamcode.robot.shot;

/** Everything the turret, shooter, hood and rail need for one shot. */
public class ShotSolution {
    /** Robot-relative turret angle in degrees (0 = robot front, counter-clockwise positive). */
    public final double turretDeg;
    /** Inches from the turret to the point it aims at (the led target when moving). RPM, feed power, valid and the hood angle all use this. */
    public final double distance;
    /** Inches from the turret to the target itself. Same as distance unless LEAD moved the aim point. Only for telemetry. */
    public final double targetDistance;
    public final double rpm;
    public final double feedPower;
    /**
     * False when the aim distance is outside the table's distance range: SHOT_RPM or COAX_RPM for
     * shots, PASS_RPM for passes with either hood (G405). A coax pass is also false when no RPM
     * lands it at CoaxHood.PASS_LAUNCH_DEG.
     */
    public final boolean valid;
    /**
     * Coax hood only: launch angle (degrees above horizontal) that scores (a shot) or lands on the
     * pass target (a pass) at the flywheel's measured RPM. NaN with the static hood, and when no
     * hood angle works at this RPM.
     */
    public final double launchDeg;

    public ShotSolution(double turretDeg, double distance, double targetDistance,
                        double rpm, double feedPower, boolean valid, double launchDeg) {
        this.turretDeg = turretDeg;
        this.distance = distance;
        this.targetDistance = targetDistance;
        this.rpm = rpm;
        this.feedPower = feedPower;
        this.valid = valid;
        this.launchDeg = launchDeg;
    }

    /** Coax hood only: true when some hood angle scores at the flywheel's measured RPM. */
    public boolean hoodCanScore() {
        return !Double.isNaN(launchDeg);
    }
}
