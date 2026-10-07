package org.firstinspires.ftc.teamcode.shot;

/** Everything the turret, shooter and rail need for one shot. */
public class ShotSolution {
    /** Robot-relative turret angle in degrees (0 = robot front, counter-clockwise positive). */
    public final double turretDeg;
    /** Inches from the turret to the point it aims at (the led target when moving). RPM, feed power and valid all use this. */
    public final double distance;
    /** Inches from the turret to the target itself. Same as distance unless LEAD moved the aim point. Only for telemetry. */
    public final double targetDistance;
    public final double rpm;
    public final double feedPower;
    /** False when the aim distance is outside the table's distance range. */
    public final boolean valid;

    public ShotSolution(double turretDeg, double distance, double targetDistance,
                        double rpm, double feedPower, boolean valid) {
        this.turretDeg = turretDeg;
        this.distance = distance;
        this.targetDistance = targetDistance;
        this.rpm = rpm;
        this.feedPower = feedPower;
        this.valid = valid;
    }
}
