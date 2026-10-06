package org.firstinspires.ftc.teamcode.shot;

/** Everything the turret, shooter and rail need for one shot. */
public class ShotSolution {
    /** Robot-relative turret angle in degrees (0 = robot front, counter-clockwise positive). */
    public final double turretDeg;
    /** Inches from the turret to the point it aims at (the led target when moving). */
    public final double distance;
    public final double rpm;
    public final double feedPower;
    /** False when the target is outside the table's distance range. */
    public final boolean valid;

    public ShotSolution(double turretDeg, double distance, double rpm, double feedPower, boolean valid) {
        this.turretDeg = turretDeg;
        this.distance = distance;
        this.rpm = rpm;
        this.feedPower = feedPower;
        this.valid = valid;
    }
}
