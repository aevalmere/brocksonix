package org.firstinspires.ftc.teamcode.robot;

/** Which hood the shooter uses. Robot.HOOD picks it, live in Panels. */
public enum HoodType {
    /**
     * Fixed launch angle: ShotTables.SHOT_RPM sets the range. If the coax hood is on the
     * robot anyway, it holds CoaxHood.STATIC_LAUNCH_DEG, so the same tables still fit.
     */
    STATIC,
    /** The coax hood: ShotTables.COAX_RPM sets the wheel speed, HoodSolver the angle from the wheel's real speed. */
    COAX
}
