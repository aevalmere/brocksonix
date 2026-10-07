package org.firstinspires.ftc.teamcode.robot;

/**
 * Problems the drivers must know about. Robot.warnings() says which are active,
 * the telemetry shows each one's text, and Feedback buzzes the gamepad.
 * The order here is the order the lines show up on the screen.
 */
public enum Warning {
    NO_ALLIANCE("!! NO ALLIANCE: restart the OpMode and pick one in init"),
    POSE_UNKNOWN("!! POSE UNKNOWN: relocalize in a corner before shooting"),
    POSE_NAN("!! POSE IS NaN: odometry glitch, not aiming"),
    OUT_OF_RANGE("!! Target out of range"),
    // Each of these has cut its own power, and stays off until the OpMode restarts.
    SHOOTER_OFF("!! SHOOTER OFF (watchdog): check the encoder, then restart the OpMode"),
    TURRET_OFF("!! TURRET OFF (watchdog): check the encoder and the REVERSED flags, then restart the OpMode"),
    // A Panels edit put a shot table's distances out of order, so aiming would use wrong numbers.
    SHOT_TABLE_ORDER("!! SHOT TABLE OUT OF ORDER: distances must increase (fix in Panels, ShotTables)");

    /** The line the drivers see. */
    public final String text;

    Warning(String text) {
        this.text = text;
    }
}
