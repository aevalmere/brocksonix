package org.firstinspires.ftc.teamcode.robot.shot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;

/**
 * Predicts how the robot is about to turn from the turn power sent to the wheels, so
 * the turret can turn the other way at the same moment instead of waiting for the
 * Pinpoint to see the heading change.
 *
 * The model: full turn power spins the robot at MAX_TURN_RATE_DEG_PER_SEC, and the robot
 * gets to a new turn rate the way a first-order system does, closing 63% of the gap every
 * TURN_TIME_CONSTANT_SEC (it has to speed up and slow down, so it doesn't follow the stick
 * at once). Let go of the stick and the model keeps turning for a moment, like the robot.
 *
 * Two outputs, both for LEAD_SEC ahead (the turret's own delay: loop time, servo reaction):
 *   - headingAheadDeg(): how far the robot will have turned by then. Aim at the goal from
 *     the heading plus this, and the turret is already there when the robot is.
 *   - feedforwardRateDegPerSec(): how fast the robot will be turning then, plus extra
 *     while it speeds up or slows down (TURRET_RESPONSE_SEC), because the turret needs
 *     time to speed up too. The turret's target moves at minus this, so it goes to
 *     Turret.setTargetRateDegPerSec as -rate, where KV turns it into a push with no lag.
 *
 * Only the lead and the feedforward come from the model. The aim itself is still worked out
 * every loop from the real Pinpoint pose, so a wrong model number makes it lag or lead a
 * little while turning, never aim at the wrong spot when still. Being shoved isn't in the
 * model: the PID catches that, as before.
 *
 * Pedro's Mecanum scales all four wheel powers down together when one would go past 1, so
 * turning while driving at full stick turns slower. effectiveTurn() does the same.
 */
@Configurable
public class TurnPredictor {
    /** Off: no lead and no feedforward from the stick (the turret works as before). */
    public static boolean ENABLED = true;
    // TODO(11): Continuous Test, robot on the field: hold the turn stick fully (no drive stick, slow mode off) and read "Yaw rate measured" once it is steady.
    /** How fast the robot turns at full turn power, deg/s. */
    public static double MAX_TURN_RATE_DEG_PER_SEC = 360;
    // TODO(11): Continuous Test: snap the turn stick fully and back. "Yaw rate predicted" should rise and fall with "Yaw rate measured": raise this if predicted gets there first, lower it if measured does.
    /** Seconds for the robot to close 63% of the gap to a new turn rate. */
    public static double TURN_TIME_CONSTANT_SEC = 0.15;
    // TODO(11): Continuous Test, tracking on: turn in place. Raise until the orange turret line stops trailing the yellow line, lower if it runs ahead.
    /** How far ahead to predict, seconds: the turret's own delay. */
    public static double LEAD_SEC = 0.05;
    // TODO(11): Continuous Test, tracking and prediction on: snap the turn stick from 0 to full. Raise until the orange turret line no longer falls behind the yellow one at the start of the turn; lower if it jumps ahead of it.
    /**
     * The turret needs time to speed up too. While the robot speeds up (or slows down), the
     * turret is asked for this many seconds' worth of the robot's acceleration on top of the
     * robot's speed, so it is already turning fast enough when the robot is. A harder stick
     * jump means more acceleration, so more extra push; at a steady turn it adds nothing.
     * 0 = off.
     */
    public static double TURRET_RESPONSE_SEC = 0.08;

    // Guards against dividing by 0 if Panels sets the time constant to 0.
    private static final double MIN_TIME_CONSTANT = 1e-3;

    private double rateDegPerSec = 0;
    private double commandedDegPerSec = 0;

    /** Call once per loop with the robot-centric powers sent to the wheels this loop, and the seconds since the last call. */
    public void update(DrivePowers sent, double dt) {
        commandedDegPerSec = effectiveTurn(sent) * MAX_TURN_RATE_DEG_PER_SEC;
        if (Double.isNaN(commandedDegPerSec)) commandedDegPerSec = 0;
        double tau = Math.max(TURN_TIME_CONSTANT_SEC, MIN_TIME_CONSTANT);
        rateDegPerSec = commandedDegPerSec + (rateDegPerSec - commandedDegPerSec) * Math.exp(-Math.max(dt, 0) / tau);
    }

    /** The model's turn rate now, deg/s, counter-clockwise positive. */
    public double rateDegPerSec() {
        return rateDegPerSec;
    }

    /** How fast the robot will be turning LEAD_SEC from now, deg/s. 0 while off. */
    public double rateAheadDegPerSec() {
        if (!ENABLED) return 0;
        double tau = Math.max(TURN_TIME_CONSTANT_SEC, MIN_TIME_CONSTANT);
        return commandedDegPerSec + (rateDegPerSec - commandedDegPerSec) * Math.exp(-Math.max(LEAD_SEC, 0) / tau);
    }

    /** How fast the robot's turn rate will be changing LEAD_SEC from now, deg/s². 0 while off. */
    public double accelAheadDegPerSec2() {
        if (!ENABLED) return 0;
        double tau = Math.max(TURN_TIME_CONSTANT_SEC, MIN_TIME_CONSTANT);
        return (commandedDegPerSec - rateAheadDegPerSec()) / tau;
    }

    /**
     * The robot turn rate the turret should be fed, deg/s: the rate LEAD_SEC ahead plus
     * TURRET_RESPONSE_SEC of the acceleration then, so the turret speeds up in time.
     * The turret's target turns at minus this. 0 while off.
     */
    public double feedforwardRateDegPerSec() {
        return rateAheadDegPerSec() + Math.max(TURRET_RESPONSE_SEC, 0) * accelAheadDegPerSec2();
    }

    /** How many degrees the robot will turn in the next LEAD_SEC (the rate above, added up over that time). 0 while off. */
    public double headingAheadDeg() {
        if (!ENABLED) return 0;
        double tau = Math.max(TURN_TIME_CONSTANT_SEC, MIN_TIME_CONSTANT);
        double lead = Math.max(LEAD_SEC, 0);
        return commandedDegPerSec * lead + (rateDegPerSec - commandedDegPerSec) * tau * (1 - Math.exp(-lead / tau));
    }

    /** The turn power the wheels really get: Pedro's Mecanum scales every wheel down together so none goes past 1. */
    static double effectiveTurn(DrivePowers p) {
        double fl = p.forward() - p.strafe() - p.turn();
        double fr = p.forward() + p.strafe() + p.turn();
        double bl = p.forward() + p.strafe() - p.turn();
        double br = p.forward() - p.strafe() + p.turn();
        double most = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        return p.turn() / most;
    }
}
