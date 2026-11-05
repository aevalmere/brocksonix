package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.List;

/**
 * The storage rail and the POLLEN on it (ROBOT.md, "Ball path and storage").
 *
 * Each ball is the position of its center, in inches along the rail from the door:
 * 0 is the door, positive is back toward the intake, negative is past the door on
 * the way to the wheel. Slot 1 (against the door) is centered at half a ball, slot 2
 * one ball behind it, and so on to slot 4. The transfer motor moves the balls in
 * slots 1-2, the intake motor the ones further back. Balls never overlap: a ball
 * that is pushed or blocked moves the ones touching it. The closed door stops the
 * front ball; the open door lets it roll to the wheel, which launches it once the
 * wheel is fast enough. A ball pushed out past slot 4 falls out of the intake.
 *
 * The three beams sit on the slot 1, slot 2 and slot 4 centers, as on the robot.
 */
final class RailPlant {
    /** SPEC: a POLLEN ball, about 2.8 in. across (ROBOT.md). */
    static final double BALL = 2.8;
    // TODO(3): Film the rail feeding balls at full power and measure how fast a ball moves along it, in/s.
    /** PLACEHOLDER: ball speed along the rail at full motor speed, in/s (the wheels slip, so far under their surface speed). */
    static final double BALL_SPEED = 40;
    // TODO(6): Measure from the closed door to where the wheel first grabs a ball, in.
    /** PLACEHOLDER: how far past the door a ball's center goes before the wheel takes it. */
    static final double DOOR_TO_WHEEL = 2.0;
    /** Where the transfer's wheels end and the intake's begin: between slot 2 and slot 3. */
    static final double TRANSFER_END = 2 * BALL;
    /** Past the slot 4 center by a whole ball, a ball has left the rail out the intake end. */
    static final double INTAKE_END = 4.5 * BALL;
    /** A beam reads blocked while a ball's center is within this of it: the beam is thinner than the ball. */
    static final double BEAM_REACH = 0.4 * BALL;

    static final double SLOT_1 = 0.5 * BALL, SLOT_2 = 1.5 * BALL, SLOT_4 = 3.5 * BALL;

    /** Ball centers, front (nearest the door) first. */
    private final List<Double> balls = new ArrayList<>();
    private int launched = 0;
    private int spilled = 0;

    /** Fills slots 1 to count, touching each other and the door, as a preload sits. */
    void load(int count) {
        balls.clear();
        for (int i = 0; i < count; i++) balls.add(SLOT_1 + i * BALL);
    }

    /**
     * One tick.
     * @param transferSpeed how fast the transfer moves balls toward the door, in/s (negative: away)
     * @param intakeSpeed   the same for the intake's part of the rail
     * @param doorOpen      the door is far enough open for a ball to pass
     * @param wheelCanLaunch the wheel is fast enough to throw a ball that reaches it
     * @return how many balls the wheel launched this tick
     */
    int update(double dt, double transferSpeed, double intakeSpeed, boolean doorOpen, boolean wheelCanLaunch) {
        // Where each ball would go on its own, then front to back so none passes the door
        // (closed), the wheel (not launching yet) or the ball ahead of it.
        double ahead = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < balls.size(); i++) {
            double s = balls.get(i);
            double speed = s < TRANSFER_END ? transferSpeed : intakeSpeed;
            double moved = s - speed * dt;
            if (!doorOpen && s >= SLOT_1 - 1e-9) moved = Math.max(moved, SLOT_1);
            moved = Math.max(moved, -DOOR_TO_WHEEL);
            moved = Math.max(moved, ahead + BALL);
            balls.set(i, moved);
            ahead = moved;
        }

        int launchedNow = 0;
        while (!balls.isEmpty() && wheelCanLaunch && balls.get(0) <= -DOOR_TO_WHEEL + 1e-9) {
            balls.remove(0);
            launchedNow++;
        }
        while (!balls.isEmpty() && balls.get(balls.size() - 1) > INTAKE_END) {
            balls.remove(balls.size() - 1);
            spilled++;
        }
        launched += launchedNow;
        return launchedNow;
    }

    boolean blocked(double beam) {
        for (double s : balls) {
            if (Math.abs(s - beam) < BEAM_REACH) return true;
        }
        return false;
    }

    /** Balls still in the robot: on the rail, or at the wheel waiting for it to spin up. */
    int held() {
        return balls.size();
    }

    int launched() {
        return launched;
    }

    int spilled() {
        return spilled;
    }
}
