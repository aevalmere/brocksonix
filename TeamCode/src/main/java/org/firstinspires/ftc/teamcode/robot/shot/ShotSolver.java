package org.firstinspires.ftc.teamcode.robot.shot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.util.Angles;
import org.firstinspires.ftc.teamcode.util.LookupTable;

/**
 * Turns "robot is here, moving like this, target is there" into a turret
 * angle and an RPM. Built in layers, each with its own switch so it can be
 * turned on or off on the field (LEAD and PREDICT start off):
 *
 * 1. Static: aim at the target, RPM from distance. Always on.
 * 2. LEAD: a ball keeps the robot's velocity, so aim at
 *    target - velocity * flightTime instead. Repeated a few times, because
 *    moving the aim point changes the distance, which changes the flight time.
 * 3. PREDICT: the turret needs time to react, so solve from where the robot
 *    will be TURRET_LATENCY_SEC from now.
 *
 * Passes use layer 1 only: they land on the floor and don't need to be exact.
 */
@Configurable
public class ShotSolver {
    // TODO(9): Only after stationary shots work. Turn on, drive steadily past the HIVE and fire. Lands ahead of the cell = flight times too short; behind = too long.
    public static boolean LEAD = false;
    // TODO(9): Turn on after LEAD works.
    public static boolean PREDICT = false;
    // TODO(9): With PREDICT on: if the turret lags while driving, raise it; if it aims ahead, lower it.
    public static double TURRET_LATENCY_SEC = 0.08;
    public static int LEAD_ITERATIONS = 3;

    public ShotSolution solveShot(Pose robot, Velocity velocity, Pose target) {
        Pose from = PREDICT ? predict(robot, velocity, TURRET_LATENCY_SEC) : robot;

        double aimX = target.x();
        double aimY = target.y();
        if (LEAD) {
            for (int i = 0; i < LEAD_ITERATIONS; i++) {
                double distance = Math.hypot(aimX - from.x(), aimY - from.y());
                double flightTime = LookupTable.at(ShotTables.SHOT_FLIGHT_TIME, distance);
                aimX = target.x() - velocity.vx * flightTime;
                aimY = target.y() - velocity.vy * flightTime;
            }
        }

        double distance = Math.hypot(aimX - from.x(), aimY - from.y());
        // The real distance to the target, so telemetry can show it next to the led one.
        double targetDistance = Math.hypot(target.x() - from.x(), target.y() - from.y());
        return new ShotSolution(
                turretAngle(from, aimX, aimY),
                distance,
                targetDistance,
                LookupTable.at(ShotTables.SHOT_RPM, distance),
                LookupTable.at(ShotTables.SHOT_FEED_POWER, distance),
                inRange(ShotTables.SHOT_RPM, distance));
    }

    /**
     * Lob to the pass target. Only valid inside the table's distance range:
     * a pass from too far away would need enough speed to clear the wall,
     * and a ball sent out of the field is a MAJOR FOUL (G405).
     */
    public ShotSolution solvePass(Pose robot, Pose target) {
        double distance = Math.hypot(target.x() - robot.x(), target.y() - robot.y());
        return new ShotSolution(
                turretAngle(robot, target.x(), target.y()),
                distance,
                distance, // no lead on a pass, so both distances are the same
                LookupTable.at(ShotTables.PASS_RPM, distance),
                LookupTable.at(ShotTables.PASS_FEED_POWER, distance),
                inRange(ShotTables.PASS_RPM, distance));
    }

    /** Where the robot will be after `seconds`, assuming it keeps its current velocity. */
    private static Pose predict(Pose robot, Velocity velocity, double seconds) {
        return new Pose(
                robot.x() + velocity.vx * seconds,
                robot.y() + velocity.vy * seconds,
                robot.heading() + velocity.omega * seconds);
    }

    private static double turretAngle(Pose robot, double aimX, double aimY) {
        double fieldAngle = Math.atan2(aimY - robot.y(), aimX - robot.x());
        return Angles.wrapDegrees(Math.toDegrees(fieldAngle - robot.heading()));
    }

    private static boolean inRange(double[][] table, double distance) {
        return distance >= LookupTable.minX(table) && distance <= LookupTable.maxX(table);
    }
}
