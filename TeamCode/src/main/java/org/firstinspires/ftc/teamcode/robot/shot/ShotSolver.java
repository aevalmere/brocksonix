package org.firstinspires.ftc.teamcode.robot.shot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.CoaxHood;
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
 * 3. PREDICT: solve from where the robot will be, not where it is now.
 *    MotionPredictor does the predicting.
 *    - Two moments. The aim keeps updating every loop while a ball is fed, so
 *      the last aim before it leaves is about one loop old (RELEASE_LATENCY_SEC):
 *      distance, RPM, hood and flight time use the pose then, and LEAD uses the
 *      velocity then. The turret needs TURRET_LATENCY_SEC to get to a new angle, so it
 *      aims from the pose then, and is on target when the ball leaves.
 *    - Pedro's velocity is a smoothed difference, so it is VELOCITY_LATENCY_SEC
 *      old. It is moved forward that much using the acceleration (USE_ACCEL).
 *    - A braking robot stops and stays stopped: the prediction never runs on
 *      backward. So a robot that brakes and fires gets no lead the wrong way.
 *    The acceleration estimate updates on every shot solve (every loop while
 *    aiming at the HIVE), also with PREDICT off, so it is ready the moment
 *    PREDICT is turned on.
 *
 * PREDICT follows GNCE's DECODE (2025-26) robot Ditto, https://github.com/GNCE/DECODEv2-Ditto:
 * ShotPlanner's plan, predictWithStop and compensateVel in
 * https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/core/util/ShotPlanner.java
 *
 * Passes use layer 1 only: they land on the floor and don't need to be exact.
 *
 * With the coax hood (Robot.HOOD = COAX), solveCoaxShot does the same three
 * layers, but the RPM comes from COAX_RPM and the hood angle from HoodSolver,
 * using the RPM the flywheel really has right now. LEAD's flight time comes from
 * the same physics instead of the SHOT_FLIGHT_TIME table. solveCoaxPass works
 * out passes from the physics too, so the hood makes up for RPM error there as well.
 */
@Configurable
public class ShotSolver {
    // TODO(11): Only after stationary shots work. Turn on, drive steadily past the HIVE and fire. Lands ahead of the cell = flight times too short; behind = too long.
    public static boolean LEAD = false;
    // TODO(11): Turn on after LEAD works.
    public static boolean PREDICT = false;
    // TODO(11): With PREDICT on: if the turret lags while driving, raise it; if it aims ahead, lower it.
    /** Seconds the turret takes to get to a new angle. It aims from where the robot will be then. */
    public static double TURRET_LATENCY_SEC = 0.08;
    // TODO(11): About one loop (Loop ms in telemetry, 0.02 to 0.04 s). Not the feed time: the aim keeps updating while the ball is fed.
    /** How old the last aim is when the ball leaves. Distance, RPM, hood and LEAD use where the robot will be then. */
    public static double RELEASE_LATENCY_SEC = 0.03;
    // TODO(11): With LEAD and PREDICT on, drive past the HIVE, brake hard and fire as it stops. Lands behind the cell (along the way it drove) = raise it; ahead = lower it.
    /** How old Pedro's velocity is (it's smoothed). PREDICT moves it forward this much using the acceleration. */
    public static double VELOCITY_LATENCY_SEC = 0.06;
    // TODO(11): Only if the aim twitches while driving at a steady speed: raise ACCEL_FILTER toward 0.9, or set this false.
    /** PREDICT uses the acceleration. Off: constant velocity, and VELOCITY_LATENCY_SEC does nothing. */
    public static boolean USE_ACCEL = true;
    /** Smoothing for the acceleration: 0 = none, closer to 1 = smoother but slower to see a brake. */
    public static double ACCEL_FILTER = 0.7;
    // TODO(11): Only if hard brakes still land behind the cell after VELOCITY_LATENCY_SEC: raise it toward DriveModel.TRACTION_ACCEL.
    /** Biggest believable acceleration, in/s². A bigger change from one loop to the next is cut down to this. */
    public static double MAX_ACCEL = 200;
    // TODO(11): Only if PREDICT's turret aim lags when the robot starts or stops spinning hard: raise it. If the aim twitches while turning steadily, lower it.
    /** The same for turning, rad/s². */
    public static double MAX_ANGULAR_ACCEL = 20;
    public static int LEAD_ITERATIONS = 3;

    private final MotionPredictor motion = new MotionPredictor();
    private final ElapsedTime clock = new ElapsedTime();
    /** The velocity LEAD used in the last shot solve, for telemetry. */
    private Velocity lastLaunchVelocity = new Velocity(0, 0, 0);

    /** Static hood: the launch angle is fixed, so RPM is the only thing that sets the range. */
    public ShotSolution solveShot(Pose robot, Velocity velocity, Pose target) {
        return solve(robot, velocity, target, false, 0, clock.seconds());
    }

    /**
     * Coax hood: RPM from COAX_RPM, and the launch angle that scores at measuredRpm, the
     * speed the wheel really has right now. The hood moves much faster than the wheel can
     * change speed, so a wheel that dipped after a shot, or is still spinning up, still scores.
     */
    public ShotSolution solveCoaxShot(Pose robot, Velocity velocity, Pose target, double measuredRpm) {
        return solve(robot, velocity, target, true, measuredRpm, clock.seconds());
    }

    /** nowSeconds is the time, for the acceleration estimate. Not private, so the tests can pass their own. */
    ShotSolution solve(Pose robot, Velocity velocity, Pose target, boolean coax, double measuredRpm, double nowSeconds) {
        // Also with PREDICT off, so the estimate is ready the moment PREDICT is turned on.
        motion.update(velocity, nowSeconds);

        // Distance, RPM, hood and flight time are solved from shotFrom, LEAD uses the
        // velocity the ball leaves with, and the turret aims from turretFrom.
        // Without PREDICT, all of them are the robot right now.
        Pose shotFrom = robot;
        Velocity launchVelocity = velocity;
        Pose turretFrom = robot;
        if (PREDICT) {
            MotionPredictor.Prediction atRelease = motion.predict(robot, velocity, RELEASE_LATENCY_SEC);
            shotFrom = atRelease.pose;
            launchVelocity = atRelease.velocity;
            turretFrom = motion.predict(robot, velocity, TURRET_LATENCY_SEC).pose;
        }
        lastLaunchVelocity = launchVelocity;

        double aimX = target.x();
        double aimY = target.y();
        if (LEAD) {
            for (int i = 0; i < LEAD_ITERATIONS; i++) {
                double distance = Math.hypot(aimX - shotFrom.x(), aimY - shotFrom.y());
                double flightTime = coax ? coaxFlightTime(distance, measuredRpm)
                        : LookupTable.at(ShotTables.SHOT_FLIGHT_TIME, distance);
                aimX = target.x() - launchVelocity.vx * flightTime;
                aimY = target.y() - launchVelocity.vy * flightTime;
            }
        }

        double distance = Math.hypot(aimX - shotFrom.x(), aimY - shotFrom.y());
        // The real distance to the target, so telemetry can show it next to the led one.
        double targetDistance = Math.hypot(target.x() - shotFrom.x(), target.y() - shotFrom.y());
        double[][] rpmTable = coax ? ShotTables.COAX_RPM : ShotTables.SHOT_RPM;
        double launchDeg = coax
                ? HoodSolver.launchDeg(distance, measuredRpm, CoaxHood.MIN_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG)
                : Double.NaN;
        return new ShotSolution(
                turretAngle(turretFrom, aimX, aimY),
                distance,
                targetDistance,
                LookupTable.at(rpmTable, distance),
                LookupTable.at(ShotTables.SHOT_FEED_POWER, distance),
                inRange(rpmTable, distance),
                launchDeg);
    }

    /**
     * Shoot-on-the-move numbers from the last shot solve, for tuning VELOCITY_LATENCY_SEC (GNCE
     * Ditto showed the same): the velocity LEAD used, next to the measured one. Braking hard to a
     * stop, the used one should drop to about 0 while the measured one still lags behind.
     */
    public void addTelemetry(Telemetry telemetry, Velocity measured) {
        telemetry.addData("Move shot", "lead velocity %.0f, %.0f in/s (measured %.0f, %.0f), accel %.0f, %.0f in/s^2",
                lastLaunchVelocity.vx, lastLaunchVelocity.vy, measured.vx, measured.vy, motion.ax(), motion.ay());
    }

    /**
     * Flight time from the physics, at the angle the hood would use. When no angle scores
     * (the wheel is still spinning up, say) it falls back to the static hood's table: the
     * shot won't fire then anyway, it just keeps the aim point sensible.
     */
    private static double coaxFlightTime(double distance, double measuredRpm) {
        double launchDeg = HoodSolver.launchDeg(distance, measuredRpm, CoaxHood.MIN_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG);
        double seconds = HoodSolver.flightTime(distance, measuredRpm, launchDeg);
        return Double.isNaN(seconds) ? LookupTable.at(ShotTables.SHOT_FLIGHT_TIME, distance) : seconds;
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
                inRange(ShotTables.PASS_RPM, distance),
                Double.NaN);
    }

    /**
     * Coax hood pass: a lob at about CoaxHood.PASS_LAUNCH_DEG. The RPM is what lands it on the
     * pass target at that angle, and the hood angle is solved from measuredRpm, like a shot.
     * Same distance range as the PASS_RPM table, for the same reason (G405).
     */
    public ShotSolution solveCoaxPass(Pose robot, Pose target, double measuredRpm) {
        double distance = Math.hypot(target.x() - robot.x(), target.y() - robot.y());
        double lobDeg = Math.max(CoaxHood.MIN_LAUNCH_DEG, Math.min(CoaxHood.PASS_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG));
        double rpm = HoodSolver.passRpm(distance, lobDeg);
        double launchDeg = HoodSolver.passLaunchDeg(distance, measuredRpm, lobDeg,
                CoaxHood.MIN_LAUNCH_DEG, CoaxHood.MAX_LAUNCH_DEG);
        boolean possible = !Double.isNaN(rpm);
        return new ShotSolution(
                turretAngle(robot, target.x(), target.y()),
                distance,
                distance,
                possible ? rpm : 0,
                LookupTable.at(ShotTables.PASS_FEED_POWER, distance),
                possible && inRange(ShotTables.PASS_RPM, distance),
                launchDeg);
    }

    private static double turretAngle(Pose robot, double aimX, double aimY) {
        double fieldAngle = Math.atan2(aimY - robot.y(), aimX - robot.x());
        return Angles.wrapDegrees(Math.toDegrees(fieldAngle - robot.heading()));
    }

    private static boolean inRange(double[][] table, double distance) {
        return distance >= LookupTable.minX(table) && distance <= LookupTable.maxX(table);
    }
}
