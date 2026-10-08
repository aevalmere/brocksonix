package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

/**
 * A chain of target poses, driven in order from wherever the robot is when the
 * drive starts. Poses are written for red (flipped for blue), like every other
 * pose in this code.
 *
 * <pre>
 * Route route = new Route()
 *         .through(GATE).heading(Heading.tangent())   // drive through, facing forward
 *         .to(SHOOT_SPOT).heading(Heading.finishBy(0.5))
 *         .to(FLOWER).approach(180, 10).maxSpeed(30); // last 10 in. straight in, at most 30 in/s
 * Route quick = new Route()
 *         .to(36, 48, 180)                            // x, y in inches, heading in degrees
 *         .to(60, 100).heading(Heading.tangent());    // no heading: face forward, arrive any way
 * </pre>
 *
 * to() stops and settles at the pose; through() passes it without slowing.
 * The last waypoint is always a stop. Calls like heading(), approach() and
 * maxSpeed() change the waypoint added just before them.
 *
 * A waypoint given without a heading (to(x, y)) keeps the heading the robot
 * has on the way there, or with Heading.tangent() faces the way it drives and
 * may arrive from any direction (the planner picks the fastest).
 *
 * pickUp() drives over things on the floor intake first, in the fastest order
 * (see Pickup); Plan.pickedUp() says which it gets:
 * <pre>
 * new Route()
 *         .pickUp(new Pose(40, 60), new Pose(52, 66), new Pose(30, 90)).most(2)  // 2 free slots
 *         .to(SHOOT_SPOT);                                                      // then go shoot
 * </pre>
 */
public final class Route {
    private final List<Waypoint> waypoints = new ArrayList<>();

    /** Drive to the pose and stop there. */
    public Route to(Pose redPose) {
        return add(new Waypoint(redPose, null, true));
    }

    /** Drive through the pose without stopping, facing its heading as it passes. */
    public Route through(Pose redPose) {
        return add(new Waypoint(redPose, null, false));
    }

    /** Drive to (x, y) and stop there, facing headingDegrees (0 is +x, counter-clockwise). Inches, written for red. */
    public Route to(double x, double y, double headingDegrees) {
        return to(new Pose(x, y, Math.toRadians(headingDegrees)));
    }

    /** Drive through (x, y) facing headingDegrees as it passes. Inches and degrees, written for red. */
    public Route through(double x, double y, double headingDegrees) {
        return through(new Pose(x, y, Math.toRadians(headingDegrees)));
    }

    /**
     * Drive to (x, y) and stop there, without a heading to end at: the robot keeps
     * the heading it has (or, with Heading.tangent(), faces the way it drives).
     */
    public Route to(double x, double y) {
        return add(new Waypoint(new Pose(x, y, 0), null, true).withoutHeading());
    }

    /** Drive through (x, y) without a heading to pass it at (see to(x, y)). */
    public Route through(double x, double y) {
        return add(new Waypoint(new Pose(x, y, 0), null, false).withoutHeading());
    }

    /**
     * Drive to a pose asked for when the drive starts, and again at every replan,
     * for targets vision finds. It is asked on the main (OpMode) thread, inside
     * Scheduler.execute() or estimate(), never on the planning thread, so it can
     * read what the loop keeps up to date. The pose is in field coordinates, not
     * written for red. If it gives null (or throws) when asked, there is no plan:
     * a drive that is starting ends with NO_PLAN, and a replan keeps the old plan.
     */
    public Route toLive(Supplier<Pose> fieldPose) {
        return add(new Waypoint(null, fieldPose, true));
    }

    /** Like toLive, but drives through without stopping. */
    public Route throughLive(Supplier<Pose> fieldPose) {
        return add(new Waypoint(null, fieldPose, false));
    }

    /**
     * Drive over these targets (POLLEN on the floor, say) intake first, without
     * stopping, in the order that takes the least time, counting the way to the
     * waypoint after them. Written for red; only x and y count. most() and
     * inOrder() change it; maxSpeed() and allowContact() apply to every pass;
     * heading() and approach() don't apply. See Pickup for how it works.
     */
    public Route pickUp(Pose... redTargets) {
        return pickUp(Arrays.asList(redTargets));
    }

    /** Same as pickUp(Pose...), from a list. */
    public Route pickUp(List<Pose> redTargets) {
        return add(Waypoint.pickUp(new Pickup(new ArrayList<>(redTargets), null)));
    }

    /**
     * Like pickUp, with targets asked for when the drive starts (vision), in field
     * coordinates, on the main thread like toLive. Null or NaN targets are left out.
     */
    public Route pickUpLive(Supplier<List<Pose>> fieldTargets) {
        return add(Waypoint.pickUp(new Pickup(null, fieldTargets)));
    }

    /** Pick up at most this many of the last pickUp's targets (the free storage slots), the ones that are quickest to get. */
    public Route most(int count) {
        lastPickup().most = Math.max(0, count);
        return this;
    }

    /** Pick up the last pickUp's targets in the order given, not the quickest order. */
    public Route inOrder() {
        lastPickup().inOrder = true;
        return this;
    }

    /** How to turn on the way to the last waypoint added. */
    public Route heading(Heading heading) {
        last().heading = heading;
        return this;
    }

    /**
     * Arrive at the last waypoint added driving straight in this direction for at
     * least the last {@code inches}. Degrees, 0 is +x, counter-clockwise positive,
     * written for red (field coordinates for a live pose).
     */
    public Route approach(double travelDegrees, double inches) {
        last().approachAngle = Math.toRadians(travelDegrees);
        last().approachLength = Math.max(0, inches);
        return this;
    }

    /**
     * On the way to the last waypoint added, the flower intake is down: AgateFlow
     * plans that stretch with the robot reaching IntakePoses.FLOWER_INTAKE_REACH
     * ahead (10.84 in.) instead of ROBOT_FRONT. Every other stretch is planned with
     * the flower intake up, so lower it only for this stretch (and raise it after).
     * DriveRoute.flowerIntakeDown() says which stretch is being driven. Not for
     * pickUp() (that is the floor intake): its passes always plan with it up.
     */
    public Route flowerIntakeDown() {
        last().flowerDown = true;
        return this;
    }

    /** Speed limit on the way to the last waypoint added, inches/second. */
    public Route maxSpeed(double inchesPerSecond) {
        last().maxSpeed = inchesPerSecond;
        return this;
    }

    /**
     * On the way to the last waypoint added, let the robot's center come as close
     * as AgateFlow.CONTACT_RADIUS to walls and zones, instead of ROBOT_RADIUS +
     * SAFETY_MARGIN. For runs that must hug a wall, like picking up the GARDEN.
     * AgateFlow can't keep a margin there, so keep the robot's narrow side
     * toward the wall (heading) and add a maxSpeed.
     */
    public Route allowContact() {
        last().contact = true;
        return this;
    }

    public List<Waypoint> waypoints() {
        return Collections.unmodifiableList(waypoints);
    }

    public boolean isEmpty() {
        return waypoints.isEmpty();
    }

    /** A route made of these waypoints (AgateFlow uses it to replan what's left). */
    static Route of(List<Waypoint> waypoints) {
        Route route = new Route();
        route.waypoints.addAll(waypoints);
        return route;
    }

    /** This route with every live pose asked for now (see Waypoint.askNow). Call on the main thread. */
    Route askLivePosesNow() {
        Route asked = new Route();
        for (Waypoint waypoint : waypoints) asked.waypoints.add(waypoint.askNow());
        return asked;
    }

    private Route add(Waypoint waypoint) {
        waypoints.add(waypoint);
        return this;
    }

    private Waypoint last() {
        if (waypoints.isEmpty()) throw new IllegalStateException("Add a waypoint (to or through) first");
        return waypoints.get(waypoints.size() - 1);
    }

    private Pickup lastPickup() {
        Waypoint last = last();
        if (last.pickup == null) throw new IllegalStateException("most() and inOrder() go right after pickUp()");
        return last.pickup;
    }
}
