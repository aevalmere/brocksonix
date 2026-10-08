package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;

import java.util.function.Supplier;

/**
 * One target in a Route. Made by Route; read by AgateFlow.
 *
 * A pose is either written for red (flipped for blue when the route is
 * planned) or "live": a Supplier asked for a field pose, for targets vision
 * finds. AgateFlowCommands asks it on the main (OpMode) thread each time it
 * plans (the drive starting, every replan, every estimate), so the supplier
 * never runs on the planning thread. Live poses are already in field
 * coordinates, so they aren't flipped, and neither is their approach angle.
 * The Heading is always written for red (like facing our HIVE) and is
 * flipped for blue either way.
 */
public final class Waypoint {
    final Pose redPose;
    final Supplier<Pose> livePose;
    /** True: stop and settle here. False: drive through without slowing for it. */
    final boolean stop;
    /** The waypoint in the Route this one was copied from by askNow(), or itself. */
    final Waypoint original;

    Heading heading = Heading.linear();
    /** Direction of travel for the last approachLength inches before this point, radians. NaN: any. */
    double approachAngle = Double.NaN;
    double approachLength = 0;
    /** Speed limit on the way to this point, inches/second. NaN: none. */
    double maxSpeed = Double.NaN;
    /** On the way to this point the robot may come as close as CONTACT_RADIUS to zones (see Route.allowContact). */
    boolean contact = false;
    /** No heading was given (Route.to(x, y)): keep the heading, or with Heading.tangent() arrive any way. */
    boolean anyHeading = false;
    /** On the way to this point the flower intake is down (Route.flowerIntakeDown): the robot reaches further ahead. */
    boolean flowerDown = false;
    /** Route.pickUp: the targets to pick up. Null for an ordinary waypoint. AgateFlow turns it into passes. */
    Pickup pickup;
    /** A pass made from a pickUp: its speed limit near the waypoint (see AgateFlow.PICKUP_SPEED), in/s. NaN: none. */
    double pickupSpeed = Double.NaN;
    /** A pass made from a pickUp: the targets it takes, x, y pairs in field coordinates. Null otherwise. */
    double[] targets;

    Waypoint(Pose redPose, Supplier<Pose> livePose, boolean stop) {
        this(redPose, livePose, stop, null);
    }

    private Waypoint(Pose redPose, Supplier<Pose> livePose, boolean stop, Waypoint original) {
        this.redPose = redPose;
        this.livePose = livePose;
        this.stop = stop;
        this.original = original != null ? original : this;
    }

    /** A Route.pickUp: no pose of its own, AgateFlow works out the passes. */
    static Waypoint pickUp(Pickup pickup) {
        Waypoint waypoint = new Waypoint(null, null, false);
        waypoint.pickup = pickup;
        return waypoint;
    }

    /**
     * One pass of a pickUp, made by AgateFlow: drive through (or stop at, if
     * {@code stop}) this field pose facing its heading, straight in along that
     * heading for PICKUP_RUN inches, at most pickupSpeed near it. A fixed pose in
     * field coordinates, so a replan drives the same pass.
     */
    static Waypoint pass(Pose fieldPose, boolean stop, double maxSpeed, boolean contact, double pickupSpeed, double[] targets) {
        Waypoint waypoint = new Waypoint(null, () -> fieldPose, stop);
        waypoint.targets = targets;
        waypoint.heading = Heading.tangent();
        waypoint.approachAngle = fieldPose.heading();
        waypoint.approachLength = AgateFlow.PICKUP_RUN;
        waypoint.maxSpeed = maxSpeed;
        waypoint.contact = contact;
        waypoint.pickupSpeed = pickupSpeed;
        return waypoint;
    }

    /** This waypoint, marked as having no heading of its own (Route.to(x, y)). */
    Waypoint withoutHeading() {
        anyHeading = true;
        return this;
    }

    /**
     * For a live waypoint: a copy that holds the pose its supplier gives right
     * now, so planning on another thread doesn't call the supplier. Call on the
     * main thread. A supplier that throws counts as giving null (no pose).
     * Other waypoints are returned as they are.
     */
    Waypoint askNow() {
        if (pickup != null) {
            Pickup asked = pickup.askNow();
            if (asked == pickup) return this;
            Waypoint copy = pickUp(asked);
            copy.maxSpeed = maxSpeed;
            copy.contact = contact;
            return copy;
        }
        if (livePose == null) return this;
        Pose asked;
        try {
            asked = livePose.get();
        } catch (RuntimeException e) {
            asked = null;
        }
        final Pose now = asked;
        Waypoint copy = new Waypoint(null, () -> now, stop, original);
        copy.heading = heading;
        copy.approachAngle = approachAngle;
        copy.approachLength = approachLength;
        copy.maxSpeed = maxSpeed;
        copy.contact = contact;
        copy.anyHeading = anyHeading;
        copy.pickupSpeed = pickupSpeed;
        copy.targets = targets;
        copy.flowerDown = flowerDown;
        return copy;
    }

    /** This waypoint with its pose worked out for the alliance, ready to plan. Null if a live pose isn't available. */
    Resolved resolve(Alliance alliance) {
        Pose pose;
        double approach = approachAngle;
        if (livePose != null) {
            pose = livePose.get();
            if (pose == null) return null;
        } else if (alliance == Alliance.BLUE) {
            pose = alliance.fromRed(redPose);
            if (!Double.isNaN(approach)) approach += Math.PI;
        } else {
            pose = redPose;
        }
        // A NaN anywhere (vision lost the target, say) means there is no pose, like null.
        if (Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading()) || Double.isInfinite(pose.heading())
                || pose.x() < 0 || pose.y() < 0 || pose.x() > Field.SIZE || pose.y() > Field.SIZE) {
            return null;
        }
        double h = anyHeading ? Double.NaN : pose.heading();
        return new Resolved(pose.x(), pose.y(), h, stop, heading.forAlliance(alliance),
                approach, approachLength, maxSpeed, contact, pickupSpeed, targets, flowerDown, original);
    }

    /**
     * A waypoint in field coordinates. AgateFlow may move x, y if it was inside a
     * zone, and may make a through-waypoint a stop if it can't be driven through
     * smoothly (see AgateFlow).
     */
    static final class Resolved {
        double x, y;
        /** NaN if the waypoint has no heading of its own (see Route.to(x, y)). */
        final double heading;
        boolean stop;
        final Heading turnRule;
        final double approachAngle, approachLength, maxSpeed;
        final boolean contact;
        /** A pickUp pass's speed limit near it, in/s, or NaN; and the targets it takes (x, y pairs), or null. */
        final double pickupSpeed;
        final double[] targets;
        /** The flower intake is down on the way here (Route.flowerIntakeDown). */
        final boolean flowerDown;
        /** The Route's own waypoint (never an askNow copy), so a replan asks a live pose again. */
        final Waypoint source;

        Resolved(double x, double y, double heading, boolean stop, Heading turnRule,
                 double approachAngle, double approachLength, double maxSpeed, boolean contact, double pickupSpeed,
                 double[] targets, boolean flowerDown, Waypoint source) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.stop = stop;
            this.turnRule = turnRule;
            this.approachAngle = approachAngle;
            this.approachLength = approachLength;
            this.maxSpeed = maxSpeed;
            this.contact = contact;
            this.pickupSpeed = pickupSpeed;
            this.targets = targets;
            this.flowerDown = flowerDown;
            this.source = source;
        }

        /** A copy that planning can change (it moves targets and makes waypoints stops) without touching this one. */
        Resolved copy() {
            return new Resolved(x, y, heading, stop, turnRule, approachAngle, approachLength, maxSpeed, contact, pickupSpeed, targets,
                    flowerDown, source);
        }
    }
}
