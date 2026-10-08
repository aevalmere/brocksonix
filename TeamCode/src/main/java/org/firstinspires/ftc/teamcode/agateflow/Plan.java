package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Pose;

import java.util.Collections;
import java.util.List;

/**
 * What AgateFlow made from a Route: legs to follow one after another, or a
 * reason it couldn't. Check ok() first.
 */
public final class Plan {
    /** Null if the plan is good, otherwise why there is no plan. */
    public final String problem;
    public final List<Leg> legs;
    /** Things AgateFlow changed or noticed, like a target moved out of a zone. */
    public final List<String> notes;
    /** How long planning took, milliseconds. */
    public final double planMillis;
    /** FieldMap.Snapshot version the plan was made with. */
    public final long zonesVersion;
    /** The zones the plan was made with. DriveRoute compares new zones to these. */
    final FieldMap.Snapshot zones;
    private final List<Pose> pickedUp, skipped;

    Plan(String problem, List<Leg> legs, List<String> notes, double planMillis, FieldMap.Snapshot zones,
         List<Pose> pickedUp, List<Pose> skipped) {
        this.problem = problem;
        this.legs = Collections.unmodifiableList(legs);
        this.notes = Collections.unmodifiableList(notes);
        this.planMillis = planMillis;
        this.zones = zones;
        this.zonesVersion = zones.version;
        this.pickedUp = Collections.unmodifiableList(pickedUp);
        this.skipped = Collections.unmodifiableList(skipped);
    }

    /**
     * Route.pickUp targets the predicted path drives into the intake, in the order
     * it gets them, field coordinates. Worked out when the route is planned at the
     * start (a replan's Plan has none).
     */
    public List<Pose> pickedUp() {
        return pickedUp;
    }

    /** Route.pickUp targets it doesn't get (not chosen, past most(), or missed). */
    public List<Pose> skipped() {
        return skipped;
    }

    public boolean ok() {
        return problem == null;
    }

    /** Predicted seconds for the whole route, straight from the model. */
    public double rawSeconds() {
        double total = 0;
        for (Leg leg : legs) total += leg.seconds;
        return total;
    }

    /** Predicted seconds for the whole route, corrected by what EtaCalibration has learned on the robot. */
    public double seconds() {
        return EtaCalibration.correct(rawSeconds());
    }

    public double length() {
        double total = 0;
        for (Leg leg : legs) total += leg.length;
        return total;
    }

    @Override
    public String toString() {
        if (!ok()) return "No plan: " + problem;
        return String.format("%d legs, %.0f in, %.2f s (%.1f ms to plan)", legs.size(), length(), seconds(), planMillis);
    }
}
