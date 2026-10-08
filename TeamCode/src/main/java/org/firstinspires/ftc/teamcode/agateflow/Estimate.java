package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

/**
 * How long a Route would take from where the robot was when it was asked,
 * worked out on the planning thread without driving it. Made by
 * AgateFlowCommands.estimate. Check ready() first; it is usually ready a few
 * loops later.
 *
 * To drive it afterwards, use AgateFlowCommands.driveTo(estimate). That skips
 * planning at the start only while the plan still fits: the robot is within
 * 1 in. and 5° of where the estimate was made from, or the estimate is under half
 * a second old and the robot is on its predicted path. It still replans while
 * driving, like any DriveRoute.
 */
public final class Estimate {
    final Route route;
    final Future<Plan> future;
    /** Field pose and alliance the plan was made from. */
    final Pose from;
    final Alliance alliance;
    private final long madeAtNanos = System.nanoTime();

    Estimate(Route route, Future<Plan> future, Pose from, Alliance alliance) {
        this.route = route;
        this.future = future;
        this.from = from;
        this.alliance = alliance;
    }

    public boolean ready() {
        return future.isDone();
    }

    /** The plan, or null if it isn't ready. Check plan().ok(): there may be no way. */
    public Plan plan() {
        if (!ready()) return null;
        try {
            return future.get();
        } catch (InterruptedException | ExecutionException e) {
            return null;
        }
    }

    /** Predicted seconds to drive the route and settle, calibrated. NaN if not ready or there is no plan. */
    public double seconds() {
        Plan plan = plan();
        return plan != null && plan.ok() ? plan.seconds() : Double.NaN;
    }

    /** Where the robot was when this was asked, field coordinates. */
    public Pose from() {
        return from;
    }

    /** Seconds since this was asked. */
    public double age() {
        return (System.nanoTime() - madeAtNanos) / 1e9;
    }
}
