package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.test.AgateFlowBenchmark;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * The AgateFlow Benchmark OpMode's routes all plan, for both alliances, on the
 * simulated robot. A route that found no way would only time a failure on the
 * Control Hub.
 */
public class AgateFlowBenchmarkTest {
    @Test
    public void everyRoutePlans() {
        double timeLimit = AgateFlow.PLAN_TIME_LIMIT_MS;
        AgateFlow.PLAN_TIME_LIMIT_MS = 1e9;
        try {
            TestRobot robot = new TestRobot();
            AgateFlow agateFlow = robot.agateFlow();
            List<Pose> starts = new ArrayList<>();
            List<Route> routes = new ArrayList<>();
            AgateFlowBenchmark.routes(starts, routes);
            List<String> failed = new ArrayList<>();
            for (Alliance alliance : Alliance.values()) {
                for (int i = 0; i < routes.size(); i++) {
                    Plan plan = robot.plan(agateFlow, alliance.fromRed(starts.get(i)), routes.get(i), alliance);
                    if (!plan.ok()) failed.add(alliance + " route " + (i + 1) + ": " + plan);
                }
            }
            assertTrue(failed.toString(), failed.isEmpty());
        } finally {
            AgateFlow.PLAN_TIME_LIMIT_MS = timeLimit;
        }
    }
}
