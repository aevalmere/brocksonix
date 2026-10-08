package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.ivy.Scheduler;

import java.util.function.BooleanSupplier;

/**
 * One-button driving for TeleOp: press a button and AgateFlow drives a route
 * (to the park spot, a shooting spot), around everything in its FieldMap. The
 * moment the driver touches the sticks, control goes straight back to them.
 * Not used by any OpMode yet.
 *
 * <pre>
 * DriverAssist park = new DriverAssist(agateFlow, new Route().to(PARK_SPOT),
 *         () -> Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x) > 0.1);
 * while (opModeIsActive()) {
 *     park.update(gamepad1.triangle);   // before Scheduler.execute()
 *     Scheduler.execute();
 *     if (!park.driving()) drive with the sticks as usual;
 *     telemetry.addData("Assist", park.describe());
 * }
 * </pre>
 *
 * A second press while it drives starts it again from where the robot is. If
 * AgateFlow finds no way (or the drive times out), the robot holds still and
 * describe() says why; rumble on failed() if the driver should know.
 */
public final class DriverAssist {
    private final AgateFlowCommands agateFlow;
    private final Route route;
    private final BooleanSupplier driverDriving;
    private DriveRoute drive;
    private boolean wasPressed, handedBack;

    /**
     * @param driverDriving true while the driver is using the sticks (with a small
     *                      dead band), which cancels the drive at once
     */
    public DriverAssist(AgateFlowCommands agateFlow, Route route, BooleanSupplier driverDriving) {
        this.agateFlow = agateFlow;
        this.route = route;
        this.driverDriving = driverDriving;
    }

    /** Call every loop with the button, before Scheduler.execute(). */
    public void update(boolean pressed) {
        if (pressed && !wasPressed && !driverDriving.getAsBoolean()) {
            if (driving()) Scheduler.cancel(drive);
            drive = agateFlow.driveTo(route);
            handedBack = false;
            Scheduler.schedule(drive);
        }
        wasPressed = pressed;
        if (driving() && driverDriving.getAsBoolean()) {
            Scheduler.cancel(drive);
            handedBack = true;
        }
    }

    /** True while AgateFlow is driving (or planning). Then the OpMode must not drive with the sticks. */
    public boolean driving() {
        return drive != null && Scheduler.isScheduled(drive);
    }

    /** True if the last drive ended without getting there (no way, or out of time), not because the driver took over. */
    public boolean failed() {
        if (drive == null || driving() || handedBack) return false;
        return drive.status() == DriveRoute.Status.NO_PLAN || drive.status() == DriveRoute.Status.TIMED_OUT;
    }

    /** One line for telemetry. */
    public String describe() {
        if (drive == null) return "Ready";
        if (driving()) return drive.describe();
        if (handedBack) return "Driver took over";
        return drive.status() + (drive.problem() == null ? "" : ": " + drive.problem());
    }
}
