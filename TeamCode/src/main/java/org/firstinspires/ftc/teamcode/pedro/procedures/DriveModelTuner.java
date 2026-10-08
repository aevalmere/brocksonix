package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Measures what agateflow/DriveModel needs for its time estimates: for forward,
 * strafe and turning, the top speed per battery volt and the time constant (how
 * fast it gets there), plus the battery's resistance. Run it after
 * ForesightTuner, with a charged battery and room to drive straight.
 *
 * Each run is full power from a stop. A DC motor drive speeds up as
 * acceleration = (speed per volt x volts - speed) / time constant, so a straight
 * line fit of acceleration against (volts, speed) gives both numbers. The volts
 * are measured during the run, so battery sag doesn't skew them.
 */
public class DriveModelTuner extends Procedure {
    private final Function<HardwareMap, Localizer> localizerFunction;
    private final Function<HardwareMap, Drivetrain> drivetrainFunction;

    public DriveModelTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Drive Model Tuner", "Measures top speeds, time constants and battery resistance for AgateFlow's time estimates.");
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    public void run() throws InterruptedException {
        Inputs distanceInput = inputs("Distance", "How far the robot may drive forward, and then left, in inches. More is more accurate; 60 is plenty.");
        Inputs.Field<Double> distance = distanceInput.d("Distance").withDefault(60.0);
        awaitInputs(distanceInput);

        Run forward = runOpMode(new FullPower(localizerFunction, drivetrainFunction, Axis.FORWARD, distance.get()));
        Run strafe = runOpMode(new FullPower(localizerFunction, drivetrainFunction, Axis.STRAFE, distance.get()));
        Run turn = runOpMode(new FullPower(localizerFunction, drivetrainFunction, Axis.TURN, 0));

        double[] f = forward.fit(), s = strafe.fit(), t = turn.fit();
        if (Double.isNaN(f[0]) || Double.isNaN(s[0]) || Double.isNaN(t[0])) {
            abort("Not enough data to fit. Check the localizer works (Tests procedure) and give it more distance.");
            return;
        }
        double ohms = forward.batteryOhms();

        result("forward speed per volt (in/s/V)", f[0]);
        result("forward time constant (s)", f[1]);
        result("strafe speed per volt (in/s/V)", s[0]);
        result("strafe time constant (s)", s[1]);
        result("turn speed per volt (rad/s/V)", t[0]);
        result("turn time constant (s)", t[1]);
        result("battery resistance (ohms)", Double.isNaN(ohms) ? "not measured (drivetrain isn't Mecanum)" : String.valueOf(ohms));

        code(Language.JAVA,
                "// Paste over the matching fields in agateflow/DriveModel.java\n" +
                "public static double FORWARD_SPEED_PER_VOLT = " + f[0] + ";\n" +
                "public static double STRAFE_SPEED_PER_VOLT = " + s[0] + ";\n" +
                "public static double TURN_SPEED_PER_VOLT = " + t[0] + ";\n" +
                "public static double FORWARD_TIME_CONSTANT = " + f[1] + ";\n" +
                "public static double STRAFE_TIME_CONSTANT = " + s[1] + ";\n" +
                "public static double TURN_TIME_CONSTANT = " + t[1] + ";\n" +
                (Double.isNaN(ohms) ? "" : "public static double BATTERY_OHMS = " + ohms + ";\n"));
    }

    enum Axis { FORWARD, STRAFE, TURN }

    /** What one full-power run recorded. */
    static final class Run {
        final List<double[]> samples = new ArrayList<>(); // {seconds, speed, volts, amps (NaN if not read)}

        /**
         * Least squares fit of acceleration = b1 x volts + b2 x speed (no constant),
         * so time constant = -1 / b2 and speed per volt = b1 x time constant.
         * Returns {speed per volt, time constant}, or NaNs if the data doesn't fit.
         */
        double[] fit() {
            double vv = 0, vs = 0, ss = 0, av = 0, as = 0;
            int used = 0;
            for (int i = 2; i < samples.size() - 2; i++) {
                double[] before = samples.get(i - 2), here = samples.get(i), after = samples.get(i + 2);
                if (here[0] < 0.05) continue; // the first loops carry startup lag
                double accel = (after[1] - before[1]) / (after[0] - before[0]);
                double volts = here[2], speed = here[1];
                vv += volts * volts;
                vs += volts * speed;
                ss += speed * speed;
                av += accel * volts;
                as += accel * speed;
                used++;
            }
            double det = vv * ss - vs * vs;
            if (used < 10 || Math.abs(det) < 1e-9) return new double[]{Double.NaN, Double.NaN};
            double b1 = (av * ss - as * vs) / det, b2 = (as * vv - av * vs) / det;
            if (b2 >= 0) return new double[]{Double.NaN, Double.NaN};
            double tau = -1 / b2;
            return new double[]{b1 * tau, tau};
        }

        /** Fit of volts = open-circuit volts - resistance x amps. NaN if no currents were read. */
        double batteryOhms() {
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            int n = 0;
            for (double[] sample : samples) {
                if (Double.isNaN(sample[3])) continue;
                sx += sample[3];
                sy += sample[2];
                sxx += sample[3] * sample[3];
                sxy += sample[3] * sample[2];
                n++;
            }
            double det = n * sxx - sx * sx;
            if (n < 5 || Math.abs(det) < 1e-9) return Double.NaN;
            return -(n * sxy - sx * sy) / det;
        }
    }
}

/** Full power along one axis from a stop, recording speed, volts and (on Mecanum) current. */
class FullPower extends TuningOpMode<DriveModelTuner.Run> {
    private static final double MAX_SECONDS = 2.0;
    private final Function<HardwareMap, Localizer> localizerFunction;
    private final Function<HardwareMap, Drivetrain> drivetrainFunction;
    private final DriveModelTuner.Axis axis;
    private final double distance;

    FullPower(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction,
              DriveModelTuner.Axis axis, double distance) {
        super("Full Power " + axis, axis == DriveModelTuner.Axis.TURN
                ? "The robot will spin counter-clockwise at full power for 2 seconds."
                : "The robot will drive " + (axis == DriveModelTuner.Axis.FORWARD ? "forward" : "left") + " at full power for up to "
                + distance + " inches, then stop.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.axis = axis;
        this.distance = distance;
    }

    @Override
    protected DriveModelTuner.Run runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        DriveModelTuner.Run run = new DriveModelTuner.Run();
        DrivePowers power = axis == DriveModelTuner.Axis.FORWARD ? new DrivePowers(1, 0, 0)
                : axis == DriveModelTuner.Axis.STRAFE ? new DrivePowers(0, 1, 0) : new DrivePowers(0, 0, 1);

        localizer.setPose(Pose.zero());
        localizer.update();
        Thread.sleep(500);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        ElapsedTime timer = new ElapsedTime();
        int loops = 0;
        while (!isStopRequested() && timer.seconds() < MAX_SECONDS) {
            localizer.update();
            Pose pose = localizer.pose();
            if (axis == DriveModelTuner.Axis.FORWARD && Math.abs(pose.x()) > distance) break;
            if (axis == DriveModelTuner.Axis.STRAFE && Math.abs(pose.y()) > distance) break;
            drivetrain.drive(power, false);
            double speed = axis == DriveModelTuner.Axis.FORWARD ? localizer.twist().vx
                    : axis == DriveModelTuner.Axis.STRAFE ? localizer.twist().vy : localizer.twist().omega;
            // Reading current is slow (one hub call per motor), so only every few loops.
            double amps = drivetrain instanceof Mecanum && loops % 4 == 0 ? ((Mecanum) drivetrain).currentAmps() : Double.NaN;
            run.samples.add(new double[]{timer.seconds(), speed, battery.getVoltage(), amps});
            loops++;
        }
        drivetrain.stop();
        Thread.sleep(1000);
        return run;
    }
}
