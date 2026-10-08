package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Battery voltage, re-read only every few hundred ms because each read is a slow hub call. */
public class VoltageCache {
    private static final double REFRESH_MS = 250;
    // The reference battery voltage: compensation() is 1.0 at this level.
    private static final double NOMINAL_VOLTS = 12.0;
    // Lower readings count as this, so a sag or a 0 V reading can't push the boost past 1.5x (12 / 8).
    private static final double MIN_VOLTS = 8.0;

    private final VoltageSensor sensor;
    private final ElapsedTime sinceRead = new ElapsedTime();
    private double volts;

    public VoltageCache(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        volts = sensor.getVoltage();
    }

    public double get() {
        if (sinceRead.milliseconds() >= REFRESH_MS) {
            volts = sensor.getVoltage();
            sinceRead.reset();
        }
        return volts;
    }

    /** Multiply a motor power by this so it pushes the same at any battery level. */
    public double compensation() {
        return NOMINAL_VOLTS / Math.max(get(), MIN_VOLTS);
    }
}
