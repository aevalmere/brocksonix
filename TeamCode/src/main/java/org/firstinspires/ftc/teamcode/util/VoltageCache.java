package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Battery voltage, re-read only every few hundred ms because each read is a slow hub call. */
public class VoltageCache {
    private static final double REFRESH_MS = 250;

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
        return 12.0 / Math.max(get(), 8.0);
    }
}
