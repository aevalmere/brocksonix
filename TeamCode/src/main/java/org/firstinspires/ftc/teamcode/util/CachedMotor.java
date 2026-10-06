package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Only sends a power when it changes by more than a small step. */
public class CachedMotor {
    private static final double MIN_CHANGE = 0.01;

    private final DcMotorEx motor;
    private double last = Double.NaN;

    public CachedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        boolean changedSign = Math.signum(power) != Math.signum(last);
        if (Double.isNaN(last) || changedSign || Math.abs(power - last) >= MIN_CHANGE) {
            motor.setPower(power);
            last = power;
        }
    }

    public DcMotorEx raw() {
        return motor;
    }
}
