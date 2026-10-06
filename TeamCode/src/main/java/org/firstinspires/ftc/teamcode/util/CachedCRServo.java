package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;

/** Only sends a power when it changes by more than a small step. */
public class CachedCRServo {
    private static final double MIN_CHANGE = 0.002;

    private final CRServo servo;
    private double last = Double.NaN;

    public CachedCRServo(CRServo servo) {
        this.servo = servo;
    }

    public void setPower(double power) {
        boolean changedSign = Math.signum(power) != Math.signum(last);
        if (Double.isNaN(last) || changedSign || Math.abs(power - last) >= MIN_CHANGE) {
            servo.setPower(power);
            last = power;
        }
    }

    public CRServo raw() {
        return servo;
    }
}
