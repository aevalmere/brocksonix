package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Only sends a power when it changes by more than a small step (0.01 unless you pass your own). */
public class CachedMotor {
    private static final double DEFAULT_MIN_CHANGE = 0.01;

    private final DcMotorEx motor;
    private final double minChange;
    private double last = Double.NaN;

    public CachedMotor(DcMotorEx motor) {
        this(motor, DEFAULT_MIN_CHANGE);
    }

    /** Use a smaller step when small power changes matter, like the shooter's feedforward. */
    public CachedMotor(DcMotorEx motor, double minChange) {
        this.motor = motor;
        this.minChange = minChange;
    }

    public void setPower(double power) {
        boolean changedSign = Math.signum(power) != Math.signum(last);
        if (Double.isNaN(last) || changedSign || Math.abs(power - last) >= minChange) {
            motor.setPower(power);
            last = power;
        }
    }

    public DcMotorEx raw() {
        return motor;
    }
}
