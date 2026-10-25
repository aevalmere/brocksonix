package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.CachedMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;

/**
 * The intake motor. It drives the intake end of the rail (slots 3-4); Transfer
 * drives the door end. Positive power moves balls toward the door.
 *
 * No update(): each method writes the motor right away.
 */
@Configurable
public class Intake {
    /** Checked in Intake Test: with this on, balls move in, toward the door. */
    public static boolean REVERSED = true;

    // TODO(3): Lower it if balls jam or bounce.
    public static double COLLECT_POWER = 1.0;
    // TODO(3): Storage Test, hold Circle with balls loaded: they must back out without flying out of the intake. Lower it (toward 0) if they do.
    /** Unjam power, negative: balls move away from the door. */
    public static double REVERSE_POWER = -1.0;

    private final CachedMotor motor;
    private String state = "stopped";

    public Intake(HardwareMap hardwareMap) {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor = new CachedMotor(intakeMotor);
    }

    /** The motor stays FORWARD. REVERSED is applied here, so flipping it in Panels works live. */
    private void setPower(double power) {
        motor.setPower(REVERSED ? -power : power);
    }

    /** Normal collecting: pull balls in until storage is full. */
    public void collect(boolean full) {
        setPower(full ? 0 : COLLECT_POWER);
        state = full ? "full" : "collecting";
    }

    /** Push balls through the open door into the shooter. */
    public void feed(double power) {
        setPower(power);
        state = "feeding";
    }

    public void reverse() {
        setPower(REVERSE_POWER);
        state = "reversing";
    }

    public void stop() {
        setPower(0);
        state = "stopped";
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", state);
    }
}
