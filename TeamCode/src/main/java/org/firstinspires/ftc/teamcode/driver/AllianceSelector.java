package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotState;

/**
 * Alliance picking for the init of every match OpMode, teleop and auto.
 * D-pad up on either gamepad switches red/blue, any time during init. Both
 * gamepads light up in the alliance color (PlayStation controllers), or
 * white when nothing is picked.
 *
 * Right after auto the alliance is kept, so teleop starts already picked.
 * Otherwise it starts unpicked and nothing fires until someone picks it.
 * Switching alliance drops the pose saved by auto, since it is in the other
 * alliance's frame: relocalize in a corner instead.
 */
public class AllianceSelector {
    private final Gamepad gamepad1, gamepad2;
    private Alliance shown = null;

    public AllianceSelector(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        RobotState.forgetIfStale();
        light(gamepad1);
        light(gamepad2);
        shown = RobotState.alliance;
    }

    /** Call once per init loop. */
    public void update(Telemetry telemetry) {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            RobotState.setAlliance(RobotState.alliance == null ? Alliance.RED : RobotState.alliance.other());
        }
        if (RobotState.alliance != shown) {
            light(gamepad1);
            light(gamepad2);
            shown = RobotState.alliance;
        }
        telemetry.addData("Alliance (D-pad up to switch)", RobotState.alliance == null ? "NOT PICKED" : RobotState.alliance);
    }

    private static void light(Gamepad pad) {
        if (RobotState.alliance == Alliance.RED) pad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        else if (RobotState.alliance == Alliance.BLUE) pad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        else pad.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
    }
}
