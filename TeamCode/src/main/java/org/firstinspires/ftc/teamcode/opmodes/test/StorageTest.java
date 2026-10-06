package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.util.BulkReads;

/**
 * Checks the break beams and the ball count. Block each beam by hand first,
 * then run the rail and feed real balls in.
 *
 * Cross: rail collect on/off. Circle (hold): reverse.
 */
@TeleOp(name = "Storage Test", group = "Test")
public class StorageTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        Storage storage = new Storage(hardwareMap);
        Rail rail = new Rail(hardwareMap);
        boolean collecting = false;

        waitForStart();
        while (opModeIsActive()) {
            bulkReads.clear();
            storage.update();

            if (gamepad1.crossWasPressed()) collecting = !collecting;
            if (gamepad1.circle) rail.reverse();
            else if (collecting) rail.collect(storage.slot1Occupied(), storage.isFull());
            else rail.stop();

            screen.addData("Collecting (Cross)", collecting);
            storage.addTelemetry(screen);
            rail.addTelemetry(screen);
            screen.update();
        }
    }
}
