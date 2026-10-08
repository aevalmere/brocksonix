package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Reads every sensor on a hub in one call per loop instead of one call per
 * sensor. Call clear() once at the start of each loop, or values go stale.
 */
public class BulkReads {
    private final List<LynxModule> hubs;

    public BulkReads(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void clear() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
