package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.robotcore.util.SerialNumber;

import org.horizon36596.simloop.fakehardware.FakeDevice;

import java.lang.reflect.Proxy;

/**
 * A hub for BulkReads to find with hardwareMap.getAll(LynxModule.class). SimLoop has
 * no hub fake. This is the SDK's own LynxModule with no USB link behind it, which is
 * enough for the two calls BulkReads makes: setBulkCachingMode and clearBulkCache only
 * change the module's own cache. Anything that sends a command to the hub would fail.
 */
final class FakeLynxModule extends LynxModule implements FakeDevice {
    /** SPEC: the Control Hub's own module address, as the SDK numbers it. */
    private static final int CONTROL_HUB_ADDRESS = 173;

    FakeLynxModule() {
        super(noUsbLink(), CONTROL_HUB_ADDRESS, true, true);
    }

    /** The constructor only asks the USB device for its serial number and registers for arming notices. */
    private static LynxUsbDevice noUsbLink() {
        return (LynxUsbDevice) Proxy.newProxyInstance(LynxUsbDevice.class.getClassLoader(),
                new Class<?>[] {LynxUsbDevice.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getSerialNumber": return SerialNumber.createEmbedded();
                        case "registerCallback": return null; // arming notices: there is no USB to arm
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == args[0];
                        case "toString": return "the sim's hub (no USB link)";
                    }
                    throw new UnsupportedOperationException("the sim's hub has no USB link: " + method.getName());
                });
    }

    @Override
    public void update(double deltaTime) {
    }
}
