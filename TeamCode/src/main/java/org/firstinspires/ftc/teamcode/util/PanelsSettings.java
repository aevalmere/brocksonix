package org.firstinspires.ftc.teamcode.util;

import com.bylazar.limelightproxy.LimelightProxyConfig;
import com.bylazar.panels.PanelsConfig;

/**
 * Keeps Panels off at events. The Competition Manual names FTControl Panels as a
 * prohibited tool (R704.D), but Panels starts with the robot app. Its Driver Station
 * switch (the "Enable/Disable Panels" OpMode) only lasts until the app restarts, and
 * every restart turns Panels back on. With EVENT_BUILD true, it stays off at every start.
 *
 * Nothing in our code calls this class. When the app starts, Panels looks through every
 * class in the app for one that extends PanelsConfig and uses the first it finds, so keep
 * this the only one.
 *
 * EVENT_BUILD is a build switch, not a Panels tunable: Panels reads it once when the app
 * starts. Change it in the code and install the app again. Leave its TODO in, since it
 * comes up before every event, and set it back to false afterward to use Panels again.
 */
public class PanelsSettings extends PanelsConfig {
    // TODO(10): set true for event builds so Panels never starts (R704.D).
    public static final boolean EVENT_BUILD = false;

    @Override
    public boolean isDisabled() {
        return EVENT_BUILD;
    }

    /**
     * Panels' Limelight plugin opens ports 5800, 5801, 5805 and 5807 at every app start,
     * even while Panels is off, so an event build turns it off too. Panels finds this
     * class the same way (it extends the plugin's config class).
     */
    public static class LimelightProxySettings extends LimelightProxyConfig {
        @Override
        public boolean isEnabled() {
            return !EVENT_BUILD;
        }
    }
}
