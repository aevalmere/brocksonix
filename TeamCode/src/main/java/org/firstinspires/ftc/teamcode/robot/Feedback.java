package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Storage;

/**
 * Tells the driver what the robot is doing without looking at telemetry.
 *
 * Rumble once when storage fills, and once when a burst empties it (time to drive off).
 * Gamepad light (PlayStation controllers only): red = 0-1 balls (can't shoot),
 * yellow = 2-3, green = 4.
 */
@Configurable
public class Feedback {
    // TODO(10): Long enough to feel, short enough not to annoy. The light only works on PlayStation controllers.
    public static int FULL_RUMBLE_MS = 300;
    public static int EMPTY_RUMBLE_MS = 250;

    private final Gamepad gamepad;
    private boolean wasFull = false;
    private boolean wasEmpty = true;
    private String lastLight = "";

    public Feedback(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update(Storage storage, boolean firing) {
        boolean full = storage.isFull();
        boolean empty = storage.isEmpty();

        if (full && !wasFull) gamepad.rumble(FULL_RUMBLE_MS);
        if (firing && empty && !wasEmpty) gamepad.rumble(EMPTY_RUMBLE_MS);
        wasFull = full;
        wasEmpty = empty;

        String light = full ? "green" : storage.hasAtLeastTwo() ? "yellow" : "red";
        if (!light.equals(lastLight)) {
            if (light.equals("green")) setLight(0, 1, 0);
            else if (light.equals("yellow")) setLight(1, 0.8, 0);
            else setLight(1, 0, 0);
            lastLight = light;
        }
    }

    private void setLight(double r, double g, double b) {
        gamepad.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
    }
}
