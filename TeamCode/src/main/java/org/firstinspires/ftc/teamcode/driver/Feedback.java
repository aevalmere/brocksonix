package org.firstinspires.ftc.teamcode.driver;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Warning;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;

import java.util.Arrays;
import java.util.Set;

/**
 * Tells the driver what the robot is doing without looking at telemetry.
 * There are only two rumbles, so each one always means the same thing:
 * - Warning: 3 strong pulses when a warning appears (see Warning). The same
 *   warning only buzzes again after a cooldown. Target out of range never buzzes:
 *   it's normal while driving, and it shows on the screen.
 * - Full: a soft, constant rumble for as long as storage is full. It stops when
 *   balls leave. A warning buzz plays over it, then the full rumble comes back.
 *
 * Gamepad light (PlayStation controllers only): red = 0-1 balls (can't shoot),
 * yellow = 2-3, green = 4.
 */
@Configurable
public class Feedback {
    // Strengths are 0 to 1. Times are in ms.
    // TODO(10): Warning buzz: strong and unmistakable, long enough to feel, short enough not to annoy. Raise the cooldown if a flickering warning still annoys.
    public static double WARNING_STRENGTH = 1.0;
    public static int WARNING_PULSE_MS = 150;
    public static int WARNING_GAP_MS = 100;
    public static int WARNING_PULSES = 3;
    /** A warning that goes away and comes back sooner than this does not buzz again. */
    public static int WARNING_COOLDOWN_MS = 3000;

    // TODO(10): Full rumble: soft enough to drive with, strong enough to notice. The gamepad light only works on PlayStation controllers.
    public static double FULL_STRENGTH = 0.25;

    private static final Warning[] ALL_WARNINGS = Warning.values();

    private final Gamepad gamepad;
    private final ElapsedTime clock = new ElapsedTime();
    private String lastLight = "";

    /** Per warning (same order as Warning): was it active last loop, and when did it last buzz. */
    private final boolean[] wasActive = new boolean[ALL_WARNINGS.length];
    private final double[] lastBuzzMs = new double[ALL_WARNINGS.length];
    /** When the warning buzz in progress ends, on the same clock. */
    private double warningBuzzEndMs = 0;
    /** The full-rumble strength sent to the gamepad, or 0 while it's off. */
    private double fullRumbleSent = 0;

    public Feedback(Gamepad gamepad) {
        this.gamepad = gamepad;
        // So the first buzz of each warning is never held back by the cooldown.
        Arrays.fill(lastBuzzMs, Double.NEGATIVE_INFINITY);
    }

    /**
     * Call once per loop.
     * @param warnings the problems active right now (Robot.warnings())
     */
    public void update(Storage storage, Set<Warning> warnings) {
        double now = clock.milliseconds();
        boolean full = storage.isFull();

        boolean warningBuzz = false;
        for (int i = 0; i < ALL_WARNINGS.length; i++) {
            Warning warning = ALL_WARNINGS[i];
            // Out of range is normal while driving around, so it stays on the screen only.
            boolean active = warning != Warning.OUT_OF_RANGE && warnings.contains(warning);
            if (active && !wasActive[i] && now - lastBuzzMs[i] >= WARNING_COOLDOWN_MS) {
                lastBuzzMs[i] = now;
                warningBuzz = true;
            }
            wasActive[i] = active;
        }

        if (warningBuzz) {
            playWarning();
            warningBuzzEndMs = now + warningMs();
            // The warning replaced the full rumble. It is sent again once the warning is over.
            fullRumbleSent = 0;
        } else if (now >= warningBuzzEndMs) {
            double wanted = full ? FULL_STRENGTH : 0;
            // Only send a change: on, off, or a new strength from Panels.
            if (wanted != fullRumbleSent) {
                if (wanted > 0) gamepad.rumble(wanted, wanted, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                else gamepad.stopRumble();
                fullRumbleSent = wanted;
            }
        }

        String light = full ? "green" : storage.hasAtLeastTwo() ? "yellow" : "red";
        if (!light.equals(lastLight)) {
            if (light.equals("green")) setLight(0, 1, 0);
            else if (light.equals("yellow")) setLight(1, 0.8, 0);
            else setLight(1, 0, 0);
            lastLight = light;
        }
    }

    /** The full rumble is continuous, so turn it off when the OpMode ends. */
    public void stop() {
        gamepad.stopRumble();
    }

    /** Built here each time, so Panels changes show up on the next warning. */
    private void playWarning() {
        if (WARNING_PULSES <= 0) return;
        Gamepad.RumbleEffect.Builder effect = new Gamepad.RumbleEffect.Builder();
        for (int i = 0; i < WARNING_PULSES; i++) {
            if (i > 0 && WARNING_GAP_MS > 0) effect.addStep(0, 0, WARNING_GAP_MS);
            effect.addStep(WARNING_STRENGTH, WARNING_STRENGTH, WARNING_PULSE_MS);
        }
        gamepad.runRumbleEffect(effect.build());
    }

    private static double warningMs() {
        if (WARNING_PULSES <= 0) return 0;
        return WARNING_PULSES * WARNING_PULSE_MS + (WARNING_PULSES - 1) * Math.max(WARNING_GAP_MS, 0);
    }

    private void setLight(double r, double g, double b) {
        gamepad.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
    }
}
