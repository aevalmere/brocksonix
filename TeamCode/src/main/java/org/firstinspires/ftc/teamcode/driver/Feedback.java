package org.firstinspires.ftc.teamcode.driver;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Warning;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;

import java.util.Arrays;
import java.util.Set;

/**
 * Tells the drivers what the robot is doing without looking at telemetry.
 * There are three rumbles, and each one always means the same thing:
 * - Warning (driver 1): 3 strong pulses when a warning appears (see Warning).
 *   The same warning only buzzes again after a cooldown. Target out of range
 *   never buzzes: it's normal while driving, and it shows on the screen.
 * - Relocalized (both drivers): one short, strong pulse when a relocalize
 *   happens (relocalized()). Driver 2 knows the press counted, and driver 1
 *   knows the robot can leave the corner. On driver 1 it waits for a warning
 *   buzz that is already playing to finish.
 * - Full (driver 1): a soft, constant rumble for as long as storage is full. It
 *   stops when balls leave. A warning or relocalized buzz plays over it, then the
 *   full rumble comes back.
 *
 * Gamepad light (driver 1, PlayStation controllers only): red = 0-1 balls
 * (can't shoot), yellow = 2-3, green = 4.
 */
@Configurable
public class Feedback {
    // Strengths are 0 to 1. Times are in ms.
    // TODO(12): Warning buzz: strong and unmistakable, long enough to feel, short enough not to annoy. Raise the cooldown if a flickering warning still annoys.
    public static double WARNING_STRENGTH = 1.0;
    public static int WARNING_PULSE_MS = 150;
    public static int WARNING_GAP_MS = 100;
    public static int WARNING_PULSES = 3;
    /** A warning that goes away and comes back sooner than this does not buzz again. */
    public static int WARNING_COOLDOWN_MS = 3000;

    // TODO(12): Relocalized buzz: one short, strong pulse that can't be mistaken for the 3-pulse warning. Long enough to feel while driving.
    public static double RELOCALIZE_STRENGTH = 1.0;
    public static int RELOCALIZE_MS = 200;

    // TODO(12): Full rumble: soft enough to drive with, strong enough to notice. The gamepad light only works on PlayStation controllers.
    public static double FULL_STRENGTH = 0.25;

    private static final Warning[] ALL_WARNINGS = Warning.values();

    /** Driver 1 gets every rumble and the light. Driver 2 only gets the relocalized buzz. */
    private final Gamepad driver1, driver2;
    private final ElapsedTime clock = new ElapsedTime();
    private String lastLight = "";

    /** Per warning (same order as Warning): was it active last loop, and when did it last buzz. */
    private final boolean[] wasActive = new boolean[ALL_WARNINGS.length];
    private final double[] lastBuzzMs = new double[ALL_WARNINGS.length];
    /** When the warning or relocalized buzz in progress on driver 1 ends, on the same clock. */
    private double buzzEndMs = 0;
    /** A relocalize happened, and driver 1 hasn't felt it yet. */
    private boolean relocalizePending = false;
    /** The full-rumble strength sent to the gamepad, or 0 while it's off. */
    private double fullRumbleSent = 0;

    public Feedback(Gamepad driver1, Gamepad driver2) {
        this.driver1 = driver1;
        this.driver2 = driver2;
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
            buzzEndMs = now + warningMs();
            // The warning replaced the full rumble. It is sent again once the warning is over.
            fullRumbleSent = 0;
        } else if (relocalizePending && now >= buzzEndMs) {
            relocalizePending = false;
            if (RELOCALIZE_MS > 0) {
                driver1.rumble(RELOCALIZE_STRENGTH, RELOCALIZE_STRENGTH, RELOCALIZE_MS);
                buzzEndMs = now + RELOCALIZE_MS;
                // Like a warning, it replaced the full rumble until it is over.
                fullRumbleSent = 0;
            }
        } else if (now >= buzzEndMs) {
            double wanted = full ? FULL_STRENGTH : 0;
            // Only send a change: on, off, or a new strength from Panels.
            if (wanted != fullRumbleSent) {
                if (wanted > 0) driver1.rumble(wanted, wanted, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                else driver1.stopRumble();
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

    /**
     * Call when a relocalize happened. Driver 2 feels it right away, driver 1 on the next
     * update() (after a warning buzz that is already playing).
     */
    public void relocalized() {
        if (RELOCALIZE_MS > 0) driver2.rumble(RELOCALIZE_STRENGTH, RELOCALIZE_STRENGTH, RELOCALIZE_MS);
        relocalizePending = true;
    }

    /** The full rumble is continuous, so turn it off when the OpMode ends. */
    public void stop() {
        driver1.stopRumble();
        driver2.stopRumble();
    }

    /** Built here each time, so Panels changes show up on the next warning. */
    private void playWarning() {
        if (WARNING_PULSES <= 0) return;
        Gamepad.RumbleEffect.Builder effect = new Gamepad.RumbleEffect.Builder();
        for (int i = 0; i < WARNING_PULSES; i++) {
            if (i > 0 && WARNING_GAP_MS > 0) effect.addStep(0, 0, WARNING_GAP_MS);
            effect.addStep(WARNING_STRENGTH, WARNING_STRENGTH, WARNING_PULSE_MS);
        }
        driver1.runRumbleEffect(effect.build());
    }

    private static double warningMs() {
        if (WARNING_PULSES <= 0) return 0;
        return WARNING_PULSES * WARNING_PULSE_MS + (WARNING_PULSES - 1) * Math.max(WARNING_GAP_MS, 0);
    }

    private void setLight(double r, double g, double b) {
        driver1.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
    }
}
