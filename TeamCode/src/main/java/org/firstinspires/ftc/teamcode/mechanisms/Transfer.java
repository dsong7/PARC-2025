package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Transfer: rotating chamber driven by a positional servo.
 *
 * Controls (gamepad2):
 *  - dpad_right: rotate +120 degrees
 *  - dpad_left : rotate -120 degrees
 *
 * Safety:
 *  - Will NOT rotate if the kicker is "up".
 *
 * Assumptions:
 *  - "More positive servo position" = rotate right (positive degrees).
 *  - Your chamber servo has ~DEG_PER_POS degrees of travel across [0..1] position.
 */
public class Transfer {

    // --- Chamber calibration ---
    private static final double CHAMBER_ZERO_POS = 0.50;  // <-- set your chamber's "0 deg" servo position
    private static final double DEG_PER_POS      = 360.0; // <-- 180 / 270 / 360 depending on servo
    private static final double STEP_DEG         = 120.0;

    // Optional clamp in degrees (prevents commanding outside usable range).
    // Set to your safe physical range if you know it.
    private static final double MIN_DEG = 0.0;
    private static final double MAX_DEG = 360.0;

    // --- Kicker gating ---
    // Reminder: your Kicker class exposes getCurrentDeg() (0 or 90 in your design)
    private static final double KICKER_UP_THRESHOLD_DEG = 45.0;

    private final Servo chamberServo;
    private final Gamepad gamepad2;
    private final Kicker kicker;

    private boolean lastRight = false;
    private boolean lastLeft  = false;

    // Current chamber angle relative to "zero"
    private double chamberDeg = 0.0;

    public Transfer(Servo chamberServo, Gamepad gamepad2, Kicker kicker) {
        this.chamberServo = chamberServo;
        this.gamepad2 = gamepad2;
        this.kicker = kicker;

        // Initialize to 0 deg
        applyAngleDeg(0.0);
    }

    /** Call once per loop. */
    public void update() {
        boolean right = gamepad2.right_bumper;
        boolean left  = gamepad2.left_bumper;

        // Don't turn while kicker is up
        if (isKickerUp()) {
            lastRight = right;
            lastLeft  = left;
            return;
        }

        // Rising-edge detect
        if (right && !lastRight) {
            applyAngleDeg(chamberDeg + STEP_DEG);
        }
        if (left && !lastLeft) {
            applyAngleDeg(chamberDeg - STEP_DEG);
        }

        lastRight = right;
        lastLeft  = left;
    }


    /** True if kicker is in the "up" position. */
    private boolean isKickerUp() {
        // Uses your Kicker state variable (0 vs 90). Adjust threshold if needed.
        return kicker != null && kicker.getCurrentDeg() >= KICKER_UP_THRESHOLD_DEG;
    }

    /** Applies angle, clamps, maps to servo pos, and commands servo. */
    private void applyAngleDeg(double deg) {
        chamberDeg = clamp(deg, MIN_DEG, MAX_DEG);

        // Map degrees -> servo position (positive degrees => more positive position)
        double targetPos = CHAMBER_ZERO_POS + (chamberDeg / DEG_PER_POS);
        targetPos = clamp(targetPos, 0.0, 1.0);

        chamberServo.setPosition(targetPos);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}