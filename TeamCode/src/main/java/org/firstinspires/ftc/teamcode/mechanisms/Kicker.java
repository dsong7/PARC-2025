package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {

    private static final double ZERO_POS = 0.7156;

    // Set this to your servo's travel over pos 0..1 (often 180 or 270).
    private static final double DEG_PER_POS = 180.0;

    private static final double TARGET_DEG = 90.0;

    private final Servo servoKicker;
    private final Gamepad gamepad2;

    private boolean lastA = false;
    private boolean lastB = false;

    // Track angle state (0 or 90)
    private double currentDeg = 0.0;

    public Kicker(Servo servoKicker, Gamepad gamepad2) {
        this.servoKicker = servoKicker;
        this.gamepad2 = gamepad2;

        // Initialize at zero
        setAngleDeg(0.0);
    }

    /** Call this once per loop. A -> 90 deg, B -> 0 deg. */
    public void update() {
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;

        // Rising-edge detect
        if (a && !lastA) {
            setAngleDeg(TARGET_DEG);
        }
        if (b && !lastB) {
            setAngleDeg(0.0);
        }

        lastA = a;
        lastB = b;
    }

    /** Optional: direct command methods (no gamepad). */
    public void kick() {
        setAngleDeg(TARGET_DEG);
    }

    public void zero() {
        setAngleDeg(0.0);
    }

    /** Convert degrees to servo position (positive degrees => more positive position). */
    private void setAngleDeg(double deg) {
        currentDeg = deg;

        double targetPos = ZERO_POS + (currentDeg / DEG_PER_POS);
        targetPos = clamp01(targetPos);

        servoKicker.setPosition(targetPos);
    }

    private static double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
    }

    public double getCurrentDeg() {
        return currentDeg;
    }
}