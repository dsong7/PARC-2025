package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Flywheel outtake/launcher mechanism with PIDF velocity control on two motors.
 *
 * Usage:
 *   Outtake outtake = new Outtake(hardwareMap, "Launcher1", "Launcher2");
 *   outtake.setTargetRPM(1800);
 *   outtake.setHolding(true);
 *   loop: outtake.update();
 */
public class Outtake {

    // Encoder / gearing constants
    private static final double TICKS_PER_MOTOR_REV   = 28.0;
    private static final double GEAR_RATIO            = 1.0;
    private static final double TICKS_PER_OUTPUT_REV  = TICKS_PER_MOTOR_REV * GEAR_RATIO;


    // Assumed max RPM of the motor at the encoder shaft
    private static final double ASSUMED_MAX_RPM = 6000.0;

    private ElapsedTime timer = new ElapsedTime();

    // Hardware
    private DcMotorEx flywheel1;

    // State
    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    Gamepad gamepad2;
    private double chargeTimer = 0.0;
    private static final double MAX_RPM = 4000.0;
    private static final double CHARGE_INTERVAL_S = 0.10; // 0.1 sec
    private static final double CHARGE_STEP_RPM = 200.0;

    public Outtake(DcMotorEx flywheel1, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        this.flywheel1 = flywheel1;
        this.flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update() {
        double dt = timer.seconds();
        timer.reset();

        boolean charging = (-gamepad2.left_stick_y) > 0.7;  // threshold; tweak 0.3-0.8

        if (charging) {
            chargeTimer += dt;

            // Add +100 rpm once per 0.1s (works even if dt is large)
            while (chargeTimer >= CHARGE_INTERVAL_S) {
                chargeTimer -= CHARGE_INTERVAL_S;
                targetRPM = Math.min(MAX_RPM, targetRPM + CHARGE_STEP_RPM);
            }
        } else {
            // release behavior:
            targetRPM = 0.0;      // <-- change to "keep targetRPM" if you want it to hold
            chargeTimer = 0.0;
        }

        // Smoothly move currentRPM toward targetRPM (optional but recommended)
        double error = targetRPM - currentRPM;
        double maxStep = 500.0 * dt; // rpm per second ramp rate (500 here)

        if (Math.abs(error) > maxStep) {
            currentRPM += Math.signum(error) * maxStep;
        } else {
            currentRPM = targetRPM;
        }

        flywheel1.setVelocity(rpmToTicksPerSec(currentRPM));
    }
    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_OUTPUT_REV / 60.0;
    }

}
