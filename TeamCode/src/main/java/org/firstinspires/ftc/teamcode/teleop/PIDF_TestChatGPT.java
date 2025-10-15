package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="PIDF TESTER")
public class PIDF_TestChatGPT extends LinearOpMode {

    // === UPDATE THESE FOR YOUR HARDWARE ===
    // 28 counts per motor rev, hub reports x4 → 112 ticks per *motor* rev (most FTC motors)
    static final double TICKS_PER_MOTOR_REV = 112.0;
    // If your encoder is on the flywheel shaft after gearing, set GEAR_RATIO accordingly.
    // Example: motor:flywheel = 1:3 (geared up 3x) → encoder still on motor → GEAR_RATIO = 1.0
    // If encoder is on the flywheel shaft itself, set GEAR_RATIO to the actual reduction (e.g., 0.333).
    static final double GEAR_RATIO = 1.0;
    static final double TICKS_PER_OUTPUT_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;

    // Targets you can toggle (short vs long shot)
    double targetRPM = 4200;         // default hold speed
    final double closeShotRPM = 4200;
    final double longShotRPM  = 6000;

    // PIDF (we’ll compute kF from assumed max and tune kP live)
    double kP = 0.12, kI = 0.00, kD = 0.00, kF = 0.0;
    final double assumedMaxRPM = 6000.0; // flywheel free speed at the shaft with the encoder

    // “ready” window to allow firing
    final double rpmTolerance = 100;     // allowed error to fire
    final int    readyHoldMs  = 150;     // must be in-window this long

    @Override
    public void runOpMode() {
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // flywheels prefer FLOAT

        // Initial kF: REV convention (ticks/sec domain)
        double maxTicksPerSec = (assumedMaxRPM * TICKS_PER_OUTPUT_REV) / 60.0;
        kF = 32767.0 / maxTicksPerSec;

        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));

        telemetry.addLine("Controls:");
        telemetry.addLine("  RB: spin/hold   LB: stop");
        telemetry.addLine("  X: close shot   B: long shot");
        telemetry.addLine("  Y/A: kP +/- 0.01   Dpad L/R: kF +/- 0.0005");
        telemetry.update();

        waitForStart();

        boolean holding = false;
        long readySince = 0;

        while (opModeIsActive()) {
            // Setpoints
            if (gamepad1.x) targetRPM = closeShotRPM;
            if (gamepad1.b) targetRPM = longShotRPM;

            // Hold enable/disable
            if (gamepad1.right_bumper) holding = true;
            if (gamepad1.left_bumper)  { holding = false; outtake.setPower(0); }

            // Live tuning
            if (gamepad1.y) { kP += 0.01; applyPIDF(outtake, kP, kI, kD, kF); sleep(100); }
            if (gamepad1.a) { kP = Math.max(0, kP - 0.01); applyPIDF(outtake, kP, kI, kD, kF); sleep(100); }
            if (gamepad1.dpad_right) { kF += 0.0005; applyPIDF(outtake, kP, kI, kD, kF); sleep(100); }
            if (gamepad1.dpad_left)  { kF = Math.max(0, kF - 0.0005); applyPIDF(outtake, kP, kI, kD, kF); sleep(100); }

            // Command velocity
            double targetTicksPerSec = (targetRPM * TICKS_PER_OUTPUT_REV) / 60.0;
            if (holding) outtake.setVelocity(targetTicksPerSec);

            // Telemetry + “ready to fire” gate
            double measTicksPerSec = outtake.getVelocity();
            double measRPM = measTicksPerSec * 60.0 / TICKS_PER_OUTPUT_REV;
            double err = targetRPM - measRPM;

            boolean inWindow = Math.abs(err) <= rpmTolerance;
            long now = System.currentTimeMillis();
            if (inWindow) {
                if (readySince == 0) readySince = now;
            } else {
                readySince = 0;
            }
            boolean ready = holding && readySince != 0 && (now - readySince) >= readyHoldMs;

            telemetry.addData("kP", "%.3f", kP);
            telemetry.addData("kF", "%.5f", kF);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Meas RPM", "%.0f", measRPM);
            telemetry.addData("Error", "%.0f", err);
            telemetry.addData("READY", ready ? "YES" : "no");
            telemetry.update();
        }
    }

    private void applyPIDF(DcMotorEx m, double p, double i, double d, double f) {
        m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }
}
