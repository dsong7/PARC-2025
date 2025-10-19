package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="PIDF TESTER")
public class PIDF_TestChatGPT extends LinearOpMode {

    // === UPDATE THESE FOR YOUR HARDWARE ===
    // 28 counts per motor rev, hub reports x4 → 112 ticks per *motor* rev (most FTC motors)
    static final double TICKS_PER_MOTOR_REV = 28.0;
    // If your encoder is on the flywheel shaft after gearing, set GEAR_RATIO accordingly.
    // Example: motor:flywheel = 1:3 (geared up 3x) → encoder still on motor → GEAR_RATIO = 1.0
    // If encoder is on the flywheel shaft itself, set GEAR_RATIO to the actual reduction (e.g., 0.333).
    static final double GEAR_RATIO = 1.0;
    static final double TICKS_PER_OUTPUT_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;

    // Targets you can toggle (short vs long shot)
    double targetRPM = 250;         // default hold speed
    final double closeShotRPM = 500;
    final double longShotRPM  = 1000;

    // PIDF (we’ll compute kF from assumed max and tune kP live)
    double kP = 0.12, kI = 0.10, kD = 0.10, kF = 0.0;
    final double assumedMaxRPM = 6000.0; // flywheel free speed at the shaft with the encoder

    // “ready” window to allow firing
    final double rpmTolerance = 100;     // allowed error to fire
    final int    readyHoldMs  = 150;     // must be in-window this long

    @Override
    public void runOpMode() {
        DcMotorEx outtake1 = hardwareMap.get(DcMotorEx.class, "Launcher1");
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotorEx outtake2 = hardwareMap.get(DcMotorEx.class, "Launcher2");
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setDirection(REVERSE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);// flywheels prefer FLOAT

        // Initial kF: REV convention (ticks/sec domain)
        double maxTicksPerSec = (assumedMaxRPM * TICKS_PER_OUTPUT_REV) / 60.0;
        kF = 32767.0 / maxTicksPerSec;

        outtake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
        outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
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
            if (gamepad1.left_bumper)  { holding = false; outtake1.setPower(0); outtake2.setPower(0);}

            // Live tuning
            if (gamepad1.y) { kP += 0.01; applyPIDF(outtake1, kP, kI, kD, kF); applyPIDF(outtake2, kP, kI, kD, kF); sleep(100); }
            if (gamepad1.a) { kP = Math.max(0, kP - 0.01); applyPIDF(outtake1, kP, kI, kD, kF); applyPIDF(outtake2, kP, kI, kD, kF); sleep(100); }
            if (gamepad1.dpad_right) { kF += 0.0005; applyPIDF(outtake1, kP, kI, kD, kF); applyPIDF(outtake2, kP, kI, kD, kF); sleep(100); }
            if (gamepad1.dpad_left)  { kF = Math.max(0, kF - 0.0005); applyPIDF(outtake1, kP, kI, kD, kF); applyPIDF(outtake2, kP, kI, kD, kF); sleep(100); }

            // Command velocity
            double targetTicksPerSec = (targetRPM * TICKS_PER_OUTPUT_REV) / 60.0;
            if (holding) outtake1.setVelocity(targetTicksPerSec);
            if (holding) outtake2.setVelocity(targetTicksPerSec);

            // Telemetry + “ready to fire” gate
            double measTicksPerSec1 = outtake1.getVelocity();
            double measRPM1 = measTicksPerSec1 * 60.0 / TICKS_PER_OUTPUT_REV;
            double err1 = targetRPM - measRPM1;
            double measTicksPerSec2 = outtake2.getVelocity();
            double measRPM2 = measTicksPerSec2 * 60.0 / TICKS_PER_OUTPUT_REV;
            double err2 = targetRPM - measRPM2;

            boolean inWindow = Math.abs(err1) <= rpmTolerance;
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
            telemetry.addData("Meas RPM1", measRPM1);
            telemetry.addData("Error1", err1);
            telemetry.addData("Meas RPM2", measRPM2);
            telemetry.addData("Error2", err2);
            telemetry.update();
        }
    }

    private void applyPIDF(DcMotorEx m, double p, double i, double d, double f) {
        m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }
}
