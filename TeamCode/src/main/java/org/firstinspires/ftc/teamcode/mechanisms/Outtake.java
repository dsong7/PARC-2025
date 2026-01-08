package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

    // "Ready to fire" window
    private final double rpmTolerance;   // how close we have to be to target
    private final int    readyHoldMs;    // how long we must stay in-window

    // Hardware
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    // State
    private double targetRPM = 0.0;
    private boolean holding = false;
    private long readySinceMs = 0;
    private boolean ready = false;
    /**
     * Construct an Outtake using two named motors from the hardware map.
     * Uses default tolerance and ready time.
     */
    public Outtake(DcMotorEx flywheel1) {
        //this(flywheel1, flywheel2, 100, 150);
        this(flywheel1, 100, 150);
    }

    /**
     * Full constructor with tolerance and ready time configuration.
     */
    public Outtake(DcMotorEx flywheel1,
                   //DcMotorEx flywheel2,
                   double rpmTolerance,
                   int readyHoldMs) {

        this.rpmTolerance = rpmTolerance;
        this.readyHoldMs  = readyHoldMs;

        this.flywheel1 = flywheel1;
        // Reset and configure encoders
        this.flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //flywheel2.setDirection(REVERSE);        // reverse one side if they face opposite ways

        // Compute a reasonable kF from assumed max RPM (REV-style ticks/sec domain)
        double maxTicksPerSec = (ASSUMED_MAX_RPM * TICKS_PER_OUTPUT_REV) / 60.0;

    }

    /** Set target RPM (will be used next time update() runs while holding = true). */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    /** Enable or disable closed-loop holding of the target RPM. */
    public void setHolding(boolean holding) {
        this.holding = holding;
        if (!holding) {
            // stop motors and reset ready gate
            flywheel1.setPower(0);
            //flywheel2.setPower(0);
            readySinceMs = 0;
            ready = false;
        }
    }

    /** Returns whether closed-loop holding is currently enabled. */
    public boolean isHolding() {
        return holding;
    }
    //returns if RPM is at target RPM
    public boolean isReady(){
        return ready;
    }

    public void update() {
        if (!holding || targetRPM <= 0) {
            ready = false;
            return;
        }

        // Command velocity using FTC's built-in PIDF controller
        double targetTicksPerSec = rpmToTicksPerSec(targetRPM);
        flywheel1.setVelocity(targetTicksPerSec);
        //flywheel2.setVelocity(targetTicksPerSec);

        // Measure velocities and compute errors (for ready gate)
        double measRPM1 = ticksPerSecToRPM(flywheel1.getVelocity());
        //double measRPM2 = ticksPerSecToRPM(flywheel2.getVelocity());

        double err1 = targetRPM - measRPM1;
        //double err2 = targetRPM - measRPM2;

        boolean inWindow =
                Math.abs(err1) <= rpmTolerance;// && Math.abs(err2) <= rpmTolerance;

        long now = System.currentTimeMillis();
        if (inWindow) {
            if (readySinceMs == 0) {
                readySinceMs = now;
            }
        } else {
            readySinceMs = 0;
        }

        ready = holding && readySinceMs != 0 && (now - readySinceMs) >= readyHoldMs;

    }

    public void launch(){

    }



    // ------------------- Helpers -------------------

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_OUTPUT_REV / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_OUTPUT_REV;
    }

    // For telemetry from your OpMode if you want:
    public double getTargetRPM() {
        return targetRPM;
    }

    public double getMeasuredRPM1() {
        return ticksPerSecToRPM(flywheel1.getVelocity());
    }

    public double getMeasuredRPM2() {
        return ticksPerSecToRPM(flywheel2.getVelocity());
    }



}
