package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Main Teleop ", group = "Teleop")
public class MainTeleOp extends LinearOpMode {

    private GamepadEx gp1,gp2;
    private double driveSpeed = 1, driveMultiplier = 1;

    ElapsedTime time = new ElapsedTime();

    public MotorEx bl, br, fl, fr;
    public OpMode opMode;

    public void runOpMode() throws InterruptedException{

        gp1 = new GamepadEx(gamepad1);

        fl = new MotorEx(this.hardwareMap, "LFD",Motor.GoBILDA.RPM_1150);
        fr = new MotorEx(this.hardwareMap, "RFD", Motor.GoBILDA.RPM_1150);
        bl = new MotorEx(this.hardwareMap, "LBD", Motor.GoBILDA.RPM_1150);
        br = new MotorEx(this.hardwareMap, "RBD", Motor.GoBILDA.RPM_1150);
        waitForStart();
        while (opModeIsActive()) {
            drive();
        }

    }

    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                (forwardBackSpeed - strafeSpeed + turnSpeed),
                (forwardBackSpeed + strafeSpeed - turnSpeed),
                (-forwardBackSpeed - strafeSpeed - turnSpeed),
                (-forwardBackSpeed + strafeSpeed + turnSpeed)
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(-speeds[2]);
        br.set(-speeds[3]);
    }
    public void drive() {
        gp1.readButtons();

        fl.setInverted(true);
        bl.setInverted(true);
        fl.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);

        double x = Math.max(Math.max(Math.abs(gp1.getLeftX()), Math.abs(gp1.getLeftY())), Math.abs(gp1.getRightX()));
        driveSpeed = 0.0121*Math.pow(2.7182818284,4.48*x)-0.0121;
        driveSpeed = Math.max(0, driveSpeed);
        Vector2d driveVector = new Vector2d(-gp1.getLeftX(), gp1.getLeftY()),
                turnVector = new Vector2d(gp1
                        .getRightX(), 0);
        driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed
        );
    }
}
