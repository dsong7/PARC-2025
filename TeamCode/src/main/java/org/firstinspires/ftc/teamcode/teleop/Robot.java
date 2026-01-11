package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Kicker;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.TelemetryMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

public class Robot {
    TelemetryMecanumDrive drive;
    Intake intake;
    Outtake cannon;

    Gamepad gamepad1;
    Gamepad gamepad2;

    Kicker kicker;
    Transfer transfer;

    public boolean update = false;

    public Robot(Gamepad gamepad1, Gamepad gamepad2, BNO055IMU imu, DcMotorEx LFD, DcMotorEx LBD, DcMotorEx RFD, DcMotorEx RBD, DcMotorEx spinner, DcMotorEx flywheel1, Servo kickerServo, Servo chamberServo) {
        drive = new TelemetryMecanumDrive(gamepad1, LFD, LBD, RFD, RBD, imu);
        intake = new Intake(spinner, gamepad2);
        cannon = new Outtake(flywheel1, gamepad2);
        kicker = new Kicker(kickerServo, gamepad2);
        transfer = new Transfer(chamberServo, gamepad2, kicker);
    }

    public void update() {
        //TelemetryPacket packet = drive.fieldCentricDrive();
        cannon.update();
        kicker.update();
        transfer.update();
        intake.update();
    }

    //gp2:
    /*
    outtake
         left joystick: charge the launcher
         right trigger: kick the ball into launcher
    transfer:
         right bumper: cycle 120 deg clockwise
         left bumper: cycle 120 deg counterclockwise
    intake:
         right joystick: spin the intake
     */

}