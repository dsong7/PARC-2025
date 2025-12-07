package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    MecanumDrive drive;
    Intake intake;
    Outtake outtake;


    public boolean update = false;

    public Robot(Gamepad gamepad1, DcMotor LFD, DcMotor LBD, DcMotor RFD, DcMotor RBD, DcMotor spinner, DcMotor[] lifts, Servo drop, BNO055IMU imu, HardwareMap hardwareMap) {
        drive = new MecanumDrive(gamepad1, LFD, LBD, RFD, RBD, imu, hardwareMap);
        intake = new Intake(gamepad1, spinner);
        outtake = new Outtake(gamepad1, lifts, drop);
    }
    public Robot(DcMotor LFD, DcMotor LBD, DcMotor RFD, DcMotor RBD, DcMotor spinner, DcMotor[] lifts, Servo drop, BNO055IMU imu, HardwareMap hardwareMap) {
        drive = new MecanumDrive(LFD, LBD, RFD, RBD, imu, hardwareMap);
        intake = new Intake(spinner);
        outtake = new Outtake(lifts, drop);
    }

    public void controllerMode() {
        drive.fieldCentricDrive();
        intake.intakeByController();
        outtake.liftByController();
    }

}