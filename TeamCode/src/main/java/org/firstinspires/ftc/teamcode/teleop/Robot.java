package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.TelemetryMecanumDrive;

public class Robot {
    TelemetryMecanumDrive drive;
    Intake intake;
    Outtake outtake;

    Gamepad gamepad1;
    Gamepad gamepad2;

    public boolean update = false;

    public Robot(Gamepad gamepad1, BNO055IMU imu, DcMotorEx LFD, DcMotorEx LBD, DcMotorEx RFD, DcMotorEx RBD, DcMotorEx spinner, DcMotorEx flywheel1, DcMotorEx flywheel2) {
        drive = new TelemetryMecanumDrive(gamepad1, LFD, LBD, RFD, RBD, imu);
        intake = new Intake(spinner);
        outtake = new Outtake((DcMotorEx) flywheel1, (DcMotorEx) flywheel2);
    }
    public Robot(DcMotorEx LFD, DcMotorEx LBD, DcMotorEx RFD, DcMotorEx RBD, DcMotorEx spinner, Servo drop, BNO055IMU imu) {
        //drive = new MecanumDrive(LFD, LBD, RFD, RBD, imu, hardwareMap);
        intake = new Intake(spinner);
        //outtake = new Outtake(lifts, drop);
    }

    public void update() {
        drive.fieldCentricDrive();
        outtake.update();
        runCommands();
    }

    private void runCommands(){
        //INTAKE
        if(gamepad1.right_trigger > 0.5){
            intake.forward();
        } else if (gamepad1.left_trigger > 0.5) {
            intake.reverse();
        }else{
            intake.stop();
        }

        //outtake
        if (gamepad2.right_trigger > 0.8){
            //launch
            outtake.launch(1600);
        }
    }


}