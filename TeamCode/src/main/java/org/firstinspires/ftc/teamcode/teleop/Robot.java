package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    public Robot(Gamepad gamepad1, Gamepad gamepad2, BNO055IMU imu, DcMotorEx LFD, DcMotorEx LBD, DcMotorEx RFD, DcMotorEx RBD, DcMotorEx spinner, DcMotorEx flywheel1, DcMotorEx flywheel2) {
        //drive = new TelemetryMecanumDrive(gamepad1, LFD, LBD, RFD, RBD, imu);
        //intake = new Intake(spinner);
        outtake = new Outtake(flywheel1);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public Robot(DcMotorEx LFD, DcMotorEx LBD, DcMotorEx RFD, DcMotorEx RBD, DcMotorEx spinner, Servo drop, BNO055IMU imu) {
        //drive = new MecanumDrive(LFD, LBD, RFD, RBD, imu, hardwareMap);
        //intake = new Intake(spinner);
        //outtake = new Outtake(lifts, drop);
    }

    public void update() {
        //TelemetryPacket packet = drive.fieldCentricDrive();
        outtake.update();
        runCommands();

        //return packet;
    }

    private void runCommands(){
        //INTAKE
        /*
        if(gamepad1.right_trigger > 0.5){
            intake.forward();
        } else if (gamepad1.left_trigger > 0.5) {
            intake.reverse();
        }else{
            intake.stop();
        }
        */
        //outtake

        //gp2:
        /*
        outtake
            right bumper: set holding speed
            left bumper: spin it down
            right trigger: launch = servo kicker
        transfer:
            a: cycle 120 deg clockwise
            b: cycle 120 deg counterclockwise
        intake:
            left joystick: control the spin
         */

        if (gamepad2.right_trigger > 0.8){
            //launch
            outtake.launch();
        }

        if (gamepad2.right_bumper) {
            outtake.setTargetRPM(4000);
            outtake.setHolding(true);
        }

        if (gamepad2.left_bumper){
            outtake.setHolding(false);
        }


    }


}