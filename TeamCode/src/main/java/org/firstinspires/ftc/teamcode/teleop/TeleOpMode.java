package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name = "TeleOpMode", group = "TeleOpModes")
public class TeleOpMode extends LinearOpMode {

    Robot robot;

    public void runOpMode() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        DcMotorEx lfd = hardwareMap.get(DcMotorEx.class, "LFD");
        DcMotorEx lbd = hardwareMap.get(DcMotorEx.class, "LBD");
        DcMotorEx rfd = hardwareMap.get(DcMotorEx.class, "RFD");
        DcMotorEx rbd = hardwareMap.get(DcMotorEx.class, "RBD");
        DcMotorEx intakeMotor = null;
        DcMotorEx flywheel1 = null;
        DcMotorEx flywheel2 = null;

        try{
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
            flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        } catch (Exception e){
        }


        robot = new Robot(gamepad1, imu, lfd, lbd, rfd, rbd, intakeMotor, flywheel1, flywheel2);
        telemetry.setMsTransmissionInterval(250);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                robot.update();
            }
        }
    }
}
