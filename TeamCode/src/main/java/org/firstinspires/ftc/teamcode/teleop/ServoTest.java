package org.firstinspires.ftc.teamcode.teleop;
import java.lang.Thread;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name = "Test", group = "TeleOpModes")
public class ServoTest extends LinearOpMode {

    Servo test;

    public void runOpMode() {
        test = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                test.setPosition(0);
                try{Thread.sleep(1000);}catch(Exception e){}
                test.setPosition(0.5);
                try{Thread.sleep(1000);}catch(Exception e){}
            }
        }
    }
}
