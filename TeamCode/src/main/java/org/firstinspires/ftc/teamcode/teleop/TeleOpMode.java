package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name = "TeleOpMode", group = "TeleOpModes")
public class TeleOpMode extends LinearOpMode {

    Robot robot;

    public void runOpMode() {
        robot = new Robot(gamepad1, hardwareMap.get(BNO055IMU.class, "imu"), hardwareMap.get(DcMotor.class, "LFD"), hardwareMap.get(DcMotor.class, "LBD"), hardwareMap.get(DcMotor.class, "RFD"), hardwareMap.get(DcMotor.class, "RBD"),
                hardwareMap.get(DcMotor.class, "intake"), hardwareMap.get(DcMotor.class, "flywheel1"), hardwareMap.get(DcMotor.class, "flywheel2"));
        telemetry.setMsTransmissionInterval(250);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                robot.update();
            }
        }
    }
}
