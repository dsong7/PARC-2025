package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.HardwareMap;


import java.lang.*;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "CannonTest")
public class CannonTest1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotor Motor1 = hardwareMap.get(DcMotor.class, "Launcher1");
        DcMotor Motor2 = hardwareMap.get(DcMotor.class, "Launcher2");

        waitForStart();
        while (opModeIsActive()){
            double power = this.gamepad1.left_stick_y;
            power = Math.min(power, 1);
            power = Math.max(power, 0);

            Motor1.setPower(power);
            Motor2.setPower(-power);
        }

    }
}