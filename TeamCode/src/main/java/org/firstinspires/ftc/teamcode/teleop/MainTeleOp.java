package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Main Teleop ", group = "Teleop")
public class MainTeleOp extends LinearOpMode {

    private GamepadEx gp1,gp2;
    private double driveSpeed = 1, driveMultiplier = 1;

    ElapsedTime time = new ElapsedTime();
    public void runOpMode() throws InterruptedException{

    }
    public void drive() {
        gp1.readButtons();
        bot.prepMotors();
        double x = Math.max(Math.max(Math.abs(gp1.getLeftX()), Math.abs(gp1.getLeftY())), Math.abs(gp1.getRightX()));
        driveSpeed = 0.0121*Math.pow(2.7182818284,4.48*x)-0.0121;
        driveSpeed = Math.max(0, driveSpeed);
        Vector2d driveVector = new Vector2d(-gp1.getLeftX(), gp1.getLeftY()),
                turnVector = new Vector2d(gp1
                        .getRightX(), 0);
        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed
        );
    }
}
