package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "DECODE Auto RR (No Tags)", group = "DECODE")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        Pose2d parkPose = new Pose2d(5, 5, Math.toRadians(90));

        telemetry.addData("StartPose", startPose);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 3) Build the main sequence
        Action park = drive.actionBuilder(startPose).strafeTo(new Vector2d(5, 5)).build();
        Actions.runBlocking(park);

        // Telemetry loop (helpful for debugging final pose drift)
        while (opModeIsActive()) {

            idle();
        }
    }

}
