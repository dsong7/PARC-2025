package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "DECODE Auto RR (No Tags)", group = "DECODE")
public class DecodeAutoNoTags extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(10, 10, Math.toRadians(90));

        telemetry.addData("StartPose", startPose);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Action park = drive.actionBuilder(startPose)
                .strafeTo(parkPose.position) // Vector2d target
                .build();


        // Telemetry loop (helpful for debugging final pose drift)
        while (opModeIsActive()) {
            // Update localization each loop
            drive.updatePoseEstimate();
            Actions.runBlocking(park);

            // Read the current estimated pose from the drive's localizer
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("Final Pose X", pose.position.x);
            telemetry.addData("Final Pose Y", pose.position.y);
            telemetry.addData("Final Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
            idle();
        }
    }
}