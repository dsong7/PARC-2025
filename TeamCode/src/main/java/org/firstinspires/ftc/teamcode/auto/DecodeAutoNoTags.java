package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.field.FieldPoses;

@Autonomous(name = "DECODE Auto RR (No Tags)", group = "DECODE")
public class DecodeAutoNoTags extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(5, 5, Math.toRadians(90));

        telemetry.addData("StartPose", startPose);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 3) Build the main sequence
        Action park = drive.actionBuilder(startPose).strafeTo(scorePose).build();
        Actions.runBlocking(park);

        // Telemetry loop (helpful for debugging final pose drift)
        while (opModeIsActive()) {
            telemetry.addData("Final Pose", drive.getPoseEstimate());
            telemetry.update();
            idle();
        }
    }
}
