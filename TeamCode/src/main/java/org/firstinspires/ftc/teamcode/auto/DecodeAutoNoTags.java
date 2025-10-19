package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "DECODE Auto RR (No Tags) - FIXED", group = "DECODE")
public class DecodeAutoNoTags extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. INITIALIZATION
        // It's good practice to define the start pose once.
        Pose2d startPose = new Pose2d(0, 0, 0);

        // Pass the start pose to the drive class during initialization.
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Define your target parking pose.
        Pose2d parkPose = new Pose2d(10, 10, Math.toRadians(90));

        // 2. BUILD THE ACTION
        // Build your action sequence here, BEFORE the match starts.
        Action park = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(10,10), parkPose.heading)
                .build();


        telemetry.addData("Initialization Complete", "Ready to start!");
        telemetry.update();

        waitForStart();


        Actions.runBlocking(park);
        // 3. EXECUTE THE ACTION
        // Run the entire 'park' action sequence once.
        // Actions.runBlocking() will execute the whole path and wait until it's done.




        // 4. (OPTIONAL) TELEMETRY LOOP
        // After the action is done, you can use a loop to monitor the robot's final position.
        // This is useful for debugging pose drift.
        while (opModeIsActive()) {

            Pose2d currentPose = drive.localizer.getPose(); // RoadRunner automatically updates 'drive.pose'

            telemetry.addLine("--- Autonomous Complete ---");
            telemetry.addData("Final Pose X", currentPose.position.x);
            telemetry.addData("Final Pose Y", currentPose.position.y);
            telemetry.addData("Final Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.update();

            // Yields the CPU thread to prevent the app from crashing.

        }


    }
}