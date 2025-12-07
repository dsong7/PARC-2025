package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous(name = "AprilTagTest ID", group = "DECODE")
public class AprilTagTest extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificID(21);
        aprilTagWebcam.displayDetectionTelemetry(id21);
        // to get specific varaiables:


    }
}
