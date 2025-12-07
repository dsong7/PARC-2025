package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous(name = "AprilTagLocal", group = "DECODE")
public class AprilTagLocalizationV1 extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        try {
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificID(24);
            double range20 = id20.ftcPose.range;
            double range24 = id24.ftcPose.range;
            double bearing20 = id20.ftcPose.bearing;
            double bearing24 = id24.ftcPose.bearing;
            double h = (1.0/22)*(Math.sin(Math.abs(bearing20-bearing24) * 3.14/180))*range20*range24;
            double x = Math.sqrt(range20*range20-(h*h));
            telemetry.addLine("x "+x);
            telemetry.addLine(String.valueOf(h));
            telemetry.addLine("range" +range20);
            telemetry.addLine("range24" +range24);
            aprilTagWebcam.displayDetectionTelemetry(id20);
            aprilTagWebcam.displayDetectionTelemetry(id24);
        } catch (Exception e) {

        }



    }
}
