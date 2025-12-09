package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TelemetryMecanumDrive {

    //driving motors
    public DcMotor LFD; //left front
    public DcMotor LBD; //left back
    public DcMotor RFD; //right front
    public DcMotor RBD; //right back

    //gamepad and hardware map
    public Gamepad gamepad1;
    public HardwareMap hardwareMap;

    //power variable
    private double LFP; //left front`
    private double LBP; //left back
    private double RFP; //right front
    private double RBP; //right back

    //joystick stuff
    private double joystickAngle; //angle of joystick
    private double joystickMagnitude; //magnitude
    private double turn; //turn amount
    private double zeroangle;

    //IMU variables
    BNO055IMU imu;
    Orientation orientation;

    //Feedback heading control
    private double targetHeading = 0.0; // radians
    private double kPHeading = 2.0;     // tune this
    private double headingHoldMax = 0.5; // max auto turn power

    public TelemetryMecanumDrive(Gamepad gamepad1, DcMotor LFD, DcMotor LBD, DcMotor RFD, DcMotor RBD, BNO055IMU imu) {
        //setting inputs
        this.LFD = LFD;
        this.LBD = LBD;
        this.RFD = RFD;
        this.RBD = RBD;
        this.gamepad1 = gamepad1;
        this.imu = imu;
        this.hardwareMap = hardwareMap;

        //getting it to what we want
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        zeroangle = orientation.firstAngle;
        targetHeading = orientation.firstAngle;

        LFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void fieldCentricDrive() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        drive(zeroangle - orientation.firstAngle);
    }

    public void drive(double curAngle) {
        // Update IMU orientation
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double currentHeading = orientation.firstAngle;

        // ---- Vector math (your stuff) ----
        joystickAngle = gamepad1.left_stick_x < 0
                ? Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x) + Math.PI
                : Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x);

        joystickAngle -= curAngle; // apply field-centric rotation
        joystickMagnitude = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));

        // ---- Heading hold logic for `turn` ----
        double manualTurn = gamepad1.right_stick_x;
        double deadband = 0.05;

        if (Math.abs(manualTurn) > deadband) {
            // Driver is actively rotating: pass-through and update target
            turn = manualTurn;
            targetHeading = currentHeading;
        } else {
            // Heading hold: use P controller
            double error = wrapAngle(targetHeading - currentHeading); // [-π, π]
            double autoTurn = kPHeading * error;

            // Limit how strong heading-hold can be
            autoTurn = Range.clip(autoTurn, -headingHoldMax, headingHoldMax);
            turn = autoTurn;
        }

        // ---- Mecanum math (your stuff) ----
        double powerFrontLeftBackRight = (Math.sin(joystickAngle) - Math.cos(joystickAngle)) * joystickMagnitude;
        double powerFrontRightBackLeft = (-Math.sin(joystickAngle) - Math.cos(joystickAngle)) * joystickMagnitude;

        if (Double.isNaN(powerFrontLeftBackRight)) {
            powerFrontLeftBackRight = 0D;
        }
        if (Double.isNaN(powerFrontRightBackLeft)) {
            powerFrontRightBackLeft = 0D;
        }
        if (Math.abs(gamepad1.left_stick_y) < .01 && Math.abs(gamepad1.left_stick_x) < .01) {
            powerFrontLeftBackRight = 0;
            powerFrontRightBackLeft = 0;
        }

        // Re-zero field orientation if you want
        if (gamepad1.a) {
            zeroangle = currentHeading;
            targetHeading = currentHeading; // keep hold aligned too
        }

        // Combining power and turn
        LFP = 0.75 * Range.clip(powerFrontLeftBackRight - turn, -1, 1);
        RBP = 0.75 * Range.clip(-powerFrontLeftBackRight - turn, -1, 1);
        RFP = 0.75 * Range.clip(powerFrontRightBackLeft - turn, -1, 1);
        LBP = 0.75 * Range.clip(-powerFrontRightBackLeft - turn, -1, 1);

        // Set motor power
        LFD.setPower(LFP);
        RFD.setPower(RFP);
        LBD.setPower(LBP);
        RBD.setPower(RBP);
    }


    private double wrapAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
