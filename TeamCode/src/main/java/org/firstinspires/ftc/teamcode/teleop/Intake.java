package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotor intake;
    public Gamepad gamepad1;
    public HardwareMap hardwareMap;

    public Intake(Gamepad gamepad1, DcMotor intake) {
        this.gamepad1 = gamepad1;
        this.intake = intake;
    }

    public Intake(DcMotor intake) {
        this.intake = intake;
    }

    public void intakeByController() {
        if(gamepad1.right_trigger > 0.5){
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.5) {
            intake.setPower(-1);
        }else{
            intake.setPower(0);
        }
    }

}
