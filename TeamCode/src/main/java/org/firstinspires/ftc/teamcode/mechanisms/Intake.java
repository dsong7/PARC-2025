package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotor intake;

    public Intake(DcMotor intake) {
        this.intake = intake;
    }

    public void forward(){
        intake.setPower(1);
    }
    public void reverse(){
        intake.setPower(-1);
    }
    public void stop(){
        intake.setPower(0);
    }
}
