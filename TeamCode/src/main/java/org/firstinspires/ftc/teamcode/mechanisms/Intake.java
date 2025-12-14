package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotorEx motor;

    public Intake(DcMotorEx motor) {
        this.motor = motor;
    }

    public void forward(){
        motor.setPower(0.8);
    }
    public void reverse(){
        motor.setPower(-0.8);
    }
    public void stop(){
        motor.setPower(0);
    }

    public Action spinDown() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(0);
                    initialized = true;
                }


                packet.put("spinning", 0);
                return true;
            }
        };
    }

    public Action spinUp() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(0.8);
                    initialized = true;
                }


                packet.put("spinning", 1);
                return true;
            }
        };
    }
}
