package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake{
    private LinearOpMode opMode;

    public Servo arm;

    public outtake(LinearOpMode opMode){
        arm = opMode.hardwareMap.servo.get("arm");
    }

    public void Outtake(boolean reset){
        if(reset){
            arm.setPosition(0);
        } else {
            arm.setPosition(1);
        }
    }


}