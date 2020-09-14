package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake{
    private LinearOpMode opMode;

    private DcMotor intakeFL;
    private DcMotor intakeBR;

    public Intake(LinearOpMode opMode){
        intake = opMode.hardwareMap.dcMotor.get("intake");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setPower(double Power) {
        intake.setPower(Power);

    }

    public void intake(double timeout){
        ElapsedTime time = new ElapsedTime();

        while (time.seconds() < timeout && opMode.opModeIsActive()) {
            intake.setPower(1);

        }
        intake.setPower(0);

    }
    
}