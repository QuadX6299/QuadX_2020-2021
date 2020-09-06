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
        intakeFL = opMode.hardwareMap.dcMotor.get("intakeFL");
        intakeBR = opMode.hardwareMap.dcMotor.get("intakeBR");

        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setPower(double Power) {
        intakeFL.setPower(-Power);
        intakeBR.setPower(Power);

    }

    public void intake(double timeout){
        ElapsedTime time = new ElapsedTime();

        while (time.seconds() < timeout && opMode.opModeIsActive()) {
            intakeLF.setPower(-1);
            intakeBR.setPower(-1);

        }
        intakeFL.setPower(0);
        intakeBR.setPower(0);

    }
    }
}