package org.firstinspires.ftc.teamcode.QuadXlibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class intake {
    private LinearOpMode opMode;

    private DcMotor intakeMotor;

    public intake(LinearOpMode opMode){
        intakeMotor = opMode.hardwareMap.dcMotor.get("intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setPower(double Power) {
        intakeMotor.setPower(Power);

    }

    public void Intake(double timeout){
        ElapsedTime time = new ElapsedTime();

        while (time.seconds() < timeout && opMode.opModeIsActive()) {
            intakeMotor.setPower(1);

        }
        intakeMotor.setPower(0);

    }
    
}