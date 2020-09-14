package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class shooter{
    private LinearOpMode opMode;

    public DcMotor flywheel1;
    public DcMotor flywheel2;

    public Servo Angle;
    public Servo gate;




    public flywheels(LinearOpMode opMode) {
        this.opMode = opMode;

        flywheel1 = opMode.hardwareMap.dcMotor.get("flywheel1");
        flywheel2 = opMode.hardwareMap.dcMotor.get("flywheel2");
        gate = opMode.hardwareMap.servo.get("gate");

        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }



    public void setPower(double Power) {
        double max = Math.max(Math.abs(Power);

        if(max > 1){
            Power /= max;
        }

        flywheel1.setPower(Power);
        flywheel2.setPower(Power);
    }

    public void stopMotors() {
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }


    public void Gate(boolean reset) {
        if(reset){
            gate.setPosition(0);
        } else {
            gate.setPosition(1);
        }

    }



}