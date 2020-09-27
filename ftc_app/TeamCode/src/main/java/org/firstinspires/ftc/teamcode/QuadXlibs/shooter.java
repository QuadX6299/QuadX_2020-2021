package org.firstinspires.ftc.teamcode.QuadXlibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;
import static java.lang.Math.max;

public class Shooter{
    private LinearOpMode opMode;

    public DcMotor shooter;

    public Servo servoFlap;
    public Servo shooterGate;




    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;

        shooter = opMode.hardwareMap.dcMotor.get("shooter");
        shooterGate = opMode.hardwareMap.servo.get("shooterGate");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }



    public void setPower(double Power) {
        double max = Math.abs(Power);

        if(max > 1){
            Power /= max;
        }

        shooter.setPower(Power);
    }

    public void stopMotors() {
        shooter.setPower(0);
    }


    public void Gate(boolean reset) {
        if(reset){
            shooterGate.setPosition(0);
        } else {
            shooterGate.setPosition(1);
        }

    }



}