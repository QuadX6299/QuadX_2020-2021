package org.firstinspires.ftc.teamcode.QuadXlibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {

    private LinearOpMode opMode;
    private gyro Gyro;

    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        Gyro = new gyro(opMode, true);

        fl = opMode.hardwareMap.dcMotor.get("fl");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        br = opMode.hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

    }

    public void setPower(double LPower, double RPower) {
        double max = Math.max(Math.abs(LPower), Math.abs(RPower));

        if(max > 1){
            LPower /= max;
            RPower /=max;
        }

        fl.setPower(LPower);
        bl.setPower(LPower);
        fr.setPower(RPower);
        br.setPower(RPower);

    }

    public void stopMotors() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }

    public void turn(double power, boolean right) {
        if (right) {
            fl.setPower(power);
            bl.setPower(-power);
            fr.setPower(power);
            br.setPower(-power);

        } else {
            fl.setPower(-power);
            bl.setPower(power);
            fr.setPower(-power);
            br.setPower(power);

        }

    }

    public double getAvgEncoder(){

        double avgAdd = Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(br.getCurrentPosition());
        double div = 4.0;
        if(fl.getCurrentPosition() == 0){
            div --;
        }
        if(bl.getCurrentPosition() == 0){
            div --;
        }
        if(fr.getCurrentPosition() == 0){
            div --;
        }
        if(br.getCurrentPosition() == 0){
            div --;
        }
        if(div == 0){
            return 0;
        } else{
            return avgAdd / div;
        }

    }

    public void turn(double kP, double kI, double kD, double angle, boolean right, double timeout){
        ElapsedTime time = new ElapsedTime();

        time.startTime();

        double initialAngle = gyro.getGyroYaw();

        double error;
        double power;

        double proportional;
        double integral = 0;
        double derivative;

        double previousTime;
        double previousError = gyro.trueDiff(angle);

        while (Math.abs(gyro.getGyroYaw() - (angle + initialAngle)) > 1 && time.seconds() < timeout) {
            error = gyro.trueDiff(angle);

            previousTime = time.seconds();

            proportional = error * kP;
            integral += error * (time.seconds() - previousTime) * kI;
            derivative = ((error - previousError) / (time.seconds() - previousTime)) * kD;

            power = proportional + integral + derivative;

            turn(power, right);

            opMode.telemetry.addData("power", power);
            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("proportional", proportional);
            opMode.telemetry.addData("integral", integral);
            opMode.telemetry.addData("derivative", derivative);

            previousError = error;

            opMode.idle();


        }

        stopMotors();

    }

    public void goStraight(double distance, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double oldGyro = gyro.getGyroYaw();
        double power;
        double error = 0;
        double currTime = timer.milliseconds();
        double LhAdjust = 0;
        double RhAdjust = 0;
        double integral = 0;
        double derivative;
        double proportional;
        double oldTime = currTime;


        while (Math.abs(distance) > Math.abs(getAvgEncoder()) && !newOp.isStopRequested()) {
            currTime = timer.milliseconds();

            proportional = (distance - getAvgEncoder()) * kP;
            integral += (distance - getAvgEncoder()) * (currTime - oldTime) * kI;
            derivative = sensors.getTrueDiff(oldGyro) * kD;
            power = integral + proportional + derivative;

            error = sensors.getCurrGyro() - oldGyro;

            RhAdjust = -(error * .028);
            LhAdjust = (error * .028);

            if(power < 0.15 && distance > 0){
                power = 0.15;
            }
            if(power > -0.15 && distance < 0){
                power = -0.15;
            }

            if(Math.abs(power) > Math.abs(max)){
                power = max;
            }
            oldTime = currTime;
            straight(power + LhAdjust, power + RhAdjust);

            newOp.telemetry.addData("Avg Encoder Val", getAvgEncoder());
            newOp.telemetry.addData("Gyro Error", error);
            newOp.telemetry.addData("Forward power", power);
            newOp.telemetry.addData("Integral", integral);
            newOp.telemetry.addData("Left power: ", LhAdjust);
            newOp.telemetry.addData("Right power: ", RhAdjust);
            newOp.telemetry.update();
            if (currTime > timeout) {
                break;
            }
        }

        stopMotors();
    }


}
