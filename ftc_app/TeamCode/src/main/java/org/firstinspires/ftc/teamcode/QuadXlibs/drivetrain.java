package org.firstinspires.ftc.teamcode.QuadXlibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class drivetrain {

    private LinearOpMode opMode;
    private gyro Gyro;

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        Gyro = new gyro(opMode, true);

        frontLeft = opMode.hardwareMap.dcMotor.get("frontLeft");
        backLeft = opMode.hardwareMap.dcMotor.get("backLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("frontRight");
        backRight = opMode.hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

    }

    public void setPower(double LPower, double RPower) {
        double max = Math.max(Math.abs(LPower), Math.abs(RPower));

        if(max > 1){
            LPower /= max;
            RPower /=max;
        }

        frontLeft.setPower(LPower);
        backLeft.setPower(LPower);
        frontRight.setPower(RPower);
        backRight.setPower(RPower);

    }

    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

    }

    public void turn(double power, boolean right) {
        if (right) {
            frontLeft.setPower(power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(-power);

        } else {
            frontLeft.setPower(-power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(power);

        }

    }

    public double getAvgEncoder(){

        double avgAdd = Math.abs(frontLeft.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition());
        double div = 4.0;
        if(frontLeft.getCurrentPosition() == 0){
            div --;
        }
        if(backLeft.getCurrentPosition() == 0){
            div --;
        }
        if(frontRight.getCurrentPosition() == 0){
            div --;
        }
        if(backRight.getCurrentPosition() == 0){
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
