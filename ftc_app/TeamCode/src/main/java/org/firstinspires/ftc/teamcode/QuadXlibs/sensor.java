package org.firstinspires.ftc.teamcode.QuadXlibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Don't know if this is ok
import org.firstinspires.ftc.teamcode.QuadXlibs.resetEncoders;

public class Rangesensor {

    private LinearOpMode opMode;
    private gyro Gyro;
    ModernRoboticsI2cRangeSensor rangeSensor1;
    ModernRoboticsI2cRangeSensor rangeSensor2;

    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;



    public void goShootingY(double position1, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();
        rangeSensor1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        double rangeS1 = rangeSensor1.getDistance(DistanceUnit.CM));
        double oldRangeS1 = rangeS1;
        double oldGyro = gyro.getGyroYaw();
        double power;
        double error = Math.abs(position1-rangeS1);
        double currTime = timer.milliseconds();
        double LhAdjust = 0;
        double RhAdjust = 0;
        double integral = 0;
        double derivative;
        double proportional;
        double oldTime = currTime;



        while (Math.abs(error) > Math.abs(0.5) && !newOp.isStopRequested()) {
            currTime = timer.milliseconds();

            proportional = (position1 - rangeS1) * kP;
            integral += (position1 - rangeS1) * (currTime - oldTime) * kI;
            derivative = sensors.getTrueDiff(oldRangeS1) * kD;
            power = integral + proportional + derivative;

            rangeS1 = rangeSensor1.getDistance(DistanceUnit.CM));

            error = rangeS1 - oldRangeS1;


            RhAdjust = -(error * .028);
            LhAdjust = (error * .028);

            if(power < 0.15 && position1 > 0){
                power = 0.15;
            }
            if(power > -0.15 && position1 < 0){
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

    public void goShootingX(double position2, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();
        rangeSensor2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        double rangeS2 = rangeSensor2.getDistance(DistanceUnit.CM));
        double oldRangeS2 = rangeS2;
        double oldGyro = gyro.getGyroYaw();
        double power;
        double error = Math.abs(position2-rangeS2);
        double currTime = timer.milliseconds();
        double LhAdjust = 0;
        double RhAdjust = 0;
        double integral = 0;
        double derivative;
        double proportional;
        double oldTime = currTime;



        while (Math.abs(error) > Math.abs(0.5) && !newOp.isStopRequested()) {
            currTime = timer.milliseconds();

            proportional = (position2 - rangeS2) * kP;
            integral += (position2 - rangeS2) * (currTime - oldTime) * kI;
            derivative = sensors.getTrueDiff(oldRangeS2) * kD;
            power = integral + proportional + derivative;

            rangeS2 = rangeSensor2.getDistance(DistanceUnit.CM));

            error = rangeS2 - oldRangeS2;


            RhAdjust = -(error * .028);
            LhAdjust = (error * .028);

            if(power < 0.15 && position2 > 0){
                power = 0.15;
            }
            if(power > -0.15 && position2 < 0){
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
