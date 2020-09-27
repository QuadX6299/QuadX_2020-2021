package org.firstinspires.ftc.teamcode.QuadXlibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {

    private LinearOpMode opMode;

    private BNO055IMU Gyro;
    private Orientation angles;

    public Gyro(LinearOpMode opMode, boolean IMUenabled) {
        this.opMode = opMode;

        if (IMUenabled){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            Gyro = this.opMode.hardwareMap.get(BNO055IMU.class, "imu");
            Gyro.initialize(parameters);
        }

    }


    public void updateGyroValues() {
        angles = Gyro.getAngularOrientation();

    }

    public Acceleration getAcceleration() {
        updateGyroValues();
        return Gyro.getAcceleration();

    }

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;

    }

    public double getGyroPitch() {
        updateGyroValues();
        return angles.secondAngle;

    }

    public double getGyroRoll() {
        updateGyroValues();
        return angles.thirdAngle;

    }

    public double trueDiff(double turnDes) {
        double currAngle = getGyroYaw();
        if ((currAngle >= 0 && turnDes >= 0) || (currAngle <= 0 && turnDes <= 0)){
            return turnDes - currAngle;
        } else if(Math.abs(destTurn - currAng) <= 180) {
            return destTurn - currAng;
        } else if(destTurn > currAng) {
            return -(360 - (destTurn - currAng));
        } else {
            return 360 - (currAng - destTurn);
        }

}
