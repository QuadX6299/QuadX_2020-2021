package org.firstinspires.ftc.teamcode.QuadXlibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TeleLibs extends OpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor intakeFL;
    private DcMotor intakeBR;

    @Override
    public void init() {
        // MOTOR INITIALZATION
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        intakeFL = hardwareMap.dcMotor.get("intakeFL");
        intakeBR = hardwareMap.dcMotor.get("intakeBR");
        
    // ________________________________________________________________________________________________
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBR.setDirection(DcMotorSimple.Direction.REVERSE);
    }




        // =======================================  DRIVE  =============================================
   
    public void arcadeDrive() {
        //checking for valid range to apply power (has to give greater power than .1)
        if ((Math.abs(gamepad1.left_stick_y) > .1) ||
                Math.abs(gamepad1.right_stick_x) > .1) {

            if (Math.abs(gamepad1.left_stick_y) > .1){
                frontLeft.setPower(-gamepad1.left_stick_y);
                frontRight.setPower(-gamepad1.left_stick_y);
                backLeft.setPower(-gamepad1.left_stick_y);
                backRight.setPower(-gamepad1.left_stick_y); 
            }
            else {
                frontLeft.setPower(gamepad1.right_stick_x);
                frontRight.setPower(-gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);
            }

        }
        else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }

    }

    // =======================================  INTAKE  ============================================

    public void intakeSlides() {
        double left_trigger = gamepad1.left_trigger;
        double right_trigger = gamepad1.right_trigger;

        if (left_trigger > 0.05) {
            //the right intake slide is set to reverse, may have to change that
            intakeFL.setPower(-left_trigger);
            intakeBR.setPower(-left_trigger);

        }
        else if (right_trigger > 0.05) {
            intakeFL.setPower(right_trigger);
            intakeBR.setPower(right_trigger);

        }
        else {
            intakeFL.setPower(0);
            intakeBR.setPower(0);

        } 
    }
    }
    // https://github.com/rohitchawla28/LactoseIntolerant_/blob/master/ftc_app-master/TeamCode/src/main/java/LactoseIntolerant/Motion.java