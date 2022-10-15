package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class drivetrainEvan {

    //Motor Declaration
    private final DcMotor motorfrontleft;
    private final DcMotor motorbackleft;
    private final DcMotor motorfrontright;
    private final DcMotor motorbackright;

    //Constants
     double MOTOR1_SPEED = 0.7;
    double MOTOR2_SPEED = 0.7;
     double MOTOR3_SPEED = 0.7;
     double MOTOR4_SPEED = 0.7;

    double MOTOR1_BackSPEED = -0.7;
     double MOTOR2_BackSPEED = -0.7;
     double MOTOR3_BackSPEED = -0.7;
     double MOTOR4_BackSPEED = -0.7;

    //Constructor
    public drivetrainEvan(DcMotor motorfrontleft, DcMotor motorbackleft, DcMotor motorfrontright, DcMotor motorbackright)   {
    this.motorfrontleft = motorfrontleft;
    this.motorbackleft = motorbackleft;
    this.motorfrontright = motorfrontright;
    this.motorbackright = motorbackright;

        motorfrontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbackleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfrontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbackright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//Methods


    public void straight () {
        motorfrontleft.setPower(MOTOR1_SPEED);
        motorbackleft.setPower(MOTOR2_SPEED);
        motorfrontright.setPower(MOTOR3_SPEED);
        motorbackright.setPower(MOTOR4_SPEED);
    }

public void straightleft () {
        motorbackleft.setPower(MOTOR2_SPEED);
        motorfrontright.setPower(MOTOR3_SPEED);
    }

    public void straightright () {
        motorfrontleft.setPower(MOTOR1_SPEED);
        motorbackright.setPower(MOTOR4_SPEED);
    }

    public void left () {
        motorfrontleft.setPower(-MOTOR1_SPEED);
        motorbackleft.setPower(MOTOR2_SPEED);
        motorfrontright.setPower(MOTOR3_SPEED);
        motorbackright.setPower(-MOTOR4_SPEED);
    }

    public void right () {
        motorfrontleft.setPower(MOTOR1_SPEED);
        motorbackleft.setPower(-MOTOR2_SPEED);
        motorfrontright.setPower(-MOTOR3_SPEED);
        motorbackright.setPower(MOTOR4_SPEED);
    }

    public void backleft () {
        motorfrontleft.setPower(-MOTOR1_SPEED);
        motorbackright.setPower(-MOTOR4_SPEED);
    }

    public void backright () {
        motorbackleft.setPower(-MOTOR2_SPEED);
        motorfrontright.setPower(-MOTOR3_SPEED);
    }

    public void back () {
        motorfrontleft.setPower(-MOTOR1_SPEED);
        motorbackleft.setPower(-MOTOR2_SPEED);
        motorfrontright.setPower(-MOTOR3_SPEED);
        motorbackright.setPower(-MOTOR4_SPEED);
    }






}
