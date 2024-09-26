package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import kotlin.text.CharDirectionality;

public class Extendo {

    DcMotor extendoLeft;
    DcMotor extendoRight;
    enum LiftState {LIFTSTART, LIFTEXTEND, LIFTARM, LIFTRELEASE, LIFTRETRACT}
    LiftState liftState = LiftState.LIFTSTART;

    double currentpos = 0;
    public Extendo(HardwareMap HWMap){
        extendoLeft = HWMap.get(DcMotor.class, "extendoLeft");
        extendoRight = HWMap.get(DcMotor.class, "extendoRight");

        extendoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftState liftState = LiftState.LIFTSTART;

        currentpos = extendoLeft.getCurrentPosition();
    }

    public void goToPos(int pos) {
        extendoRight.setTargetPosition(pos);
        extendoLeft.setTargetPosition(pos);
        extendoLeft.setPower(1);
        extendoRight.setPower(1);
        extendoRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendoLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    public void update(){
//        switch (liftState) {
//            case LIFTSTART:
//                if (gamepad1.x) {
//                    liftLeftDIP.goal = 100;
//                    liftLeftDIP.position = liftLeftMotor.getCurrentPosition();
//                    liftRightDIP.goal = 100;
//                    liftRightDIP.position = liftLeftMotor.getCurrentPosition();
//                    liftState = LiftState.LIFTEXTEND;
//                }
//                break;
//            case LIFTEXTEND:
//                if (Math.abs(liftLeftDIP.goal - liftLeftDIP.position) < 10) {
//                    armLeftServo.setPosition(100);
//                    armRightServo.setPosition(100);
//                    liftState = LiftState.LIFTARM;
//                } else {
//                    liftLeftDIP.position = liftLeftMotor.getCurrentPosition();
//                    liftLeftMotor.setPower(liftLeftDIP.moveSomeIdk(timer.seconds()));
//                    liftRightDIP.position = liftRightMotor.getCurrentPosition();
//                    liftRightMotor.setPower(liftRightDIP.moveSomeIdk(timer.seconds()));
//                }
//                break;
//            case LIFTARM:
//                liftState = LiftState.LIFTRELEASE;
//                break;
//            case LIFTRELEASE:
//                liftState = LiftState.LIFTRETRACT;
//                break;
//            case LIFTRETRACT:
//                liftState = LiftState.LIFTSTART;
//                break;
//            default:
//                liftState = LiftState.LIFTSTART;
//                break;
//        }
//
//
//        switch (extendoState) {
//            case EXTENDOSTART:
//                if (gamepad1.a) {
//                    extendoMotorDIP.goal = 100;
//                    extendoMotorDIP.position = extendoMotor.getCurrentPosition();
//                    extendoState = ExtendoState.EXTENDOEXTEND;
//                }
//                break;
//            case EXTENDOEXTEND:
//                if (Math.abs(extendoMotorDIP.goal - extendoMotorDIP.position) < 10) {
//                    extendoLeftServo.setPosition(100);
//                    extendoRightServo.setPosition(100);
//                    intakeMotor.setPower(1.0);
//                    extendoState = ExtendoState.EXTENDOINTAKE;
//                } else {
//                    extendoMotorDIP.position = extendoMotor.getCurrentPosition();
//                    extendoMotor.setPower(extendoMotorDIP.moveSomeIdk(timer.seconds()));
//                }
//                break;
//            case EXTENDOINTAKE:
//                if (gamepad1.a) { //change to be based on sensor input
//                    extendoMotorDIP.goal = -100;
//                    extendoMotorDIP.position = extendoMotor.getCurrentPosition();
//                    extendoLeftServo.setPosition(100);
//                    extendoRightServo.setPosition(0);
//                    extendoState = ExtendoState.EXTENDORETRACT;
//                }
//                break;
//            case EXTENDORETRACT:
//                if (Math.abs(extendoMotorDIP.goal - extendoMotorDIP.position) < 10) {
//                    extendoState = ExtendoState.EXTENDOSTART;
//                } else {
//                    extendoMotorDIP.position = extendoMotor.getCurrentPosition();
//                    extendoMotor.setPower(extendoMotorDIP.moveSomeIdk(timer.seconds()));
//                }
//
//                break;
//            default:
//                extendoState = ExtendoState.EXTENDOSTART;
//                break;
//    }

//    }
}

