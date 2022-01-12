package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Lift extends Mechanism{
    DcMotor liftLeft;
    DcMotor liftRight;
    public static boolean retract;
    public static double retMult;

    // lift positions
    public static double highPos = 20;
    public static double midPos = 10;
    public static double lowPos = 0;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.08, 0, 0);
    public static double kF = 0; //min power to go against g

    PIDFController controller;

    // lift constants
    public static double SPOOL_SIZE_IN = 1.81102;
    public static double MOTOR_RATIO = 5.2;
    public static double TICKS_PER_REV = MOTOR_RATIO * 28.0;
    public static double GEAR_RATIO = 1.0;

    public static double targetPosition = 0;

    public void init(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

        // if you need to make sure to reverse as necessary

//        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //makes sure the motors don't move at zero power
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset encoders upon startup
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //for using RR PIDs
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PID controller
        controller = new PIDFController(coeffs, 0, 0, kF);

        //damp
        retract = false;
        retMult = 0.00005;
    }

    public void update() {
        updatePID(targetPosition);
    }

    public void updatePID(double target) {

        controller.setTargetPosition(target);
        //find the error
        double leftPow = controller.update(encoderTicksToInches(liftLeft.getCurrentPosition()));
        double rightPow = controller.update(encoderTicksToInches(liftRight.getCurrentPosition()));
        //compensate error
        if(!retract) {
            liftLeft.setPower(leftPow);
            liftRight.setPower(rightPow);
        }else {
            liftLeft.setPower(leftPow * retMult);
            liftRight.setPower(rightPow * retMult);
        }
    }

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public boolean getRetract() {
        return retract;
    }
    public void retracting(boolean status) {
        retract = status;
    }
}