package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* TODO
 * initial servo constants
 */
 /*
 * finger middle .25
 * */
@Config

public class TeleFluffy {
    DcMotor leftFront, leftBack, rightFront, rightBack, liftMotor, hangerMotor;
    DcMotorEx droneMotor;
    Servo grabberRot, finger, dronePusher, hangerLatch, leftPurple, rightPurple;
    RevBlinkinLedDriver blinkinLedDriver;

    public static double THRESHOLD = .15;
    OpMode op;

    // Lift Constants//
    public static double LIFT_MOTOR_MAX = 900;
    public static double LIFT_MOTOR_MIN = -20;
    public static int INCREMENT = 120;
    //  Drone Constants
    public static double DRONE_PUSHER_INIT = 0.8;
    public static double DRONE_PUSHER_LAUNCH = 1;
    // public static double DRONE_MOTOR_MULTIPLIER = .271;
    public static double DRONE_MOTOR_VELOCITY = 1.8; //changed from .3 during comp
    //  Finger & Grabber Constants
    public static double GRABBER_ROT_INIT = 0.5;
    public static double GRABBER_UP = 0.385;
    public static double GRABBER_DOWN = GRABBER_ROT_INIT;
    public static double FINGER_UP = 0;
    public static double FINGER_DOWN = .4;
    public static double FINGER_INIT = FINGER_DOWN;

    //  Hanger Constants
    public static double HANGER_LATCH_INIT = 0.87;
    public static double HANGER_LATCH_RELEASE = 0.815;
    public static double HANGER_POWER = 1; //uneducated guess
    public static int HANGER_MAX = 17100;
    boolean isGrabberUp = false;

    //slow approach
    public static final double APPROACH_SPEED = .26; //changed from 46 during comp


    public static double LEFT_PURPLE_INIT = .05;
    public static double RIGHT_PURPLE_INIT = 1;

    public static double CORRECT_PIXEL_DISTANCE = 4; //find actual distance in inches (we can use different units if we want)


    public TeleFluffy(OpMode op) {
        this.op = op;
        this.init();
    }

    public void init() {
        //getting where motors/servos are on map
        //motors
        leftFront = op.hardwareMap.dcMotor.get("leftFront");
        leftBack = op.hardwareMap.dcMotor.get("leftBack");
        rightFront = op.hardwareMap.dcMotor.get("rightFront");
        rightBack = op.hardwareMap.dcMotor.get("rightBack");
        liftMotor = op.hardwareMap.dcMotor.get("liftMotor");
        hangerMotor = op.hardwareMap.dcMotor.get("hangerMotor");
        droneMotor = (DcMotorEx) op.hardwareMap.dcMotor.get("droneMotor");
        //servos
        finger = op.hardwareMap.servo.get("finger");
        grabberRot = op.hardwareMap.servo.get("grabberRot");
        dronePusher = op.hardwareMap.servo.get("dronePusher");
        hangerLatch = op.hardwareMap.servo.get("hangerLatch");

        //setting direction for drive motors/resetting encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);//
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //setting initial position servos

        grabberRot.setPosition(GRABBER_ROT_INIT);
        finger.setPosition(FINGER_INIT);
        //finger.setDirection(Servo.Direction.REVERSE);
        dronePusher.setPosition(DRONE_PUSHER_INIT);
        hangerLatch.setPosition(HANGER_LATCH_INIT);


        leftPurple = op.hardwareMap.servo.get("leftPurple");
        leftPurple.setPosition(LEFT_PURPLE_INIT);

        rightPurple = op.hardwareMap.servo.get("rightPurple");
        rightPurple.setPosition(RIGHT_PURPLE_INIT);

        blinkinLedDriver = op.hardwareMap.get(RevBlinkinLedDriver.class, "bling");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        //states for grabber

    }
    //states for grabber

    public void liftTest(double power) {
        liftMotor.setPower(power);
    }

    //moves arm up and down
    public void setLiftUp(double liftMotorPower) {
        double liftMotorPosition = getLiftMotorPosition();
        if (liftMotorPosition < LIFT_MOTOR_MAX) {
            int newPosition = (int) liftMotorPosition + INCREMENT;
            liftMotor.setTargetPosition(newPosition);
            liftMotor.setPower(liftMotorPower);
        }
    }

    public void setLiftDown(double liftMotorPower) {
        double liftMotorPosition = getLiftMotorPosition();
        if (liftMotorPosition > LIFT_MOTOR_MIN) {
            int newPosition = (int) liftMotorPosition - INCREMENT;
            liftMotor.setTargetPosition(newPosition);
            liftMotor.setPower(liftMotorPower);
        }
    }

    public void setLiftStay(double position) {
        liftMotor.setTargetPosition((int) position);
    }

    public double getTargetPosition() {
        return liftMotor.getTargetPosition();
    }

    public void setTeleOpDrive(double forward, double strafe, double turn) {
        double leftFrontPower = trimPower(forward + strafe + turn);
        double rightFrontPower = trimPower(forward - strafe - turn);
        double leftBackPower = trimPower(forward - strafe + turn);
        double rightBackPower = trimPower(forward + strafe - turn);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public double trimPower(double Power) {
        if (Math.abs(Power) < THRESHOLD) {
            Power = 0;
        }
        return Power;

    }

    public void setDroneMotorSpeed() {
        droneMotor.setVelocity(DRONE_MOTOR_VELOCITY, AngleUnit.RADIANS);
    }

    public void setDroneMotorZero() {
        droneMotor.setVelocity(0);
    }

    public double getLiftMotorPosition() {
        return (liftMotor.getCurrentPosition());
    }

    public double trimMotorPower(double power) {
        return (trimPower(power));
    }

    public void setDronePusherLaunch() {
        dronePusher.setPosition(DRONE_PUSHER_LAUNCH);
    }

    public void setDronePusherInit() {
        dronePusher.setPosition(DRONE_PUSHER_INIT);
    }


    public void setModeAll(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void reportDriveMotors() {
        op.telemetry.addData("Left Front: ", leftFront.getCurrentPosition());
        op.telemetry.addData("Right Front: ", rightFront.getCurrentPosition());
        op.telemetry.addData("Left Back: ", leftBack.getCurrentPosition());
        op.telemetry.addData("Right Back: ", rightBack.getCurrentPosition());
    }

    public void raiseGrabber() {
        grabberRot.setPosition(GRABBER_UP);
        isGrabberUp = true;
    }

    public void lowerGrabber() {
        grabberRot.setPosition(GRABBER_DOWN);
        isGrabberUp = false;
    }

    public void setFingerUp() {
        finger.setPosition(FINGER_UP);
    }

    public void setFingerDown() {
        finger.setPosition(FINGER_DOWN);
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void releaseHanger() {
        hangerLatch.setPosition(HANGER_LATCH_RELEASE);
    }

    public void moveHangerUp() {
        if (hangerMotor.getCurrentPosition() < HANGER_MAX) {
            hangerMotor.setPower(HANGER_POWER);
        } // need an else clause to turn off motor if too high!
    }

    public void moveHangerDown() {
        hangerMotor.setPower(-HANGER_POWER);
    }

    public void stopHanger() {
        hangerMotor.setPower(0);
    }

    //slow approach
    public void slowForward() {
        leftFront.setPower(APPROACH_SPEED);
        rightFront.setPower(APPROACH_SPEED);
        leftBack.setPower(APPROACH_SPEED);
        rightBack.setPower(APPROACH_SPEED);
    }

    public void slowBackward() {
        leftFront.setPower(-APPROACH_SPEED);
        rightFront.setPower(-APPROACH_SPEED);
        leftBack.setPower(-APPROACH_SPEED);
        rightBack.setPower(-APPROACH_SPEED);
    }

    public void slowLeft() {
        leftFront.setPower(-APPROACH_SPEED * 2.5);
        rightFront.setPower(APPROACH_SPEED * 2.5);
        leftBack.setPower(APPROACH_SPEED * 2.5);
        rightBack.setPower(-APPROACH_SPEED * 2.5);
    }

    public void slowRight() {
        leftFront.setPower(APPROACH_SPEED * 2.5);
        rightFront.setPower(-APPROACH_SPEED * 2.5);
        leftBack.setPower(-APPROACH_SPEED * 2.5);
        rightBack.setPower(APPROACH_SPEED * 2.5);
    }


    public void setLeds(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLedDriver.setPattern(pattern);
    }

}
