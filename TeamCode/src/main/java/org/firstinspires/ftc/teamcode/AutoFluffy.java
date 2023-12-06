package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
public class AutoFluffy {
    LinearOpMode op;
    DcMotor liftMotor;
    Servo grabberRot, finger, hangerLatch, dronePusher, leftPurple, rightPurple;

    MecanumDrive drive;
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    TfodProcessor tfod;
    final int RESOLUTION_WIDTH = 960;
    final int RESOLUTION_HEIGHT = 720;
    public static double DRONE_PUSHER_RESET = 0.85;
    public static double DRONE_PUSHER_INIT=0.8;
    public static double GRABBER_ROT_INIT= 0.07;
    public static double GRABBER_UP=0.3;
    public static double GRABBER_DOWN=GRABBER_ROT_INIT;
    public static double FINGER_UP = 0;
    public static double FINGER_DOWN = .4;
    public static double FINGER_INIT = FINGER_DOWN;

    public static double HANGER_LATCH_INIT=0.87;

    public static double PURPLE_RELEASE = 0;
    public static double PURPLE_GRAB = 0;
    String side = "Red";

    String[] RED_LABELS = {"redprop"};
    String[] BLUE_LABELS = {"blueprop"};
    public AutoFluffy(LinearOpMode op) {
        this.op=op;
        this.init();
    }

    public AutoFluffy(LinearOpMode op, String side){
        this.op = op;
        this.side = side;
        this.init();

    }

    public void init() {
        liftMotor= op.hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberRot= op.hardwareMap.servo.get("grabberRot");
        grabberRot.setPosition(GRABBER_ROT_INIT);

        dronePusher= op.hardwareMap.servo.get("dronePusher");
        dronePusher.setPosition(DRONE_PUSHER_INIT);

        finger= op.hardwareMap.servo.get("finger");
        finger.setPosition(FINGER_INIT);

        hangerLatch= op.hardwareMap.servo.get("hangerLatch");
        hangerLatch.setPosition(HANGER_LATCH_INIT);




        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
        String[] LABELS;
        if (side.equals ("Red")){
            LABELS=RED_LABELS;
        }
         else {
             LABELS=BLUE_LABELS;
        }
        tfod = new TfodProcessor.Builder()
                .setModelFileName("redprop.tflite")
                .setModelAspectRatio(RESOLUTION_HEIGHT/RESOLUTION_WIDTH)  //verify with grace
                .setModelLabels(LABELS)
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------

        drive=new MecanumDrive(op.hardwareMap, new Pose2d(new Vector2d(0,0), 0));
        visionPortal = new VisionPortal.Builder()
                .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(tfod, aprilTag)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();

    }

    List<AprilTagDetection> findDetections(){
        return aprilTag.getDetections();
    }

    public void deliverPurple(){
        leftPurple.setPosition(PURPLE_RELEASE);
        rightPurple.setPosition(PURPLE_RELEASE);
        op.sleep(1000);
        leftPurple.setPosition(PURPLE_GRAB);
        rightPurple.setPosition(PURPLE_GRAB);
    }
    /*
    public void setGrabberOpen(){
        grabber.setPosition(GRABBER_OPEN);
    }
    public void setGrabberClosed(){
        grabber.setPosition(GRABBER_CLOSED);
    }
    public String getGrabberString() {
        String s = "GRABBER_OPEN: "+GRABBER_OPEN + "\nGRABBER_CLOSED: "+GRABBER_CLOSED;
        return s;
    }

    //moves arm up and down
    public void setLiftPower(double power){
        hornMotor.setPower(power);

    }

    //moves arm back and forth
    public void setBackwardArmPosition(){
        armServo.setPosition(ARM_BACKWARD);
    }
    public void setForwardArmPosition(){
        armServo.setPosition(ARM_FORWARD);
    }

    //moves grabber up and down
    public void setFlipperUp(){
        flipper.setPosition(FLIPPER_UP);
    }
    public void setFlipperDown(){
        flipper.setPosition(FLIPPER_DOWN);
    }
    public void setTeleOpDrive(double forward, double strafe, double turn){
        double leftFrontPower = trimPower(forward + strafe + turn);
        double rightFrontPower = trimPower(forward - strafe - turn);
        double leftBackPower = trimPower(forward - strafe + turn);
        double rightBackPower = trimPower(forward + strafe - turn);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
    //cycles through pusher positions
    public void setPusherPosition(int position) {
        if (position == 1){
            pusher.setPosition(PUSHER_POSITION_1);
        }
        else if (position == 2) {
            pusher.setPosition(PUSHER_POSITION_2);
        }
        else if (position == 3) {
            pusher.setPosition(PUSHER_POSITION_3);
        }
    }
    //pulled in from powerplay AgnesTeleop for use in setTeleOpDrive
    //called to trim power of driving joystick to ensure the robot does not move unintentionally

    public double trimPower(double Power) {
        if (Math.abs(Power) < THRESHOLD) {
            Power = 0;
        }
        return Power;

    }
    public void setDroneMotorSpeed(){
        droneMotor.setVelocity(DRONE_MOTOR_VELOCITY, AngleUnit.RADIANS);
    }
    public void setDroneMotorZero(){
        droneMotor.setVelocity(0);
    }

    public double getLiftMotorPosition(){
        double liftMotorPosition = hornMotor.getCurrentPosition();
        return(liftMotorPosition);
    }
    public double getLiftMotorPower(double power){
        double liftMotorPower = trimPower(power);
        return(liftMotorPower);
    }

    public void setDronePusherLaunch(){
        dronePusher.setPosition(DRONE_PUSHER_LAUNCH);
    }

    public void setDronePusherReset(){
        dronePusher.setPosition(DRONE_PUSHER_RESET);
    }


    public void setModeAll(DcMotor.RunMode mode){
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setPowerAll(double Power){
        leftFront.setPower(Power);
        leftBack.setPower(Power);
        rightFront.setPower(Power);
        rightBack.setPower(Power);
    }

    public void autoForward(int ticks, double power) {
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        setPowerAll(power);
        while(leftFront.isBusy() && op.opModeIsActive()){
            ;
        }
        setPowerAll(0);
    }

        public void autoTurn ( int ticks, double power){
            setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setTargetPosition(ticks);
            leftBack.setTargetPosition(ticks);
            rightFront.setTargetPosition(-ticks);
            rightBack.setTargetPosition(-ticks);

            setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

            setPowerAll(power);

        /*while(leftFront.isBusy() && op.isOpModeActive()){
            //nothing rn
        }
        setPowerAll(0);
    }

            public void reportAll(){
                op.telemetry.addData("leftFront:", leftFront.getCurrentPosition());
                op.telemetry.addData("leftBack:", leftBack.getCurrentPosition());
                op.telemetry.addData("rightFront:", rightFront.getCurrentPosition());
                op.telemetry.addData("rightBack:", rightBack.getCurrentPosition());
                op.telemetry.update();
            }
        }*/
    }
