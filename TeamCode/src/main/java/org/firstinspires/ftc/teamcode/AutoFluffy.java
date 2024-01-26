package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AutoFluffy {
    LinearOpMode op;
    DcMotor liftMotor;
    Servo grabberRot, finger, hangerLatch, dronePusher, leftPurple, rightPurple;
    RevBlinkinLedDriver blinkinLedDriver;

    public MecanumDrive drive;
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    //TfodProcessor tfod;//
    //RedFinder redFinder;
    final int RESOLUTION_WIDTH = 1920;
    final int RESOLUTION_HEIGHT = 1080;
    public static double DRONE_PUSHER_INIT = 0.8;
    public static double GRABBER_ROT_INIT = 0.07;
    public static double GRABBER_UP = 0.3;
    public static double FINGER_UP = 0;
    public static double FINGER_DOWN = .4;
    public static double FINGER_INIT = FINGER_DOWN;

    public static double HANGER_LATCH_INIT = 0.87;

    public static double LEFT_PURPLE_GRAB = .05;
    public static double LEFT_PURPLE_RELEASE = .2;

    public static double RIGHT_PURPLE_GRAB = 1;
    public static double RIGHT_PURPLE_RELEASE = .6;
    public static double LEFT_PURPLE_INIT = LEFT_PURPLE_GRAB;
    public static double RIGHT_PURPLE_INIT = RIGHT_PURPLE_GRAB;
    public static int LIFT_UP = 300;//changed from 300 during comp  //fix values
    public static int LIFT_UP_HIGH = 360;
    public static int LIFT_DOWN = 0;  //fix values
    public static double LIFT_POWER = 1;  //fix values
    public static int FINGER_UP_WAIT = 500;
    boolean isGrabberUp = false;
    String side = "Red";
    double deltaC_X, deltaC_Y;

    final Pose2d RR_CENTER_DELIVERY = new Pose2d(new Vector2d(52.2,-34.8), Math.toRadians(0.1));
    final Pose2d RR_RIGHT_DELIVERY = new Pose2d(new Vector2d(52.2,-41.3), Math.toRadians(0.1));
    final Pose2d RR_LEFT_DELIVERY = new Pose2d(new Vector2d(52.2,-27.4), Math.toRadians(0.1));
    final Pose2d RL_CENTER_DELIVERY = new Pose2d(new Vector2d(50,-37.1), Math.toRadians(0.1));
    final Pose2d RL_RIGHT_DELIVERY = new Pose2d(new Vector2d(50,-43.1), Math.toRadians(0.1));
    final Pose2d RL_LEFT_DELIVERY = new Pose2d(new Vector2d(50,-31.1), Math.toRadians(0.1));
    final Pose2d BR_CENTER_DELIVERY = new Pose2d(new Vector2d(77,30), Math.toRadians(0));
    final Pose2d BR_RIGHT_DELIVERY = new Pose2d(new Vector2d(77,26), Math.toRadians(0));
    final Pose2d BR_LEFT_DELIVERY = new Pose2d(new Vector2d(77, 34), Math.toRadians(0));
    final Pose2d BL_CENTER_DELIVERY = new Pose2d(new Vector2d(77,30), Math.toRadians(0));
    final Pose2d BL_LEFT_DELIVERY = new Pose2d(new Vector2d(77,34), Math.toRadians(0));
    final Pose2d BL_RIGHT_DELIVERY = new Pose2d(new Vector2d(77,26), Math.toRadians(0));

    public final Vector2d deltaF = new Vector2d(7.5,4.5);

    public final Vector2d[] tagPositions = new Vector2d[] {new Vector2d(62, 41.5),
                                                        new Vector2d(62, 35.5),
                                                        new Vector2d(62, 29.5),
                                                        new Vector2d(62,-29.5),
                                                        new Vector2d(62,-35.5),
                                                        new Vector2d(62, -41.5),
                                                                };

    public final Vector2d DELIVERY_OFFSET = new Vector2d(-8.75,-0.5);

    //String[] RED_LABELS = {"redprop"};
    //String[] BLUE_LABELS = {"blueprop"};
    private HueDetection hueDetector;

    public AutoFluffy(LinearOpMode op) {
        this.op = op;
        this.init();
    }

    public AutoFluffy(LinearOpMode op, String side) {
        this.op = op;
        this.side = side;
        this.init();

    }

    /*public AutoFluffy() {

    }*/

    AprilTagDetection assignID (String propLocation, String side){
        int idNum=0;

        if (side .equals("Blue")){
            if (propLocation .equals("Left")){
                idNum=1;

            }else if (propLocation .equals("Center")){
                idNum = 2;
            }else if (propLocation .equals("Right")){
                idNum= 3;
            }
        }else if (side .equals( "Red")){
            if (propLocation .equals("Left")){
                idNum= 4;
            }else if (propLocation .equals("Center")){
                idNum=5;
            }else if (propLocation .equals("Right")){
                idNum=6;
            }
        }
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = findDetections();
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections){
            if (detection.id == idNum){
                return detection;
            }
        }
        return null;
    }

    public void telemetryDetection (AprilTagDetection detection){
        if (detection==null){
            return;
        }
        if (detection.metadata!= null){
            op.telemetry.addData("ID: ", detection.id);
            op.telemetry.addData("Range(Distance from board): ", detection.ftcPose.range);
            op.telemetry.addData("Yaw: ", detection.ftcPose.yaw);
            op.telemetry.addData("Bearing: ", detection.ftcPose.bearing);
        }else{
            op.telemetry.addData("ID: ", detection.id);
            return;
        }
        op.telemetry.update();
    }


    public void init() {
        liftMotor = op.hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberRot= op.hardwareMap.servo.get("grabberRot");
        grabberRot.setPosition(GRABBER_ROT_INIT);

        dronePusher = op.hardwareMap.servo.get("dronePusher");
        dronePusher.setPosition(DRONE_PUSHER_INIT);

        finger = op.hardwareMap.servo.get("finger");
        finger.setPosition(FINGER_INIT);

        hangerLatch = op.hardwareMap.servo.get("hangerLatch");
        hangerLatch.setPosition(HANGER_LATCH_INIT);

        leftPurple = op.hardwareMap.servo.get("leftPurple");
        leftPurple.setPosition(LEFT_PURPLE_INIT);

        rightPurple = op.hardwareMap.servo.get("rightPurple");
        rightPurple.setPosition(RIGHT_PURPLE_INIT);

        aprilTag = new AprilTagProcessor.Builder()
                .build();

       // redFinder = new RedFinder();
        hueDetector= new HueDetection();

        blinkinLedDriver = op.hardwareMap.get(RevBlinkinLedDriver.class, "bling");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);




        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
        /*String[] LABELS;
        if (side.equals("Red")) {
            LABELS = RED_LABELS;
        } else {
            LABELS = BLUE_LABELS;
        }
        tfod = new TfodProcessor.Builder()
                .setModelFileName("model_20231209_112710.tflite")
                .setModelAspectRatio(RESOLUTION_WIDTH/RESOLUTION_HEIGHT)  //verify with grace
                .setModelLabels(LABELS)
                .build();
        */
        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------

        drive = new MecanumDrive(op.hardwareMap, new Pose2d(new Vector2d(0, 0), 0));

        visionPortal = new VisionPortal.Builder()
                .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(hueDetector, aprilTag)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();
        if (side.equals("Red")){
            deltaC_X = -3.86;//-2.75;
            deltaC_Y = -4.51;//-5.25;
        }
        else {
            deltaC_X = 3.86;
            deltaC_Y = 1.51;
        }
    }

    public List<AprilTagDetection> findDetections() {
        return aprilTag.getDetections();
    }

    public String getPropLocation(){
        return hueDetector.propLocation;
    }
    public double getLeftMean(){
        return hueDetector.leftMean;
    }
    public double getCenterMean(){
        return hueDetector.centerMean;
    }
    public double getRightMean(){
        return hueDetector.rightMean;
    }


    public void deliverPurple() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        leftPurple.setPosition(LEFT_PURPLE_RELEASE);
        rightPurple.setPosition(RIGHT_PURPLE_RELEASE);
        op.sleep(1000);
    }
    public void retractPurple(){
        leftPurple.setPosition(LEFT_PURPLE_GRAB);
        rightPurple.setPosition(RIGHT_PURPLE_GRAB);
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }


    /*public List<Recognition> getRecognitions(){
        return tfod.getRecognitions();
    }*/

    public void raiseGrabber(){
        grabberRot.setPosition(GRABBER_UP);
        isGrabberUp=true;
    }

    public void raiseLift() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        liftMotor.setTargetPosition(LIFT_UP);
        liftMotor.setPower(LIFT_POWER);
        while (op.opModeIsActive() && liftMotor.isBusy()) {
            op.sleep(1);
        }
    }
    public void raiseLiftHigh(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        liftMotor.setTargetPosition(LIFT_UP_HIGH);
        liftMotor.setPower(LIFT_POWER);
        while (op.opModeIsActive() && liftMotor.isBusy()){
            op.sleep(1);
        }
    }

        public void raiseFinger(){
            finger.setPosition(FINGER_UP);
            op.sleep(FINGER_UP_WAIT);
        }

        public void lowerLift(){
            liftMotor.setTargetPosition(LIFT_DOWN);
            liftMotor.setPower(LIFT_POWER);
            while (op.opModeIsActive() && liftMotor.isBusy()){
                op.sleep(1);
            }
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

     public Pose2d getPoseFromAprilTag() {
         List<AprilTagDetection> detections = findDetections();
         if (detections == null || detections.isEmpty()) {
             RobotLog.i("getPoseFromAprilTag: no detections");
             return drive.pose;
         }
         AprilTagDetection OurTag = detections.get(0);
         for (AprilTagDetection d : detections) {
             if (OurTag.ftcPose == null) {
                 OurTag = d;
                 continue;
             }
             if (d.ftcPose == null) {
                 continue;
             }
             if (Math.abs(d.ftcPose.x) < Math.abs(OurTag.ftcPose.x)) {
                 OurTag = d;
             }
         }
         if (OurTag.ftcPose == null) {
             RobotLog.i("getPoseFromAprilTag: no detections");
             return drive.pose;
         }

         Vector2d cameraVector = new Vector2d(OurTag.ftcPose.y, -OurTag.ftcPose.x);
         Vector2d rTag = tagPositions[OurTag.id - 1];
         Vector2d returnVector = rTag.minus(deltaF);
         returnVector = returnVector.minus(cameraVector);
         Pose2d returnPose = new Pose2d(returnVector, Math.toRadians(-OurTag.ftcPose.yaw));
         RobotLog.i("getPoseFromAprilTag: reference tag = "+OurTag.id);
         RobotLog.i(String.format("getPoseFromAprilTag: tag data: (%.3f, %.3f) @%.3f",OurTag.ftcPose.x, OurTag.ftcPose.y, OurTag.ftcPose.yaw));
         RobotLog.i("getPoseFromAprilTag: pose = "+returnPose.toString());

         return returnPose;

     }

     /*public Pose2d correctYellowPositionRed(String PATH, String SIDE) {
        AprilTagDetection detection = assignID(PATH, "Red");
        if (detection == null || detection.ftcPose == null) {
            RobotLog.i(String.format("running null case"));
            //need new values for backside
            if (SIDE .equals("Right")){
                if (PATH.equals("Left")) {
                    return RR_LEFT_DELIVERY;
                } else if (PATH.equals("Center")) {
                    return RR_CENTER_DELIVERY;
                } else if (PATH.equals("Right")) {
                    return RR_RIGHT_DELIVERY;
                }
            }else if(SIDE .equals("Left")){
                if (PATH.equals("Left")) {
                    return RL_LEFT_DELIVERY;
                } else if (PATH.equals("Center")) {
                    return RL_CENTER_DELIVERY;
                } else if (PATH.equals("Right")) {
                    return RL_RIGHT_DELIVERY;
                }
            }
        }
            RobotLog.i(String.format("not running null case"));
            double actual_X = -detection.ftcPose.x;
            double actual_Y = -detection.ftcPose.y;
            double D_X = actual_X - deltaC_X;
            double D_Y = actual_Y - deltaC_Y;
            double Target_X =  drive.pose.position.x + D_X;
            double Target_Y = drive.pose.position.y + D_Y;
            RobotLog.i(String.format("April Tag X: %3.1f    April Tag Y: %3.1f", actual_X, actual_Y));
            RobotLog.i(String.format("D_X: %3.1f  D_Y: %3.1f", D_X, D_Y));
            RobotLog.i(String.format("current pose: (%3.1f, %3.1f) at %3.1f deg", drive.pose.position.x, drive.pose.position.y,
                    Math.toDegrees(drive.pose.heading.toDouble())));
            double Target_Heading = Math.toRadians(detection.ftcPose.yaw) + drive.pose.heading.toDouble();
            RobotLog.i(String.format("Target: (%3.1f, %3.1f) at %3.1f deg", Target_X, Target_Y, Math.toDegrees(Target_Heading)));
            return new Pose2d(Target_X, Target_Y, Target_Heading);

        }*/

    /*public Pose2d correctYellowPositionBlue(String PATH, String SIDE) {
        //sleep(5000); //waiting for tag detections, might need less time
        AprilTagDetection detection = assignID(PATH, "Blue");
        if (detection == null || detection.ftcPose == null) {
            if (SIDE.equals("Left")) {
                if (PATH.equals("Left")) {
                    return BL_LEFT_DELIVERY;
                } else if (PATH.equals("Center")) {
                    return BL_CENTER_DELIVERY;
                } else if (PATH.equals("Right")) {
                    return BL_RIGHT_DELIVERY;
                }
            } else if (SIDE.equals("Right")) {
                if (PATH.equals("Left")) {
                    return BR_LEFT_DELIVERY;
                } else if (PATH.equals("Center")) {
                    return BR_CENTER_DELIVERY;
                } else if (PATH.equals("Right")) {
                    return BR_RIGHT_DELIVERY;
                }
            }
        }

            double actual_X = detection.ftcPose.x;
            double actual_Y = detection.ftcPose.y;
            double D_X = actual_X - deltaC_X;
            double D_Y = actual_Y - deltaC_Y;
            op.telemetry.addData("actual_X", actual_X);
            op.telemetry.addData("actual_Y", actual_Y);
            op.telemetry.addData("D_X", D_X);
            op.telemetry.addData("D_Y", D_Y);
            double Target_X = drive.pose.position.x + D_X;
            double Target_Y = drive.pose.position.y + D_Y;
            op.telemetry.addData("Target_X", Target_X);
            op.telemetry.addData("Target_Y", Target_Y);
            RobotLog.i(String.format("April Tag X: %3.1f    April Tag Y: %3.1f", actual_X, actual_Y));
            RobotLog.i(String.format("D_X: %3.1f  D_Y: %3.1f", D_X, D_Y));
            RobotLog.i(String.format("current pose: (%3.1f, %3.1f) at %3.1f deg", drive.pose.position.x, drive.pose.position.y,
                    Math.toDegrees(drive.pose.heading.toDouble())));
            double Target_Heading = Math.toRadians(detection.ftcPose.yaw) + drive.pose.heading.toDouble();
            RobotLog.i(String.format("Target: (%3.1f, %3.1f) at %3.1f deg", Target_X, Target_Y, Math.toDegrees(Target_Heading)));
            return new Pose2d(Target_X, Target_Y, Target_Heading);

        }*/

    }
