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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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
    ////
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    TfodProcessor tfod;
    final int RESOLUTION_WIDTH = 960;
    final int RESOLUTION_HEIGHT = 720;
    public static double DRONE_PUSHER_RESET = 0.85;
    public static double DRONE_PUSHER_INIT = 0.8;
    public static double GRABBER_ROT_INIT = 0.07;
    public static double GRABBER_UP = 0.3;
    public static double GRABBER_DOWN = GRABBER_ROT_INIT;
    public static double FINGER_UP = 0;
    public static double FINGER_DOWN = .4;
    public static double FINGER_INIT = FINGER_DOWN;

    public static double HANGER_LATCH_INIT = 0.87;

    public static double PURPLE_RELEASE = 0;
    public static double PURPLE_GRAB = 0;
    String side = "Red";

    String[] RED_LABELS = {"redprop"};
    String[] BLUE_LABELS = {"blueprop"};

    public AutoFluffy(LinearOpMode op) {
        this.op = op;
        this.init();
    }

    public AutoFluffy(LinearOpMode op, String side) {
        this.op = op;
        this.side = side;
        this.init();

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




        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
        String[] LABELS;
        if (side.equals("Red")) {
            LABELS = RED_LABELS;
        } else {
            LABELS = BLUE_LABELS;
        }
        tfod = new TfodProcessor.Builder()
                .setModelFileName("model_20231209_112710.tflite")
                .setModelAspectRatio(RESOLUTION_HEIGHT/RESOLUTION_WIDTH)  //verify with grace
                .setModelLabels(LABELS)
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------

        drive = new MecanumDrive(op.hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        visionPortal = new VisionPortal.Builder()
                .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(tfod, aprilTag)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();

    }

    List<AprilTagDetection> findDetections() {
        return aprilTag.getDetections();
    }

    public void deliverPurple() {
        leftPurple.setPosition(PURPLE_RELEASE);
        rightPurple.setPosition(PURPLE_RELEASE);
        op.sleep(1000);
        leftPurple.setPosition(PURPLE_GRAB);
        rightPurple.setPosition(PURPLE_GRAB);
    }
    public List<Recognition> getRecognitions(){
        return tfod.getRecognitions();
    }
}