package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Autonomous;
/*
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Centerstage_Bromine.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="RF_1AT", group="Linear OpMode")

@Disabled
public class RF_1AT extends LinearOpMode {



    private static final String TFOD_MODEL_FILE = "Red.tflite";
    int camera =0;
    private static final boolean USE_WEBCAM = true;
    private static final String[] LABELS = {
            "Pixel",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    double y = 8;
    double x = 0;
    double yaw = 0;
    double boardoffsety = 20;
    double stackoffsety = 10;
    boolean Detection = false;
    double boardoffsetx = 20;
    double stackoffsetx = 10;


    public Action RightClaw() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        Servo claw2;
        claw2= hardwareMap.get(Servo.class, "claw2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                return false;
            }
        };

    }
    public Action Release() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        Servo claw2;
        claw2= hardwareMap.get(Servo.class, "claw2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(.4);
                claw2.setPosition(.4);
                return false;
            }
        };

    }

    public Action Raise1() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                shoulder.setTargetPosition(-350);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(.95);
                wrist2.setPosition(.216);

                return false;
            }
        };

    }
    public Action Raise2() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-340);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                wrist2.setPosition(.218);
                return false;
            }
        };

    }

    public Action RaiseA() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-212);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                    wrist2.setPosition(.326);
                return false;
            }
        };

    }
    public Action RaiseD() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-212);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                wrist2.setPosition(0);
                return false;
            }
        };

    }

    public Action Wrist2() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist2.setPosition(.3268);
                return false;
            }
        };

    }

    public Action Lower() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(0);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(.3);
                wrist2.setPosition(0);
                return false;
            }
        };

    }

    public Action Wrist() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist2.setPosition(.1);
                return false;
            }
        };

    }
    public Action Servo() {
        Servo servo;
        servo= hardwareMap.get(Servo.class, "servo");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servo.setPosition(0);
                return false;
            }
        };

    }
    public Action AprilYaw (int id){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElapsedTime timer = new ElapsedTime();
                while (timer.seconds() < 0.5){
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    CameraOP();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((Detection) && (id == detection.id)) {
                            yaw = (detection.ftcPose.yaw) * Math.PI/180;
                        }
                    }
                }
                return false;
            }
        };
    }
    public Action AprilXY (int id, boolean atboard){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElapsedTime timer = new ElapsedTime();
                while (timer.seconds() < 0.5){
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    CameraOP();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((Detection) && (id == detection.id)) {
                            if(atboard) {
                                y = detection.ftcPose.y - boardoffsety;
                                x = detection.ftcPose.x - boardoffsetx;
                            }
                            else {
                                y = detection.ftcPose.y - stackoffsety;
                                x = detection.ftcPose.x - stackoffsetx;
                            }
                        }
                    }
                }
                return false;
            }
        };
    }

    private void Camera() {


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)


                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();


        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        visionPortal = builder.build();


    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //builder.setCamera(BuiltinCameraDirection.BACK);

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void CameraOP () {
        visionPortal.resumeStreaming();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        Detection = (currentDetections != null) && (currentDetections.size() != 0);
    }
    private void INit() {
        Camera();
        initAprilTag();

        sleep(5000);


        Servo servo;
        servo= hardwareMap.get(Servo.class, "servo");
        servo.scaleRange(0,1);
        servo.setPosition(.5);

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        wrist2.scaleRange(0,1);
        wrist2.setPosition(.216);

        Servo claw2;
        claw2 = hardwareMap.get(Servo.class,"claw2");
        claw2.scaleRange(0,1);

        Servo claw;
        claw = hardwareMap.get(Servo.class,"claw");
        claw.scaleRange(0, 1);
        claw.setPosition(.4);
        claw2.setPosition(0);

        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(2000);
        wrist2.setPosition(0);
    }

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-34.5,-62,Math.PI));

        INit();


        waitForStart();

        while(opModeIsActive()) {
sleep(3000);
            List<Recognition> currentRecognitions = tfod.getRecognitions();

            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                if(x>=0 && x<320) {
                    camera=2;
                    visionPortal.stopStreaming();
                }
                if(x>=320 && x<=640) {
                    camera=1;
                    visionPortal.stopStreaming();
                }


            }
            currentRecognitions.size();
            visionPortal.stopStreaming();


//left
                        //.splineToLinearHeading(new Pose2d(-53,-32.2,Math.PI),Math.PI)
            if (camera == 0) {
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-34.5, -63, Math.PI))
                        .splineToLinearHeading(new Pose2d(-53.5,-28,Math.PI),Math.PI)
                        .waitSeconds(.5)
                        .stopAndAdd(Servo())
                                .setTangent(Math.PI/2)
                                .lineToY(-39.6)
                                .setTangent(Math.PI)
                                .lineToX(-51.7)
                                .stopAndAdd(AprilYaw(8))
                                .splineToLinearHeading(new Pose2d(-53.5,-28,Math.PI+yaw),Math.PI+yaw)
                                .stopAndAdd(AprilXY(8, false))
                                .stopAndAdd(RaiseD())
                                .stopAndAdd(Wrist2())
                                .splineToLinearHeading(new Pose2d(-53.5+y,-28+x,Math.PI),Math.PI)
                        .waitSeconds(3)
                        .stopAndAdd(RightClaw())
                                .waitSeconds(1.5)
                        .stopAndAdd(Wrist())
                        .waitSeconds(.5)
                        .setTangent(Math.PI/2)
                        .lineToY(-58.6)
                        .setTangent(Math.PI)
                        .lineToX(-13)
                        .stopAndAdd(Lower())
                        .lineToX(18)
                        .turnTo(Math.PI/2)
                        .waitSeconds(4.5)
                        .stopAndAdd(Raise1())
                        .splineToSplineHeading(new Pose2d(40,-31.7,0),0)
                        .stopAndAdd(AprilYaw(4))
                        .splineToSplineHeading(new Pose2d(40,-31.7,0+yaw),0+yaw)
                        .stopAndAdd(AprilXY(4, true))
                        .splineToSplineHeading(new Pose2d(40,-31.7+x,0+yaw),0+yaw)
                        .lineToX(40+y)
                        .waitSeconds(1)
                        .stopAndAdd(Release())
                        .waitSeconds(.5)
                        .setTangent(0+yaw)
                        .lineToX(39)
                        .turnTo(Math.PI/2)
                                .stopAndAdd(Lower())
                        .build());
                requestOpModeStop();
            }
//middle +5
            else if (camera == 2) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-34.5, -63, Math.PI))
                                .stopAndAdd(RaiseA())
                        .splineToLinearHeading(new Pose2d(-39,-29.2,5*Math.PI/4),0)
                        .stopAndAdd(Servo())
                                .waitSeconds(.5)
                        .splineToLinearHeading(new Pose2d(-53,-39.4,Math.PI),Math.PI)
                                .waitSeconds(1.3)
                        .stopAndAdd(RightClaw())
                        .waitSeconds(1.7)
                        .stopAndAdd(Wrist())
                        .setTangent(Math.PI/2)
                        .lineToY(-58)
                        .setTangent(Math.PI)
                        .lineToX(-13)
                        .stopAndAdd(Lower())
                        .lineToX(18)
                        .turnTo(Math.PI/2)
                        .waitSeconds(6)
                        .stopAndAdd(Raise1())
                        .splineToSplineHeading(new Pose2d(40,-36,0),0)
                        .lineToX(47)
                        .waitSeconds(1)
                        .stopAndAdd(Release())
                        .waitSeconds(.5)
                        .setTangent(0)
                        .lineToX(39)
                        .stopAndAdd(Lower())
                        .build());
                requestOpModeStop();
            }
            else if (camera == 1) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-34.5, -63, Math.PI))
                        .stopAndAdd(RaiseA())
                        .splineToLinearHeading(new Pose2d(-31,-32,5*Math.PI/4),0)
                        .stopAndAdd(Servo())
                                .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(-52.6,-39.4,Math.PI),Math.PI)
                                .waitSeconds(.8)
                        .stopAndAdd(RightClaw())
                        .waitSeconds(1.3)
                        .stopAndAdd(Wrist())
                        .setTangent(Math.PI/2)
                        .lineToY(-58.6)
                        .setTangent(Math.PI)
                        .lineToX(-13)
                        .stopAndAdd(Lower())
                        .lineToX(18)
                        .turnTo(Math.PI/2)
                        .waitSeconds(6)
                        .stopAndAdd(Raise1())
                        .strafeTo(new Vector2d(40,-42.5))
                        .turnTo(0)
                        .lineToX(46)
                        .waitSeconds(1)
                        .stopAndAdd(Release())
                        .waitSeconds(.5)
                        .setTangent(0)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                                .stopAndAdd(Lower())
                        .build());

                requestOpModeStop();
            }

        }
    }


}




 */

