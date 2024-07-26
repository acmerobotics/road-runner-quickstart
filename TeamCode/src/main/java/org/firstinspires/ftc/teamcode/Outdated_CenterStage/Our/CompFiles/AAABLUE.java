package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.CompFiles;

/*
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Centerstage_Bromine.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.lang.Math;
import java.util.List;

@Autonomous(name="AAABLUE", group="Linear OpMode")

public class AAABLUE extends LinearOpMode {

    int camera = 1;
    //start off at 0
    private static final String TFOD_MODEL_FILE = "Blue2.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTag;
    double y;
    double x;
    double yaw = 0;
    double boarddistance = 12;


    public Action RightClaw() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        Servo claw2;
        claw2= hardwareMap.get(Servo.class, "claw2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw2.setPosition(.4);
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
                claw2.setPosition(1);
                claw.setPosition(.4);
                return false;
            }
        };

    }
    public Action Open() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        Servo claw2;
        claw2= hardwareMap.get(Servo.class, "claw2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(.4);
                return false;
            }
        };

    }
    public Action Close() {
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
                wrist2.setPosition(.24);

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
                shoulder.setTargetPosition(-187);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                wrist2.setPosition(.3279);
                return false;
            }
        };

    }
    public Action RaiseB() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-150);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                wrist2.setPosition(.305);
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
                wrist2.setPosition(0);
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
    private void camera() {

        // TensorFlow builder
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(tfod);

        visionPortal = builder.build();
    }

    public Action AprilYaw (int id){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElapsedTime timer = new ElapsedTime();
                visionPortal.resumeStreaming();
                while (timer.seconds() < 0.5){
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((CameraOP()) && (id == detection.id)) {
                            yaw = (detection.ftcPose.yaw) * Math.PI/180;
                        }
                    }
                }
                visionPortal.stopStreaming();
                return false;
            }
        };
    }
    public Action AprilXY (int id){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElapsedTime timer = new ElapsedTime();
                visionPortal.resumeStreaming();
                while (timer.seconds() < 0.5){
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((CameraOP()) && (id == detection.id)) {
                            x = detection.ftcPose.x;
                            y = detection.ftcPose.y-boarddistance;
                     }
                    }
                }
                visionPortal.stopStreaming();
                return false;
            }
        };
    }
    private boolean CameraOP () {
        visionPortal.resumeStreaming();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        return (currentDetections != null) && (currentDetections.size() !=0);
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


    private void INit() {

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
        claw2.setPosition(.4);

        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(2000);
        wrist2.setPosition(0);

    }

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-38, 64,3*Math.PI/2));

        camera();
        INit();
/*
        OpenCVB cv = new OpenCVB();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .addProcessor(cv)
                .build();

        while (opModeInInit()) {
            cv.getPropLoc();
        }




        waitForStart();
        while(opModeIsActive()) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();

            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                if(x>=320 && x<=640) {
                    camera=3;

                    visionPortal.stopStreaming();
                }
                if(x >= 70 && x < 320) {
                    camera=2;

                    visionPortal.stopStreaming();
                }
            }
            currentRecognitions.size();

            sleep(3000);
            visionPortal.stopStreaming();

            if (camera==1) {

                Actions.runBlocking(drive.mirroredActionBuilder(new Pose2d(-38, -64, Math.PI/2))

                        .splineToLinearHeading(new Pose2d(-30,-33,Math.PI),0)
                        .stopAndAdd(Servo())
                        .stopAndAdd(Open())
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(-51,-35))
                        .stopAndAdd(RaiseA())
                        .waitSeconds(2.2)
                        .stopAndAdd(Close())
                                .waitSeconds(.4)
                        .lineToX(-45)
                        .strafeTo(new Vector2d(-36,-58))
                        .stopAndAdd(Lower())
                        .waitSeconds(.5)
                        .turnTo(0)
                        .lineToX(-35)
                        .waitSeconds(.7)
                        .lineToX(10)
                        .waitSeconds(2)
                        .lineToX(28)
                        .splineToLinearHeading(new Pose2d(40,-37,0),0)
                        .stopAndAdd(Raise1())
                        .lineToX(47.5)
                        .stopAndAdd(Release())
                        .waitSeconds(1.2)
                        .lineToX(41)
                        .turnTo(Math.PI/2)
                        .stopAndAdd(Lower())
                        .waitSeconds(1.5)
                        .build());
                requestOpModeStop();
            }
            else if (camera==2) {

                Actions.runBlocking(drive.mirroredActionBuilder(new Pose2d(-38, -64, Math.PI/2))
                        .splineToLinearHeading(new Pose2d(-38,-20.5,Math.PI),0)
                        .stopAndAdd(Servo())
                                .stopAndAdd(Open())
                        .strafeTo(new Vector2d(-51,-35))
                        .stopAndAdd(RaiseA())
                        .waitSeconds(2.2)
                        .stopAndAdd(Close())
                        .waitSeconds(.4)
                        .lineToX(-45)
                        .strafeTo(new Vector2d(-36,-58))
                        .stopAndAdd(Lower())
                        .waitSeconds(.5)
                        .turnTo(0)
                        .lineToX(-35)
                        .waitSeconds(.7)
                        .lineToX(10)
                        .waitSeconds(2)
                        .lineToX(28)
                        .splineToLinearHeading(new Pose2d(40,-34,0),0)
                        .stopAndAdd(Raise1())
                        .lineToX(47.5)
                        .stopAndAdd(Release())
                        .waitSeconds(1.5)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .stopAndAdd(Lower())
                                .waitSeconds(2)
                        .build());
                requestOpModeStop();
            }
            else if (camera==3) {

                Actions.runBlocking(drive.mirroredActionBuilder(new Pose2d(-38, -64, Math.PI/2))

                        .splineToLinearHeading(new Pose2d(-39,-36,0),Math.PI)
                        .stopAndAdd(Servo())
                        .waitSeconds(1)
                        .setTangent(Math.PI/2)
                        .lineToY(-58.3)
                        .setTangent(Math.PI)
                        .lineToX(-32)
                        .turnTo(0)
                        .lineToX(17)
                        .waitSeconds(2)
                        .setTangent(Math.PI/2)
                        .lineToY(-35)
                        .waitSeconds(6)
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(40,-29.3,0),0)
                        .stopAndAdd(Raise1())
                        .lineToX(46)
                        .waitSeconds(1.5)
                        .stopAndAdd(Release())
                        .waitSeconds(1)
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


