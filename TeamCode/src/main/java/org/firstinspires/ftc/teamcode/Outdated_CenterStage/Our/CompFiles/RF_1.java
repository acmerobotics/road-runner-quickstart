package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.CompFiles;

import android.util.Size;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math;

@Autonomous(name="RF_1", group="Linear OpMode")

@Disabled
public class RF_1 extends LinearOpMode {

    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;
    VisionPortal visionPortal;

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
    /*
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

    */
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
        claw.setPosition(.4);
        claw2.setPosition(1);

        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(2000);
        //wrist2.setPosition(0);

    }

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-38, -63.4,Math.PI/2));

        INit();

        OpenCV cv = new OpenCV();

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

            if (cv.propLoc== OpenCV.PropLoc.Left) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-38, -64, Math.PI/2))

                        .splineToLinearHeading(new Pose2d(-35,-25,0),0)
                        //.stopAndAdd(Servo())
                        .waitSeconds(1.5)
                        .lineToX(-33)
                        .strafeTo(new Vector2d(-36,-58))
                        .setTangent(Math.PI)
                        .lineToX(20)
                        .splineToLinearHeading(new Pose2d(40,-30,0),0)
                        //.stopAndAdd(Raise1())
                        .lineToX(47.3)
                        .waitSeconds(1.5)
                        //.stopAndAdd(Release())
                        .lineToX(38)
                        //.stopAndAdd(Lower())
                        .splineToLinearHeading(new Pose2d(15,-58,Math.PI),Math.PI)
                        .lineToX(-40)
                        //.stopAndAdd(RaiseA())
                        .splineToLinearHeading(new Pose2d(-52.6,-39.4,Math.PI),Math.PI)
                        //.stopAndAdd(RightClaw())
                        .waitSeconds(.8)
                        //.stopAndAdd(Wrist())
                        .waitSeconds(.8)
                        .strafeTo(new Vector2d(-36,-58))
                        //.stopAndAdd(Lower())
                        .setTangent(Math.PI)
                        .lineToX(20)
                        .splineToLinearHeading(new Pose2d(40,-35,0),0)
                        //.stopAndAdd(Raise1())
                        .lineToX(47)
                        //.stopAndAdd(Release())
                        .waitSeconds(1)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        //.stopAndAdd(Lower())
                        .build());
                requestOpModeStop();
            }
            else if (cv.propLoc== OpenCV.PropLoc.Middle) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-38, -64, Math.PI/2))

                        .stopAndAdd(RaiseA())
                        .splineToLinearHeading(new Pose2d(-45,-29.2,5*Math.PI/4),0)
                        .stopAndAdd(Servo())
                        .waitSeconds(.5)
                        .splineToLinearHeading(new Pose2d(-53,-39.4,Math.PI),Math.PI)
                        .stopAndAdd(RightClaw())
                        .waitSeconds(.8)
                        .stopAndAdd(Wrist())
                        .waitSeconds(.8)
                        .lineToX(-48)
                        .turnTo(0)
                        .stopAndAdd(Lower())
                        .splineToLinearHeading(new Pose2d(-24,-35,0),0)
                        .lineToX(40)
                        .waitSeconds(.5)
                        .stopAndAdd(Raise1())
                        .lineToX(47)
                        .stopAndAdd(Release())
                        .waitSeconds(1.5)
                        .lineToX(36)
                        .turnTo(Math.PI)
                        .stopAndAdd(Lower())
                        .lineToX(-45)
                        .splineToLinearHeading(new Pose2d(-53,-39.4,Math.PI),Math.PI)
                        .stopAndAdd(RightClaw())
                        .waitSeconds(.8)
                        .stopAndAdd(Wrist())
                        .waitSeconds(.8)
                        .lineToX(-48)
                        .turnTo(0)
                        .stopAndAdd(Lower())
                        .splineToLinearHeading(new Pose2d(-24,-35,0),0)
                        .lineToX(40)

                        .waitSeconds(.5)
                        .stopAndAdd(Raise1())
                        .lineToX(47)
                        .stopAndAdd(Release())
                        .waitSeconds(1.5)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .stopAndAdd(Lower())
                        .build());
                requestOpModeStop();
            }
            else if (cv.propLoc== OpenCV.PropLoc.Right) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-38, -64, Math.PI/2))
                        .splineToLinearHeading(new Pose2d(-31,-33,7*Math.PI/6),0)
                        //.stopAndAdd(Servo())
                        .lineToX(-38)
                        .splineToLinearHeading(new Pose2d(-53.5,-39.4,Math.PI),Math.PI)
                        //.stopAndAdd(RightClaw())
                        .waitSeconds(.8)
                        //.stopAndAdd(Wrist())
                        .waitSeconds(.8)
                        .strafeTo(new Vector2d(-36,-58))
                        //.stopAndAdd(Lower())
                        .turnTo(0)
                        .lineToX(30)
                        .splineToLinearHeading(new Pose2d(40,-42,0),0)
                        .waitSeconds(.5)
                        //.stopAndAdd(Raise1())
                        .lineToX(47)
                        //.stopAndAdd(Release())
                        .waitSeconds(1.5)
                        .lineToX(36)
                        .turnTo(Math.PI)
                        //.stopAndAdd(Lower())
                        .strafeTo(new Vector2d(20,-58))
                        .setTangent(Math.PI)
                        .lineToX(-33)
                        .splineToLinearHeading(new Pose2d(-53,-39.4,Math.PI),Math.PI)
                        //.stopAndAdd(RightClaw())
                        .waitSeconds(.8)
                        //.stopAndAdd(Wrist())
                        .waitSeconds(.8)
                        .lineToX(-48)
                        .turnTo(0)
                        .strafeTo(new Vector2d(-36,-58))
                        .setTangent(Math.PI)
                        .lineToX(30)
                        .splineToLinearHeading(new Pose2d(40,-42,0),0)
                        .waitSeconds(.5)
                        //.stopAndAdd(Raise1())
                        .lineToX(47)
                        //.stopAndAdd(Release())
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        //.stopAndAdd(Lower())
                        .build());

                requestOpModeStop();
            }

        }
    }


}




