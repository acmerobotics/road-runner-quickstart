package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.lang.Math;
import java.util.List;

@Autonomous(name="ZZRF", group="Linear OpMode")

@Disabled

public class ZZRF extends LinearOpMode {



    private static final String TFOD_MODEL_FILE = "Red.tflite";
    int camera =0;
    private static final boolean USE_WEBCAM = true;
    private static final String[] LABELS = {
            "Pixel",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public Action close1() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                return false;
            }
        };

    }
    public Action close2() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                sleep(100);
                return false;
            }
        };

    }
    public Action open() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);
                sleep(200);
                return false;
            }
        };

    }
    public Action Raise1() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-260);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action Raise2() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-285);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action Raise3() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-330);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action RaiseA() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-100);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action RaiseB() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-60);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                return false;
            }
        };

    }

    public Action Lower() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(0);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(.3);
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
                sleep(100);
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

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-37,-62,Math.PI/2));
        Camera();
        Servo servo;
        servo= hardwareMap.get(Servo.class, "servo");
        servo.scaleRange(0,1);
        servo.setPosition(1);
        waitForStart();

        while(opModeIsActive()) {

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
            if (camera == 0) {
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-37, -62, Math.PI/2))
                        .splineToLinearHeading(new Pose2d(-58,-35,Math.PI),Math.PI)
                        .lineToX(1.5)

                        .splineToSplineHeading(new Pose2d(47,-42.5,0),Math.PI/2)

                        .splineToSplineHeading(new Pose2d(0,-35,Math.PI),Math.PI)
                        .lineToX(-56)
                        .lineToX(1.5)
                        .splineToSplineHeading(new Pose2d(47,-42.5,0),Math.PI/2)

                        .splineToSplineHeading(new Pose2d(0,-35,Math.PI),Math.PI)
                        .lineToX(-56)
                        .lineToX(1.5)
                        .splineToSplineHeading(new Pose2d(47,-42.5,0),Math.PI/2)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .build());
                requestOpModeStop();
            }
//middle +5
            else if (camera == 2) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-37, 62, Math.PI/2))

                        .splineToLinearHeading(new Pose2d(-45,-30,5*Math.PI/4),Math.PI)

                        .splineToLinearHeading(new Pose2d(-58,-35,Math.PI),Math.PI)
                        .lineToX(1.5)

                        .splineToSplineHeading(new Pose2d(47,-35,0),Math.PI/2)

                        .splineToSplineHeading(new Pose2d(0,-35,Math.PI),Math.PI)
                        .lineToX(-56)
                        .lineToX(1.5)
                        .splineToSplineHeading(new Pose2d(47,-42.5,0),Math.PI/2)

                        .splineToSplineHeading(new Pose2d(0,-35,Math.PI),Math.PI)
                        .lineToX(-56)
                        .lineToX(1.5)
                        .splineToSplineHeading(new Pose2d(47,-42.5,0),Math.PI/2)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .build());
                requestOpModeStop();
            }
            else if (camera == 1) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-37, 62, Math.PI/2))
                        .splineToLinearHeading(new Pose2d(-32,-32,5*Math.PI/4),Math.PI)

                        .splineToLinearHeading(new Pose2d(-58,-35,Math.PI),Math.PI)
                        .lineToX(1.5)

                        .splineToSplineHeading(new Pose2d(47,-28.5,0),Math.PI/2)

                        .splineToLinearHeading(new Pose2d(0,-36,Math.PI),Math.PI)
                        .lineToX(-56)

                        .lineToX(1.5)
                        .splineToSplineHeading(new Pose2d(47,-28.5,0),Math.PI/2)

                        .splineToLinearHeading(new Pose2d(0,-36,Math.PI),Math.PI)
                        .lineToX(-56)

                        .lineToX(1.5)
                        .splineToSplineHeading(new Pose2d(47,-28.5,0),Math.PI/2)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .build());

                requestOpModeStop();
            }

        }
    }


}




