package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Autonomous;

/*
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Centerstage_Bromine.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.lang.Math;
import java.util.List;

@Autonomous(name="RF_5", group="Linear OpMode")

@Disabled
public class RF_5 extends LinearOpMode {



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
        Servo claw2;
        claw2= hardwareMap.get(Servo.class, "claw2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                claw2.setPosition(1);
                return false;
            }
        };

    }
    public Action close2() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        Servo claw2;
        claw2= hardwareMap.get(Servo.class, "claw2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                claw2.setPosition(1);
                sleep(100);
                return false;
            }
        };

    }
    public Action open() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        Servo claw2;
        claw2= hardwareMap.get(Servo.class, "claw2");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);
                claw2.setPosition(0);
                sleep(200);
                return false;
            }
        };

    }
    public Action Raise1() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-210);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist2.setPosition(.2);

                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action Raise2() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-255);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist2.setPosition(.3);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action Raise3() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-280);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist2.setPosition(.5);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action RaiseA() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-120);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist2.setPosition(.3);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action RaiseB() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-50);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist2.setPosition(.2);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action RaiseC() {

        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(-30);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(.3);
                return false;
            }
        };

    }

    public Action Lower() {

        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(0);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist2.setPosition(0);
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
    public Action Sleep() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sleep(7000);
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

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-38, -60,Math.PI/2));
        Camera();
        Servo servo;
        servo= hardwareMap.get(Servo.class, "servo");
        servo.scaleRange(0,1);
        servo.setPosition(1);



        Servo wrist2;
        wrist2= hardwareMap.get(Servo.class, "wrist2");
        wrist2.scaleRange(0,1);
        wrist2.setPosition(0);

        Servo wrist;
        wrist= hardwareMap.get(Servo.class, "wrist");
        wrist.scaleRange(0,1);
        wrist.setPosition(1);

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
            camera=0;
            if (camera == 0) {
                Actions.runBlocking(drive.actionBuilder(new Pose2d(-38, -60, Math.PI/2))
                        .strafeTo(new Vector2d(-37,-14))
                        .build());
                requestOpModeStop();
            }
//middle +5
            else if (camera == 2) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-37, 62, Math.PI/2))
                        //raisea
                        .splineToLinearHeading(new Pose2d(-45,-30,5*Math.PI/4),Math.PI)
                        //Servo
                        .splineToLinearHeading(new Pose2d(-58,-35,Math.PI),Math.PI)
                        //grab1
                        //wait-lower
                        .lineToX(1.5)
                        //raise1
                        .splineToSplineHeading(new Pose2d(47,-28,0),Math.PI/2)
                        //release1
                        .splineToSplineHeading(new Pose2d(0,-35,Math.PI),Math.PI)
                        //lower
                        .lineToX(-56)
                        //raiseb
                        //grab2
                        //wait-lower
                        .lineToX(1.5)
                        //raise2
                        .splineToSplineHeading(new Pose2d(47,-28,0),Math.PI/2)
                        //release2
                        .splineToSplineHeading(new Pose2d(0,-35,Math.PI),Math.PI)
                        //lower
                        .lineToX(-56)
                        //raisec
                        //grab3
                        //wait-lower
                        .lineToX(1.5)
                        //raise3
                        .splineToSplineHeading(new Pose2d(47,-43,0),0)
                        //release3
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .build());
                requestOpModeStop();
            }
            else if (camera == 1) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(-37, 62, Math.PI/2))
                        //raisea
                        .splineToLinearHeading(new Pose2d(-32,-32,5*Math.PI/4),Math.PI)
                        //Servo
                        .splineToLinearHeading(new Pose2d(-58,-35,Math.PI),Math.PI)
                        //grab1
                        //wait-lower
                        .lineToX(1.5)
                        //raise1
                        .splineToSplineHeading(new Pose2d(47,-43,0),Math.PI/2)
                        //release1
                        .splineToLinearHeading(new Pose2d(0,-36,Math.PI),Math.PI)
                        //lower
                        .lineToX(-56)
                        //raiseb
                        //grab2
                        //wait-lower
                        .lineToX(1.5)
                        //raise2
                        .splineToSplineHeading(new Pose2d(47,-28.5,0),Math.PI/2)
                        //release2
                        .splineToLinearHeading(new Pose2d(0,-36,Math.PI),Math.PI)
                        //lower
                        .lineToX(-56)
                        //raisec
                        //grab3
                        //wwait-lower
                        .lineToX(1.5)
                        //raise3
                        .splineToSplineHeading(new Pose2d(47,-28.5,0),Math.PI/2)
                        //release3
                        .setTangent(0)
                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .build());

                requestOpModeStop();
            }

        }
    }


}


 */



