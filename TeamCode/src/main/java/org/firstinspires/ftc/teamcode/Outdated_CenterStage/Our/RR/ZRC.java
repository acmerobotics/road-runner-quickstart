package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.RR;

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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.lang.Math;
import java.util.List;

@Autonomous(name="ZRC", group="Linear OpMode")

@Disabled

public class ZRC extends LinearOpMode {



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
                shoulder.setTargetPosition(-210);
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
                shoulder.setTargetPosition(-235);
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
                shoulder.setTargetPosition(-280);
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
                shoulder.setTargetPosition(-80);
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
                shoulder.setTargetPosition(-50);
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

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(16,-62,Math.PI/2));
        Camera();
        Servo servo;
        servo= hardwareMap.get(Servo.class, "servo");
        servo.scaleRange(0,1);
        servo.setPosition(1);


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
            if (camera == 0) {
                Actions.runBlocking(drive.actionBuilder(new Pose2d(16, -62, Math.PI/2))
                        .stopAndAdd(close1())
                        .splineToSplineHeading(new Pose2d(15,-25,0), 0)
                        .stopAndAdd(Raise1())
                        .splineToSplineHeading(new Pose2d(47,-31.7,0),3*Math.PI/2)
                                .stopAndAdd(open())
                        .splineToSplineHeading(new Pose2d(35,-59.2,Math.PI),Math.PI)
                                .stopAndAdd(Lower())
                        .lineToX(-30)
                                .afterTime(3,RaiseA())
                        .splineToSplineHeading(new Pose2d(-58,-35,Math.PI),Math.PI/2)
                                .stopAndAdd(close2())
                        .splineToSplineHeading(new Pose2d(-30,-58.5,0),0)
                                .stopAndAdd(Lower())
                        .lineToX(5)
                                .stopAndAdd(Raise2())
                        .splineToSplineHeading(new Pose2d(47,-43,0),3*Math.PI/2)
                                .stopAndAdd(open())
                        .splineToSplineHeading(new Pose2d(35,-59.2,Math.PI),Math.PI)
                                .stopAndAdd(Lower())
                        .lineToX(-30)
                                .stopAndAdd(RaiseB())
                        .splineToSplineHeading(new Pose2d(-58,-35,Math.PI),Math.PI/2)
                                .stopAndAdd(close2())
                        .splineToSplineHeading(new Pose2d(-30,-58.5,0),0)
                                .stopAndAdd(Lower())
                        .lineToX(5)
                                .stopAndAdd(Raise3())
                        .splineToSplineHeading(new Pose2d(47,-43,0),3*Math.PI/2)
                                .stopAndAdd(open())

                        .strafeTo(new Vector2d(40,-60))
                        .turnTo(Math.PI/2)
                        .build());
                requestOpModeStop();
            }
//middle +5
            else if (camera == 2) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(16, 62, Math.PI/2))

                        .splineToSplineHeading(new Pose2d(20,-30,7*Math.PI/4), 7*Math.PI/4)
                        .splineToSplineHeading(new Pose2d(48,-43,0),3*Math.PI/2)

                        .splineToSplineHeading(new Pose2d(0,-59.2,Math.PI),Math.PI)
                        .lineToX(-30)
                        .splineToSplineHeading(new Pose2d(-58,-35,Math.PI),Math.PI/2)
                        .splineToSplineHeading(new Pose2d(-27,-58.5,0),0)
                        .lineToX(0)
                        .splineToSplineHeading(new Pose2d(47,-43,0),3*Math.PI/2)

                        .splineToSplineHeading(new Pose2d(20,-59.2,0),Math.PI)
                        .lineToX(-30)

                        .splineToSplineHeading(new Pose2d(-50,-35,Math.PI),0)
                        .lineToX(0)
                        .splineToSplineHeading(new Pose2d(48,-43,0),0)

                        .lineToX(40)
                        .turnTo(Math.PI/2)
                        .build());
                requestOpModeStop();
            }
            else if (camera == 1) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(16, 62, Math.PI/2))
                        .splineToSplineHeading(new Pose2d(32,-26,0), Math.PI/2)
                        .splineToSplineHeading(new Pose2d(48,-40.5,0),Math.PI/2)

                        .splineToSplineHeading(new Pose2d(0,-59.2,Math.PI),Math.PI)
                        .lineToX(-30)
                        .splineToSplineHeading(new Pose2d(-58,-35,Math.PI),Math.PI/2)
                        .splineToSplineHeading(new Pose2d(-30,-58.5,0),0)
                        .lineToX(5)

                        .splineToLinearHeading(new Pose2d(20,-43,0),0)
                        //wait
                        .splineToLinearHeading(new Pose2d(30,-20,0),0)
                        .splineToSplineHeading(new Pose2d(47,-28,0),3*Math.PI/2)

                        .splineToLinearHeading(new Pose2d(0,-36,Math.PI),Math.PI)

                        .splineToSplineHeading(new Pose2d(0,-35.7,Math.PI),Math.PI)
                        .lineToX(-22)

                        .splineToSplineHeading(new Pose2d(-56,-36,Math.PI),Math.PI)
                        .lineToX(3)
                        .splineToSplineHeading(new Pose2d(48,-28,0),0)
                        .strafeTo(new Vector2d(40,-60))
                        .turnTo(Math.PI/2)
                        .build());

                requestOpModeStop();
            }

        }
    }


}




