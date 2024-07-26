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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.lang.Math;

@Autonomous(name="AAABF", group="Linear OpMode")

@Disabled

public class ZBF extends LinearOpMode {


    int startPoseX = -36;
    int startPoseY = -68;
    int camera =1;
    //start off at 0
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "Blue.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public Action close() {
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
    public Action open() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);
                return false;
            }
        };

    }
    public Action Raise() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulder.setTargetPosition(440);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1);
                return false;
            }
        };

    }
    public Action Lower() {
        DcMotor shoulder;
        shoulder= hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
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
/*
    private void camera() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor.
        builder.addProcessor(tfod);

        visionPortal = builder.build();
        tfod.setZoom(1.5);

    }
*/


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(startPoseX,startPoseY,Math.PI/2));
        //camera();

        waitForStart();

        while(opModeIsActive()) {
/*
                    List<Recognition> currentRecognitions = tfod.getRecognitions();

                    for (Recognition recognition : currentRecognitions) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2;
                        double y = (recognition.getTop() + recognition.getBottom()) / 2;
                        if(x>=275 && x<=400) {
                            camera=3;
                            visionPortal.stopStreaming();
                        }
                        if(x>=175 && x<275) {
                            camera=2;
                            visionPortal.stopStreaming();
                        }
                        if(x>=0 && x<175) {
                            camera=1;
                            visionPortal.stopStreaming();
                        }
                    }

                    set camera according
                    currentRecognitions.size();

*/


            if (camera == 1) {
                Actions.runBlocking(drive.actionBuilder(new Pose2d(startPoseX, startPoseY, Math.PI/2))

                        .stopAndAdd(close())
                        .splineToLinearHeading(new Pose2d(48,-30,Math.PI), Math.PI)
                        //servo
                        .lineToX(-20)
                                .afterTime(2,Raise())
                                .afterTime(2,open())
                        .splineToLinearHeading(new Pose2d(-52,-40,Math.PI),Math.PI)
                                .afterTime(2,Lower())
                        .splineToLinearHeading(new Pose2d(-20,-30,Math.PI),Math.PI)
                        .lineToX(25)
                                .afterTime(3,close())
                        .splineToLinearHeading(new Pose2d(56,-36,0),0)
                        .lineToX(-24)
                                .afterTime(3,Raise())
                        .stopAndAdd(open())
                        .splineToLinearHeading(new Pose2d(-53,-41,Math.PI),Math.PI)
                                .afterTime(3,Lower())
                        .splineToLinearHeading(new Pose2d(-40,-60,Math.PI/2),Math.PI/2)

                        .build());
                requestOpModeStop();
            }

            if (camera == 2) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(startPoseX, startPoseY, Math.PI/2))
                                .stopAndAdd(close())
                        .splineToLinearHeading(new Pose2d(36,-5,Math.PI), Math.PI)
                        //servo
                        .lineToX(-20)
                        .afterTime(2,Raise())
                        .afterTime(2,open())
                        .splineToLinearHeading(new Pose2d(-52,-40,Math.PI),Math.PI)
                        .afterTime(2,Lower())
                        .splineToLinearHeading(new Pose2d(-20,-30,Math.PI),Math.PI)
                        .afterTime(2,close())
                        .splineToLinearHeading(new Pose2d(56,-36,0),0)
                        .lineToX(-24)
                        .afterTime(2,Raise())
                        .afterTime(2,open())
                        .splineToLinearHeading(new Pose2d(-53,-41,Math.PI),Math.PI)
                        .afterTime(2,Lower())
                        .splineToLinearHeading(new Pose2d(-40,-60,Math.PI),Math.PI)
                                        .build());
                requestOpModeStop();
            }

            if (camera == 3) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(startPoseX, startPoseY, Math.PI/2))
                        .stopAndAdd(close())
                        .splineToLinearHeading(new Pose2d(36,-36,Math.PI),
                                Math.PI)
                        .lineToX(-20)
                        //servo
                        .afterTime(2,Raise())
                        .afterTime(2,open())
                        .splineToLinearHeading(new Pose2d(-52,-40,Math.PI),Math.PI)
                        .afterTime(2,Lower())
                        .splineToLinearHeading(new Pose2d(-20,-30,Math.PI),Math.PI)
                        .lineToX(25)
                        .afterTime(2,close())
                        .splineToLinearHeading(new Pose2d(56,-36,0),0)
                        .lineToX(-24)
                        .afterTime(2,Raise())
                        .afterTime(2,open())
                        .splineToLinearHeading(new Pose2d(-53,-41,Math.PI),Math.PI)
                                .afterTime(2,Lower())
                        .splineToLinearHeading(new Pose2d(-40,-60,Math.PI),Math.PI)
                        .build());
                requestOpModeStop();
            }


        }


    }
}



