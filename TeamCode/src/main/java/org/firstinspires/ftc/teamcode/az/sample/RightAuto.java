package org.firstinspires.ftc.teamcode.az.sample;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous
public class RightAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    SpecimenTool specimenTool = null;
    Arm arm = null;
    Slides slides = null;
    Gripper gripper = null;
    DistanceSensor distanceSensor;
    private MecanumDrive drive;
    private Pose2d beginPose;
    private Action specimenDropPos;
    private Action parkPos;
    private Action specimenToolDrop;
    private Action armDrop;
    private Action slidesReset;
    private Action readyToDropSpecimen;
    private Action park;
    private Action armGripperReset;


    public class specimenHang implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            specimenTool.specimenHang();
            sleep(1000);



            return false;
        }
    }

    public Action specimenHang() {
        return new specimenHang();
    }

    public void initAuto() {

        arm = new Arm(this);
        slides = new Slides(this);
        gripper = new Gripper(this);
        specimenTool = new SpecimenTool(this);
        specimenTool.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
        beginPose = new Pose2d(0,0,Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, beginPose);
        telemetry.addData("current position", drive.pose);
        telemetry.update();
        createActions();
        waitForStart();
    }

    public void createActions() {
        specimenDropPos = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(21, 8), Math.toRadians(0))
                .build();
        parkPos = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(3, -36), Math.toRadians(0))
                .build();
        specimenToolDrop = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                new ParallelAction(
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                arm.specimenHang();
                                return false;
                            }
                        },
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                slides.specimenHang();
                                return false;
                            }
                        },
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                gripper.specimenHang();
                                return false;
                            }
                        }
                );
                return false;
            }
        };
        readyToDropSpecimen = new ParallelAction(specimenToolDrop, specimenDropPos);
        armDrop = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.specimenDrop();
                return false;
            }
        };
        slidesReset = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slides.reset();
                return false;
            }
        };
        armGripperReset = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.reset();
                gripper.reset();
                return false;
            }
        };
        park = new ParallelAction(armGripperReset, parkPos);
    }


    public void runOpMode() throws InterruptedException {
        initAuto();

        Actions.runBlocking(
                new SequentialAction(
                        readyToDropSpecimen,
                        new SleepAction(1),
                        armDrop,
                        new SleepAction(1),
                        slidesReset,
                        new SleepAction(1),
                        park
                )
        );
//
        sleep(5000);
        telemetry.addData("current position",drive.pose);
        telemetry.update();
        sleep(10000);



    }

}
