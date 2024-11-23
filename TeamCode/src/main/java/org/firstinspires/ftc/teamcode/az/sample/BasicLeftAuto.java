package org.firstinspires.ftc.teamcode.az.sample;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous ( preselectTeleOp = "IntoTheDeepTeleOp")
public class BasicLeftAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    SpecimenTool specimenTool = null;
    Arm arm = null;
    Slides slides = null;
    DistanceSensor distanceSensor;
    MecanumDrive drive;
    private Pose2d beginPose;
    Action specimenDropPos;
    Action prePark;
    Action specimenHang;
    Action specimenToolWait;
    Action levelOneAscent;


    public void initAuto() {

        arm = new Arm(this);
        slides = new Slides(this);
        specimenTool = new SpecimenTool(this);
        specimenTool.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
        beginPose = new Pose2d(0,0,Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, beginPose);
        telemetry.addData("current position", drive.pose);
        telemetry.update();
        setUpActions();
        waitForStart();

    }

    private void setUpActions(){
        specimenDropPos = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(19, 0), Math.toRadians(0))
//                .afterDisp(1, specimenHang)
                .build();

        prePark = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(18, 25), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(50, 19), Math.toRadians(-90))
                .splineTo(new Vector2d(50, 19), Math.toRadians(-90))
                .build();

        specimenHang = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.specimenHang();
                sleep(1000);
                return false;
            }
        };
        specimenToolWait = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.resetAndWait();
//                sleep(1000);
                return false;
            }
        };
        levelOneAscent = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.levelOneAscent();
                sleep(1000);
                return false;
            }
        };
    }

    public void runOpMode() throws InterruptedException {
        initAuto();

        Actions.runBlocking(
                new SequentialAction(
                        specimenDropPos,
                        specimenHang,
                        specimenToolWait,
                        prePark,
                        levelOneAscent
                        //levelOneAscent
                )
        );
//
        sleep(5000);
        telemetry.addData("current position",drive.pose);
        telemetry.update();
        sleep(15000);



    }

}
