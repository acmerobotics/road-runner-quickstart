package org.firstinspires.ftc.teamcode.az.sample;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous
public class LeftAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    SpecimenTool specimenTool = null;
    Arm arm = null;
    Slides slides = null;
    DistanceSensor distanceSensor;
    private MecanumDrive drive;
    private Pose2d beginPose;


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
        waitForStart();

    }



    public void runOpMode() throws InterruptedException {
        initAuto();

        Action specimenDropPos = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(21, -8), Math.toRadians(0))
                .build();

        Action prePark = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(23, 25), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, 22), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .build();
        Action levelOneAscent = drive.actionBuilder(drive.pose)
                .lineToYConstantHeading(19)
                .build();
        Action park = new ParallelAction(levelOneAscent,
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        arm.setArmPos(Arm.ArmPos.LEVEL_ONE_ASCENT);
                        slides.moveToPosition(Slides.SlidesPos.LEVEL_ONE_ASCENT);
                        return false;
                    }
                }
        );

        Actions.runBlocking(
                new SequentialAction(
                        specimenDropPos,
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                specimenTool.specimenHang();
                                sleep(1000);
                                return false;
                            }
                        },
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                specimenTool.resetAndWait();
                                sleep(1000);
                                return false;
                            }
                        },
                        prePark,
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                specimenTool.levelOneAscent();
                                sleep(1000);
                                return false;
                            }
                        }
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
