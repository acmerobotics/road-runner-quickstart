package org.firstinspires.ftc.teamcode.teamCode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;


@Autonomous
public class AutoPisu extends LinearOpMode {
    SampleMecanumDrive drive;
    Robot robot;


    //public Robotel robotel;

    @Override
    public void runOpMode() throws InterruptedException {{
                drive = new SampleMecanumDrive(hardwareMap);
                Pose2d START_POSE = new Pose2d(14, 65, Math.toRadians(-90));
                robot = new Robot(this);
                robot.start();
                robot.arm.goUp();
                robot.joint.goToUp();
                robot.claw.toggleLeft();
                robot.claw.toggleRight();
                drive.setPoseEstimate(START_POSE);
                waitForStart();
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(new Pose2d(14,64, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(13, 45, Math.toRadians(-120)))//pune pixel pe right
                                .addTemporalMarker(() -> {
                                    robot.claw.toggleRight();
                                })
                                .waitSeconds(0)
                                .waitSeconds(0.5)
                                .addTemporalMarker(() ->{
                                    robot.arm.goMid();
                                })
                                .waitSeconds(0)
                                .lineToConstantHeading(new Vector2d(13, 50)) //da cu spatele ig
                        .lineToLinearHeading(new Pose2d(20, 45, Math.toRadians(0))) //rotire
                        .lineToConstantHeading(new Vector2d(46, 30)) //merge la backdrop
                                .addTemporalMarker(() -> {
                                            robot.lift.goMid();
                                            robot.joint.goToUp();
                                        })
                                        .waitSeconds(0)
                                        .waitSeconds(0.02)
                                                .addTemporalMarker(() -> {
                                                 robot.claw.toggleLeft();
                                                })
                                .waitSeconds(0)
                                .waitSeconds(0.02)
                                .addTemporalMarker(() -> {
                                    robot.lift.goDown();
                                })
                                .waitSeconds(0)

                        .build()
                );
        if(isStopRequested()) {
            robot.interrupt();
            stop();
        }
        robot.interrupt();
            }}}

