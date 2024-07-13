package org.firstinspires.ftc.teamcode.teamCode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;


@Autonomous
public class AutoPisu extends LinearOpMode {
    SampleMecanumDrive drive;
    ArmController arm;
    LiftController lift;
    JointController joint;
    ClawController claw;


    //public Robotel robotel;

    @Override
    public void runOpMode() throws InterruptedException {{
                drive = new SampleMecanumDrive(hardwareMap);
                arm = new ArmController (hardwareMap);
                lift = new LiftController (hardwareMap);
                joint = new JointController(hardwareMap);
                claw = new ClawController (hardwareMap);

                Pose2d START_POSE = new Pose2d(14, 65, Math.toRadians(-90));
                arm.goMid();
                joint.goToUp();
                claw.toggleLeft();
                claw.toggleRight();

                waitForStart();
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(START_POSE)
                            .lineToLinearHeading(new Pose2d(13, 45, Math.toRadians(-120)))//pune pixel pe right


                            .lineToLinearHeading(new Pose2d(20, 50, Math.toRadians(0))) //rotire
                            .lineToConstantHeading(new Vector2d(46, 36)) //merge la backdrop



                            .lineToLinearHeading(new Pose2d(23, 11, Math.toRadians(180))) //rotire+diag

                            .lineToConstantHeading(new Vector2d(-53, 11)) //trece pe sub door si aj la stack





                            .lineToConstantHeading(new Vector2d(23, 11)) //pleaca de la stack
                            .lineToLinearHeading(new Pose2d(46, 36, Math.toRadians(0))) //ajunge la backdrop

                        .build()
                );



            }}}

