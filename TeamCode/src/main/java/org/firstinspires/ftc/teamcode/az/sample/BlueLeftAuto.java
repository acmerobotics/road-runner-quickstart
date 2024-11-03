//package org.firstinspires.ftc.teamcode.az.sample;
//
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Trajectory;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.acmerobotics.roadrunner.SequentialAction;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//@Config
//@Autonomous
//
//public class BlueLeftAuto extends LinearOpMode {
//    ElapsedTime runtime = new ElapsedTime();
//
//    SpecimenTool specimenTool = null;
//    Arm arm = null;
//    Slides slides = null;
//
//    Pose2d beginPose;
//    Trajectory pos1purpleDrop, pos2stack, pos1goToBackDrop, pos2ReadyToDrop, pos2stack2, pos2ReadyToDrop2, pos2Park;
//
//
//    public void initAuto() {
//
//        arm = new Arm(this);
//        slides = new Slides(this);
//        specimenTool = new SpecimenTool(this);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        runtime.reset();
//        waitForStart();
//    }
//
//
//
//    public void runOpMode() throws InterruptedException {
//        Pose2d beginPose = new Pose2d(0,0,Math.toRadians(0));
////        HardwareMap hardwareMap1 = new HardwareMap();
//        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//
//        initAuto();
//
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .strafeTo(new Pose2d(10,10, Math.toRadians(0))).waitSeconds(2).build()
//        );
//
//
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(beginPose)
//                .strafeTo(new Pose2d(10,10, Math.toRadians(0)))
//                .waitSeconds(2);
//
//
//
//    }
//
//}
