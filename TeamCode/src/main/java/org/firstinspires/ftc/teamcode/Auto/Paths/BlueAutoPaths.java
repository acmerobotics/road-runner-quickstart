package org.firstinspires.ftc.teamcode.Auto.Paths;



import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class BlueAutoPaths extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0,-8.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(5.74, 36.60), Math.toRadians(-45));
                //deposit sample
        TrajectoryActionBuilder block1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(24.72, 2.82), Math.toRadians(0));
                //intake sample
        TrajectoryActionBuilder block2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(25.86, 33.24), Math.toRadians(0));
                //intake sample
        TrajectoryActionBuilder block3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(48.57, 27.59), Math.toRadians(90))
                .afterDisp(0, new SleepAction(1.0));
                //intake sample
        TrajectoryActionBuilder park = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(62.49, 1));

        TrajectoryActionBuilder trajecotryegrig = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(5.74, 36.60), Math.toRadians(-45))
                .afterDisp(0, new SleepAction(1.0))
                .strafeToLinearHeading(new Vector2d(24.72, 30.02), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(5.74, 36.60), Math.toRadians(-45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(25.86, 40.74), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(5.74, 36.60), Math.toRadians(-45))
                .waitSeconds(1)
                .strafeTo(new Vector2d(5,20))
                .strafeToLinearHeading(new Vector2d(46.57, 27.59), Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(6.74, 35.60), Math.toRadians(-45))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(53.00, 4, Math.toRadians(-90)),-90);

        Pose2d StartPose2 = new Pose2d(-24, 59.5, Math.toRadians(90));



        Action basketA = basket.build();
        Action basket2A = basket.build();
        Action basket3A = basket.build();
        Action basket4A = basket.build();
        Action block1A = block1.build();
        Action block2A = block2.build();
        Action block3A = block3.build();
        Action parkA = park.build();
        Action auto = trajecotryegrig.build();
        waitForStart();

//        Actions.runBlocking(new SequentialAction(
//                basketA,
//                new SleepAction(1),
//                block1A,
//                new SleepAction(0.5),
//                basket2A,
//                new SleepAction(1),
//                block2A,
//                new SleepAction(0.5),
//                basket3A,
//                new SleepAction(1),
//                block3A,
//                new SleepAction(0.5),
//                basket4A,
//                new SleepAction(1),
//                parkA
//        ));
        Actions.runBlocking(auto);
    }
}

