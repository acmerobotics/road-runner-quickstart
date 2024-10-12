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
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(5.24, 36.60), Math.toRadians(-45));
                //deposit sample
        TrajectoryActionBuilder block1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(26.49, 26.64), Math.toRadians(0));
                //intake sample
        TrajectoryActionBuilder block2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(25.13, 35.12), Math.toRadians(0));
                //intake sample
        TrajectoryActionBuilder block3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(46.5, 27.18), Math.toRadians(90));
                //intake sample



        Pose2d StartPose2 = new Pose2d(-24, 59.5, Math.toRadians(90));

        TrajectoryActionBuilder poormansauto = drive.actionBuilder(StartPose2)
                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(225));

        Action basketA = basket.build();
        Action basket2A = basket.build();
        Action basket3A = basket.build();
        Action basket4A = basket.build();
        Action block1A = block1.build();
        Action block2A = block2.build();
        Action block3A = block3.build();
        waitForStart();

        Actions.runBlocking(new SequentialAction(
                basketA,
                new SleepAction(1),
                block1A,
                new SleepAction(0.5),
                basket2A,
                new SleepAction(1),
                block2A,
                new SleepAction(0.5),
                basket3A,
                new SleepAction(1),
                block3A,
                new SleepAction(0.5),
                basket4A
        ));
    }
}

