package org.firstinspires.ftc.teamcode.Auto.Paths;



import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class RedAutoPaths extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                //.strafeTo(new Vector2d(24.57, -25))
                //deposit specimen
                //.strafeTo(new Vector2d(24, 0))
                //intake sample
                .strafeToLinearHeading(new Vector2d(7.02, 86.07), Math.toRadians(-45))
                //deposit sample
                .strafeTo(new Vector2d(10, 0))
                .strafeToLinearHeading(new Vector2d(26.44, -7.89),Math.toRadians(-45))
                //intake sample

                //deposit sample
                .strafeToLinearHeading(new Vector2d(7.02, 86.07), Math.toRadians(-45))

                .strafeTo(new Vector2d(10, 0))//intake sample
                .strafeToLinearHeading(new Vector2d(25.02, -35.14),Math.toRadians(-45))
                .strafeTo(new Vector2d(10, 0))
                .strafeToLinearHeading(new Vector2d(7.02, 86.07), Math.toRadians(-45))
                //deposit sample

                .strafeTo(new Vector2d(10, 0));
        TrajectoryActionBuilder third = drive.actionBuilder( new Pose2d(10, 0, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(39.82, -24.03),Math.toRadians(-95))

                .strafeTo(new Vector2d(10, 0))

                .strafeToLinearHeading(new Vector2d(7.02, 86.07), Math.toRadians(-45));
                //intake sample

                //deposit sample
                //.build();
                Action path = basket.build();
                Action path2 = third.build();
                waitForStart();
                Actions.runBlocking(new SequentialAction(
                        path,
                        path2
                ));

    }
}