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
        Pose2d StartPose1 = new Pose2d(0, 8, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(25,-29.5), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15.17, 18.10), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(23.24,-48.10), Math.toRadians(-10))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15.17, 18.10), Math.toRadians(90))
                .waitSeconds(1)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15.17, 18.10), Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(8,-10));

        Action path = basket.build();
                waitForStart();
                Actions.runBlocking(path);

    }
}