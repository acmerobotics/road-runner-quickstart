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
public class BlueAutoPaths extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(40, 60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(40, 60, Math.toRadians(180));
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeTo(new Vector2d(0, 36))
                //deposit specimen
                .lineToX(10)
                .strafeToLinearHeading(new Vector2d(-53, -44), Math.toRadians(0))
                //intake sample
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(45))
                //deposit sample
                .strafeToLinearHeading(new Vector2d(-55, -44), Math.toRadians(0))
                //intake sample
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(45))
                //deposit sample
                .strafeToLinearHeading(new Vector2d(-50, -27), Math.toRadians(90))
                //intake sample
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(45));
                //deposit sample
                //.build();


        Pose2d StartPose2 = new Pose2d(20, -60, Math.toRadians(180));
        //drive.setPoseEstimate(StartPose2);
        
        TrajectoryActionBuilder speciman = drive.actionBuilder(StartPose2)
                .strafeTo(new Vector2d(5, -25))
                //Drop off specimen
                .lineToY(8)
                .strafeToLinearHeading(new Vector2d(45, -33), Math.toRadians(300))
                .lineToY(5)
                //intake block
                .lineToY(-5)
                .strafeToLinearHeading(new Vector2d(55, -68), Math.toRadians(270))
                //outtake block into human area
                .lineToY(-5)
                .waitSeconds(0.5)
                .lineToX(10)
                .strafeToLinearHeading(new Vector2d(48, -33), Math.toRadians(300))
                .lineToY(5)
                //intake 2nd sample
                .strafeToLinearHeading(new Vector2d(50, -68), Math.toRadians(270))
                //outtake block
                .lineToY(-5)
                .waitSeconds(0.5)
                .lineToX(0)
                .strafeToLinearHeading(new Vector2d(68, -33), Math.toRadians(300))
                .lineToY(5)
                //intake 3rd block
                .lineToY(-8)
                .strafeToLinearHeading(new Vector2d(45, -68), Math.toRadians(270));
                //outtake block
                //.build();

    }
}

