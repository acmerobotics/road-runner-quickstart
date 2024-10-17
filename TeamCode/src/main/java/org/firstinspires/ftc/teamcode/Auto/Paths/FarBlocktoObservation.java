package org.firstinspires.ftc.teamcode.Auto.Paths;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class FarBlocktoObservation extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(25,-37.5), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15.17, 10.10), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(23.24,-54.10), Math.toRadians(-10))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15.17, 10.10), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15.17, -32), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(40.54, -38), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15.17, 10.10), Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(15,-23));

        Action path = basket.build();
                waitForStart();
                Actions.runBlocking(path);

    }
}
