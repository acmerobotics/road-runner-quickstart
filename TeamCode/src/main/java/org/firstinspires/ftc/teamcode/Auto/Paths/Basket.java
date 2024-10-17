package org.firstinspires.ftc.teamcode.Auto.Paths;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Basket extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        TrajectoryActionBuilder trajecotryegrig = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(5.74, 42.10), Math.toRadians(-45))
                .afterDisp(0, new SleepAction(5.0))
                .strafeToLinearHeading(new Vector2d(24.72, 38.52), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(5.74, 45.10), Math.toRadians(-45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(25.86, 37.24), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(5.74, 45.10), Math.toRadians(-45))
                .waitSeconds(1)
                .strafeTo(new Vector2d(5,20))
                .strafeToLinearHeading(new Vector2d(46.57, 36.09), Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(6.74, 45.10), Math.toRadians(-45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(60, 24.17), Math.toRadians(90))
                .strafeTo(new Vector2d(60,4.67));

        Action auto = trajecotryegrig.build();
        waitForStart();
        Actions.runBlocking(auto);
    }
}

