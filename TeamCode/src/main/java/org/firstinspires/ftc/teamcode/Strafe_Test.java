package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "Strafe_Test", group = "Autonomous")
public class Strafe_Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-10, 63, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        if (hardwareMap == null) {
            telemetry.addData("Error", "hardwareMap is not initialized");
            telemetry.update();
            return;
        }

        //ArmActions  Arm = new ArmActions(hardwareMap);




        TrajectoryActionBuilder traj_1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x-20, startPose.position.y));




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;

        Action trajectory_1;

        trajectory_1 = traj_1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectory_1

                )
        );
    }
}
