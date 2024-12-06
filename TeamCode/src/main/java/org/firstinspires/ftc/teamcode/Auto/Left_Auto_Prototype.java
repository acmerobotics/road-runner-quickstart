package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Auto.Actions.ArmActions;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Left_Auto_Prototype", group = "Autonomous")
public class Left_Auto_Prototype extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-10, 63, Math.toRadians(90));
        Pose2d subPoseMid = new Pose2d(0, 35, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(-50, 63, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        if (hardwareMap == null) {
            telemetry.addData("Error", "hardwareMap is not initialized");
            telemetry.update();
            return;
        }

        ArmActions armActions = new ArmActions(hardwareMap);
        //ArmActions  Arm = new ArmActions(hardwareMap);



        TrajectoryActionBuilder traj_1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(subPoseMid.position.x-10, subPoseMid.position.y));

        TrajectoryActionBuilder traj_2 = drive.actionBuilder(new Pose2d(-10, 35, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y-3));

        TrajectoryActionBuilder traj_3 = drive.actionBuilder(new Pose2d(-10, 60, Math.toRadians(90)))
                .strafeTo(new Vector2d(startPose.position.x+30, startPose.position.y));




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        telemetry.update();
        waitForStart();

        Actions.runBlocking(armActions.raiseClaw());
        Actions.runBlocking(armActions.closeClaw());

        if (isStopRequested()) return;

        Action trajectory_1;
        Action trajectory_2;
        Action trajectory_3;

        trajectory_1 = traj_1.build();
        trajectory_2 = traj_2.build();
        trajectory_3 = traj_3.build();

        Actions.runBlocking(
                new SequentialAction(
                        armActions.raiseArm(),
                        trajectory_1,
                        armActions.halfLowerArm(),
                        armActions.openClaw(),
                        new ParallelAction(
                                trajectory_2,
                                armActions.lowerArm()
                        ),
                        trajectory_3



                )
        );
    }
}