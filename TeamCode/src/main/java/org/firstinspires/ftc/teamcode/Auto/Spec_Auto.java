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
@Autonomous(name = "Spec_Auto", group = "Autonomous")
public class Spec_Auto extends LinearOpMode {

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
                .strafeTo(new Vector2d(subPoseMid.position.x-10, subPoseMid.position.y-3));

        TrajectoryActionBuilder traj_2 = drive.actionBuilder(new Pose2d(-10, 32, Math.toRadians(90)))
                .waitSeconds(0.5)
                //.strafeTo(new Vector2d(startPose.position.x, startPose.position.y-3));
                .strafeTo(new Vector2d(-10, 40))
                .strafeTo(new Vector2d(-35, 38));

        TrajectoryActionBuilder traj_3 = drive.actionBuilder(new Pose2d(-35, 38, Math.toRadians(90)))
                .strafeTo(new Vector2d(-35, 15))
                .strafeToLinearHeading(new Vector2d(-50, 15), Math.toRadians(269))
                .strafeTo(new Vector2d(-50,46))
                .strafeTo(new Vector2d(-50, 15))
                .strafeTo(new Vector2d(-60, 15))
                .strafeTo(new Vector2d(-60, 46))
                .strafeTo(new Vector2d (-40, 38))
                .strafeTo(new Vector2d(-41, 55))
                .strafeTo(new Vector2d(-41,54.5))
                .stopAndAdd(armActions.closeClaw())
                .stopAndAdd(armActions.raiseArm())
                .strafeToLinearHeading(new Vector2d(-8, 42), Math.toRadians(90))
                .strafeTo(new Vector2d(-8, 34))
                .stopAndAdd(armActions.halfLowerArm())
                .stopAndAdd(armActions.openClaw())
                .stopAndAdd(armActions.lowerArm())
                .strafeTo(new Vector2d(-8, 45))
                .strafeToLinearHeading(new Vector2d(-50, 60), Math.toRadians(270));

        TrajectoryActionBuilder traj_wait = drive.actionBuilder(new Pose2d(-41, 58, Math.toRadians(180)))
                .strafeTo(new Vector2d(-41, 50))
                .waitSeconds(1);

        TrajectoryActionBuilder traj_4 = drive.actionBuilder(new Pose2d(-41, 50, Math.toRadians(90)))
                .strafeTo(new Vector2d(-41,54))
                .strafeToLinearHeading(new Vector2d(-15, 38), Math.toRadians(90));





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
        Action trajectory_4;
        Action trajectory_wait;

        trajectory_1 = traj_1.build();
        trajectory_2 = traj_2.build();
        trajectory_3 = traj_3.build();
        trajectory_4 = traj_4.build();
        trajectory_wait = traj_wait.build();

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
                        //armActions.closeClaw(),
                        //trajectory_wait,
                        //armActions.raiseArm()
                        //trajectory_4

                )
        );
    }
}