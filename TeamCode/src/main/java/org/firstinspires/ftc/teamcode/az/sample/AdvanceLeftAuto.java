package org.firstinspires.ftc.teamcode.az.sample;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "IntoTheDeepTeleOp")
public class AdvanceLeftAuto extends BasicLeftAuto {


    private Action firstSamplePos;
    private Action collectAction;
    private Action resetAction;
    private Action moveToCollect;
    private Action moveToDropOne;
    private TrajectoryActionBuilder specimenDropPosTraj;
    private Action highDropEjectAction;
    private Action dropAction;
    private Action moveBackToResetAction;
    private Action highDropArmSetupAction;
    private Action moveToDropTwo;
    private Action secondSamplePosAction;
    private Action parallelSpecimenHang;
    private Action movePosAction;
    private Action moveToParkAction;

    private void updateInit() {
        initAuto();
        addActions();
    }

    private void addActions() {

        parallelSpecimenHang = new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                AZUtil.runInParallel(new Runnable() {
                    @Override
                    public void run() {
                        specimenTool.specimenHang();
                        sleep(1000);
                    }
                });
                return false;
            }
        };
        specimenDropPosTraj = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(21, 0), Math.toRadians(0));
        specimenDropPos = specimenDropPosTraj.build();
        TrajectoryActionBuilder firstSamplePosTraj = specimenDropPosTraj.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(24, 27.5, Math.toRadians(50)), 0);

        firstSamplePos = firstSamplePosTraj.build();
        collectAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.autoCollect();
                return false;
            }
        };

        TrajectoryActionBuilder moveToCollectTraj = firstSamplePosTraj.endTrajectory().fresh()
                .lineToX(25);
        moveToCollect = moveToCollectTraj.build();

        TrajectoryActionBuilder moveToDropTraj1 = moveToCollectTraj.endTrajectory().fresh()
                .turn(Math.toRadians(90))
                .lineToX(10);
        moveToDropOne = moveToDropTraj1.build();

        TrajectoryActionBuilder moveToDropTraj2 = moveToDropTraj1.endTrajectory().fresh()
                .lineToX(4);
        moveToDropTwo = moveToDropTraj2.build();

        highDropArmSetupAction = new Action(){

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.dropHighBasket();
                return false;
            }
        };

        TrajectoryActionBuilder resetActioTraj = moveToDropTraj1.endTrajectory().fresh()
                .lineToX(14);
        moveBackToResetAction = resetActioTraj
                .build();

        TrajectoryActionBuilder moveToPark = resetActioTraj.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(15, -43, Math.toRadians(0)), 0)
                .lineToX(2);
        moveToParkAction = moveToPark
                .build();



        resetAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.resetAndWait();
                return false;
            }
        };

        movePosAction = new Action(){

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.move();
                return false;
            }
        };


        highDropEjectAction = new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenTool.eject();
                return false;
            }
        };

        TrajectoryActionBuilder secondSamplePosTraj = resetActioTraj.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(22, 39, Math.toRadians(50)), 0);

        secondSamplePosAction = secondSamplePosTraj.build();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        updateInit();
        Actions.runBlocking(
                new SequentialAction(
//                        parallelSpecimenHang,
                        specimenDropPos,
                        specimenHang,
                        specimenToolWait,
                        firstSamplePos,
                        collectAction,
                        new SleepAction(2),
                        movePosAction,
//                        moveToCollect,
                        moveToDropOne,
                        highDropArmSetupAction,
                        new SleepAction(2),
                        moveToDropTwo,
                        highDropEjectAction,
                        new SleepAction(3),
                        moveBackToResetAction,
                        resetAction,
                        moveToParkAction
                )
        );
//
        telemetry.addData("current position", drive.pose);
        telemetry.update();
        sleep(1000);
    }
}
