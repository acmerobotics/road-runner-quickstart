package org.firstinspires.ftc.teamcode.Auto.Paths;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class SpeciPath2 extends LinearOpMode {

    public class FailoverAction implements Action {
        private final Action mainAction;
        private final Action failoverAction;
        private boolean failedOver = false;

        public FailoverAction(Action mainAction, Action failoverAction) {
            this.mainAction = mainAction;
            this.failoverAction = failoverAction;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (failedOver) {
                return failoverAction.run(telemetryPacket);
            }

            return mainAction.run(telemetryPacket);
        }

        public void failover() {
            failedOver = true;
        }
    }

    @Override
    public void runOpMode() {

        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tele = dashboard.getTelemetry();

        TrajectoryActionBuilder path = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-3, 46.2), Math.toRadians(-90)) //pick up 1
                .waitSeconds(1.5)
                .strafeToConstantHeading(new Vector2d(-10, -5));
        TrajectoryActionBuilder path1 = path.fresh()
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0))//2+0
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(0, 49.2), Math.toRadians(-90)); //pick up 2
        Action pathhh = path.build();
        Action pathh = path1.build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(
                new FailoverAction(
                    pathhh,
                    new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)))),
                pathh));


    }

}
