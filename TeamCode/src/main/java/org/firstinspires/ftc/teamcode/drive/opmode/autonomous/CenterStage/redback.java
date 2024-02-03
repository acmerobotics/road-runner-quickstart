package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "redback auto")
@Config
public class redback extends LinearOpMode {
    private Pose2d redbackRight =new Pose2d (-47,-30,Math.toRadians(270));
    private Pose2d redbackMid =new Pose2d (-35,-24.5,Math.toRadians(270));
    private Pose2d redback =new Pose2d (-47,-30,Math.toRadians(270));
    private Pose2d StartPose =new Pose2d (-47,-30,Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
