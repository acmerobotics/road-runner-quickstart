package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;



@Config
@Autonomous(name = "RedNeutral", group = "Auto Segments")
public class autostage2 extends LinearOpMode {
    Shoulder shoulder = new Shoulder(this);
    Elbow elbow = new Elbow(this);
    Intake intake = new Intake(this);
    Viper viper = new Viper(this);
    Claw claw = new Claw(this);




    @Override
    public void runOpMode() throws InterruptedException {



        while (!isStopRequested() && !opModeIsActive()) {
            // instantiate your MecanumDrive at a particular pose.
            Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            shoulder.init();
            elbow.init();
            intake.init();
            viper.init();
            claw.init();


            TrajectoryActionBuilder redneutral = drive.actionBuilder(initialPose)
                    .splineToConstantHeading(new Vector2d(-34,-35), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-36, -26), Math.toRadians(180)), Math.toRadians(180));



            TrajectoryActionBuilder redbucket = drive.actionBuilder(new Pose2d(new Vector2d(-36, -26), Math.toRadians(180)))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-47, -47), Math.toRadians(225)), Math.toRadians(180));
        }

        waitForStart();






    }
}

