package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;


@Config
@Autonomous(name = "Left Auto", group = "Autonomous")
public class leftAuto extends LinearOpMode {
    Pose2d startPose;
    MecanumDrive drive;

    Shoulder shoulder = new Shoulder(this);
    Elbow elbow = new Elbow(this);
    Intake intake = new Intake(this);
    Viper viper = new Viper(this);
    Claw claw = new Claw(this);





    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(-14, -61, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder build = null;

        while (!isStopRequested() && !opModeIsActive()) {


            shoulder.init();
            elbow.init();
            intake.init();
            viper.init();
            claw.init();

            build = drive.actionBuilder(startPose)
                    //put arm up while strafing
                    .afterTime(0, viper.autonDown())
                    .afterTime(0, shoulder.autonHC())
                    .waitSeconds(0.5)

                    .strafeTo(new Vector2d(-10, -33))
                    //put arm up while strafing
                    //stop at (10, -33) and place the sample on the bar
                    .waitSeconds(0.5)
                    .afterTime(0, shoulder.autonDownHC())
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(-10,-40))
                    .waitSeconds(1)
                    .afterTime(0, shoulder.autonMidDown())
                    .waitSeconds(0.3)
                    .afterTime(0, shoulder.autonDown())
                    .strafeTo(new Vector2d(-30,-40))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-36, -28), Math.toRadians(180)), Math.toRadians(0))
                    .strafeTo(new Vector2d(-34, -28))
                    //pick sample
                    .waitSeconds(0.5)
                    .afterTime(0, elbow.autonPick())
                    .waitSeconds(2)
                    .afterTime(0, intake.autoIntake())
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(-38, -28))
                    .waitSeconds(1)
                    .afterTime(0, intake.autoStoptake())
                    .afterTime(0, elbow.autonZero())
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(-38, -34))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-54.5, -54.5), Math.toRadians(225)), Math.toRadians(-90))
                    .waitSeconds(1)
                    .afterTime(0, shoulder.autonUpHB())
                    .waitSeconds(1.5)
                    .afterTime(0, viper.autonHB())
                    .afterTime(0, elbow.autonHB())
                    .strafeTo(new Vector2d(-56.5, -56.5))
                    .waitSeconds(2)
                    .afterTime(0, intake.autoOuttake())
                    .waitSeconds(2)
                    .afterTime(0, intake.autoStoptake())
                    .afterTime(0, elbow.autonZero())
                    .afterTime(0, viper.autonDown())
                    .waitSeconds(1)
                    .afterTime(0, shoulder.autonMidDown())
                    .waitSeconds(0.6)
                    .strafeTo(new Vector2d(-54.5, -54.5))
                    .waitSeconds(0.5)
                    .afterTime(0, shoulder.autonDown())
                    .waitSeconds(0.2)
                    .strafeTo(new Vector2d(-56, -56))


                    ;




        }





        waitForStart();
        if (build != null) {
            Actions.runBlocking(new ParallelAction(
                    shoulder.autonListen(),
                    viper.autonListen(),
                    elbow.autonListen(),
                    intake.autonListen(),
                    claw.autonListen(),
                    build.build()


            ));
        }

    }
}

