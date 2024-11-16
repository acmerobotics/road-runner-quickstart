package org.firstinspires.ftc.teamcode.autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder.*;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;


@Config
@Autonomous(name = "Right Auto", group = "Autonomous")
public class rightAuto extends LinearOpMode {
    Pose2d startPose;
    MecanumDrive drive;

    Shoulder shoulder = new Shoulder(this);
    Elbow elbow = new Elbow(this);
    Intake intake = new Intake(this);
    Viper viper = new Viper(this);
    Claw claw = new Claw(this);





    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(14, -61, Math.toRadians(90));
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

                    .strafeTo(new Vector2d(10, -32))
                    //put arm up while strafing
                    //stop at (10, -34) and place the sample on the bar
                    .waitSeconds(0.5)
                    .afterTime(0, shoulder.autonDownHC())
                    .waitSeconds(1)
                    .setReversed(true)
                    .strafeTo(new Vector2d(10, -36))
                    .splineTo(new Vector2d(28, -36), Math.toRadians(0))
                    //move arm down to gathering position while splining
                    .afterTime(0, shoulder.autonDown())
//                    .splineTo(new Vector2d(33, -5), Math.toRadians(90)) //successfully hooked and moved away

//                    .setReversed(false)
//
//                    .splineToConstantHeading(new Vector2d(47, -20), Math.toRadians(-90))
//
//                    .splineToConstantHeading(new Vector2d(47, -50), Math.toRadians(-90))
//                    //shrey code starts here
//                    .setReversed(true)
//                    .strafeTo(new Vector2d(47, -10))
//
//
//                    //.splineToConstantHeading(new Vector2d(52,-45), Math.toRadians
//                    .strafeTo(new Vector2d(56,-10))
//
//                    .strafeTo(new Vector2d(56,-53))
//
//                    .afterTime(0, viper.autonGolf())
//                    .waitSeconds(0.3)
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)), Math.toRadians(0))
                    .strafeToConstantHeading(new Vector2d(60,-56))
                    //.waitSeconds(0.3)
                    //.afterTime(0, viper.autonDown())
            ;


            //run intake while strafing to point
            //end intake after meeting the point



        }





        waitForStart();
        if (build != null) {
            Actions.runBlocking(new ParallelAction(
                    shoulder.autonListen(),
                    viper.autonListen(),
                    build.build()


            ));
        }

    }
}