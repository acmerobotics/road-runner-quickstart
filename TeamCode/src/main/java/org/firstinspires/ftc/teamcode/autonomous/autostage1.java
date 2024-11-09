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
import org.firstinspires.ftc.teamcode.hardware.tidev2.ShoulderV0;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;


@Config
@Autonomous(name = "quarter circle forwards", group = "Autonomous")
public class autostage1 extends LinearOpMode {
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


        while (!isStopRequested() && !opModeIsActive()) {


            shoulder.init();
            elbow.init();
            intake.init();
            viper.init();
            claw.init();

            TrajectoryActionBuilder build = drive.actionBuilder(startPose).strafeTo(new Vector2d(10, -34))
                    //put arm up while strafing

                    .setReversed(true)
                    .splineTo(new Vector2d(30, -36), Math.toRadians(0))
                    //move arm down to gathering position while splining
                    .splineTo(new Vector2d(35, -5), Math.toRadians(90))
                    .setReversed(false)

                    .splineToConstantHeading(new Vector2d(48, -20), Math.toRadians(-90))

                    .splineToConstantHeading(new Vector2d(48, -50), Math.toRadians(-90))


                    //run intake while strafing to point
                    //end intake after meeting the point

                    ;


        }





        waitForStart();
        Actions.runBlocking(new ParallelAction(
                shoulder.autonListen()
        ));

    }
}

