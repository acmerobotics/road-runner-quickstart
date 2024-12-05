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
                    .afterTime(0, shoulder.autonHC())
                    .strafeTo(new Vector2d(5, -33))

                    //put arm up while strafing
                    //stop at (9, -33) and place the sample on the bar
                    .afterTime(0, viper.autonHangSpecimen())
                    .waitSeconds(1.0)
                    .strafeTo(new Vector2d(5, -31))
                    .afterTime(0, claw.autonOpenClaw())
                    .afterTime(0, viper.autonDown())
                    .waitSeconds(0.5)


//                .afterTime(0, shoulder.autonDownHC())
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)
                    .afterTime(0, shoulder.autonDown())

                    .splineTo(new Vector2d(45, -13), 0)

                    .strafeTo(new Vector2d(45,-53))
                    //one in observation zone
                    .strafeTo(new Vector2d(45,-13))
                    .strafeTo(new Vector2d(55,-13))
                    .strafeTo(new Vector2d(55,-53))
                    //two in observation zone
                    //prepare to grab
                    //.afterTime(0, claw.autonNormalPivot())
                    .strafeTo(new Vector2d(47,-53))

                    .strafeTo(new Vector2d(47,-58))

                    .waitSeconds(0.1)
                    .strafeTo(new Vector2d(47,-58))

                    .afterTime(0, claw.autonCloseClaw())
                    .waitSeconds(0.5)
                    .afterTime(0, shoulder.autonHC())
                    //grab sample, routing towards chamber.
                    //raise arm to clip
                    .strafeToSplineHeading(new Vector2d(5, -33), Math.toRadians(90))
                    //clip, routing to push final sample and grab specimen
                    .afterTime(0, viper.autonHangSpecimen())
                    .waitSeconds(1.0)
                    .strafeTo(new Vector2d(5, -31))
                    .afterTime(0, claw.autonOpenClaw())
                    .afterTime(0, viper.autonDown())
                    .waitSeconds(0.5)



                    .setReversed(true)
                    .splineTo(new Vector2d(25, -38), Math.toRadians(0))
                    .afterTime(0, shoulder.autonDown())


                    .splineToSplineHeading(new Pose2d(new Vector2d(60,-13), Math.toRadians(-90)), Math.toRadians(-90))

                    //.splineToLinearHeading(new Pose2d(61,-13, Math.toRadians(-90)), Math.toRadians(-90))
                    .strafeTo(new Vector2d(60,-53))

                    .strafeTo(new Vector2d(47,-53))
                    .strafeTo(new Vector2d(47,-58))
                    .waitSeconds(0.1)
                    .strafeTo(new Vector2d(47,-58))
                    .afterTime(0, claw.autonCloseClaw())
                    .waitSeconds(0.5)
                    .afterTime(0, shoulder.autonHC())
                    //grab sample, routing towards chamber.
                    //raise arm to clip
                    .strafeToSplineHeading(new Vector2d(5, -33), Math.toRadians(90))
                    .afterTime(0, viper.autonHangSpecimen())
                    .waitSeconds(1.0)
                    .strafeTo(new Vector2d(5, -31))
                    .afterTime(0, claw.autonOpenClaw())
                    .afterTime(0, viper.autonDown())
                    .waitSeconds(0.5)

                    .strafeToConstantHeading(new Vector2d(60,-56))
                    .afterTime(0, shoulder.autonDown())

            ;


            //run intake while strafing to point
            //end intake after meeting the point



        }





        if (build != null) {
            Actions.runBlocking(new ParallelAction(
                    shoulder.autonListen(),
                    viper.autonListen(),
                    claw.autonListen(),
                    viper.autonListen(),
                    build.build()


            ));
        }

    }
}