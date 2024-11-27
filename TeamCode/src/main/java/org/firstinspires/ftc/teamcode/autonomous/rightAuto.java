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
                    //put arm up while strafing
                    .afterTime(0, viper.autonDown())
                    .afterTime(0, shoulder.autonHC())
                    .afterTime(0, claw.autonCloseClaw())
                    .afterTime(0, claw.autonNormalPivot())
                    .waitSeconds(0.1)

                    .strafeTo(new Vector2d(10, -34))
                    //put arm up while strafing
                    //stop at (10, -34) and place the sample on the bar
                    .waitSeconds(0.1)
                    .afterTime(0, shoulder.autonDownHC())
                    .waitSeconds(0.05)
                    .afterTime(0, claw.autonOpenClaw())
                    .waitSeconds(0.2)
                    .afterTime(0, shoulder.autonDown())

                    .strafeTo(new Vector2d(10, -45))
                    .strafeToLinearHeading(new Vector2d(35,-40), Math.toRadians(-90))


                    .strafeTo(new Vector2d(35, -13))


                    .strafeTo(new Vector2d(45,-13))
                    .strafeTo(new Vector2d(45,-53))
                    //one in observation zone
                    .strafeTo(new Vector2d(45,-13))
                    .strafeTo(new Vector2d(55,-13))
                    .strafeTo(new Vector2d(55,-53))
                    //two in observation zone
                    //prepare to grab

                    .afterTime(0, claw.autonNormalPivot())
                    .strafeTo(new Vector2d(47,-48))
                    .strafeTo(new Vector2d(47,-53))

                    //grab sample, routing towards chamber.
                    .afterTime(0, claw.autonCloseClaw())
                    .waitSeconds(0.1)
                    //raise arm to clip
                    .afterTime(0, shoulder.autonHC())
                    .strafeToSplineHeading(new Vector2d(10,-34), Math.toRadians(90))
                    //clip, routing to push final sample and grab specimen
                    .afterTime(0, shoulder.autonDownHC())
                    .waitSeconds(0.05)
                    .afterTime(0, claw.autonOpenClaw())
                    .waitSeconds(0.2)
                    .afterTime(0, shoulder.autonDown())

                    .strafeTo(new Vector2d(10, -45))
                    .strafeToLinearHeading(new Vector2d(35,-40), Math.toRadians(-90))

                    .splineToLinearHeading(new Pose2d(61,-13, Math.toRadians(-90)), Math.toRadians(-90))
                    .strafeTo(new Vector2d(61,-53))

                    .strafeTo(new Vector2d(47,-48))
                    .strafeTo(new Vector2d(47,-53))
                    //grab sample, routing towards chamber.
                    .afterTime(0, claw.autonCloseClaw())
                    .waitSeconds(0.1)
                    //raise arm to clip
                    .afterTime(0, shoulder.autonHC())
                    .strafeToSplineHeading(new Vector2d(10,-34), Math.toRadians(90))
                    //clip, park
                    .afterTime(0, shoulder.autonDownHC())
                    .waitSeconds(0.05)
                    .afterTime(0, claw.autonOpenClaw())
                    .waitSeconds(0.2)

                    .strafeToConstantHeading(new Vector2d(60,-56))
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