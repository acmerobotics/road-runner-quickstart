package org.firstinspires.ftc.teamcode.auton;

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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "RedHumanSide", group = "Autonomous")
public class RedHumanSide extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(15, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot bot = new Robot(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8,-45))
                .waitSeconds(5)
                //Arm to high speci and back down
                .strafeToLinearHeading(new Vector2d(30,-48), Math.toRadians(75));
//                .strafeToLinearHeading(new Vector2d(38,-14), Math.toRadians(80))
//                .strafeToLinearHeading(new Vector2d(45,-12), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(45,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(56,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(56,-50), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(56,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(61,-13), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(61,-50), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(48,-54), Math.toRadians(90))
//                .waitSeconds(1.5)
//                .strafeToLinearHeading(new Vector2d(4,-45), Math.toRadians(90))
//                .waitSeconds(1.9)
//                .strafeTo(new Vector2d(48,-54))
//                .waitSeconds(1.5)
//                .strafeTo(new Vector2d(4,-45))
//                .waitSeconds(1.9)
//                .strafeToLinearHeading(new Vector2d(42,-50), Math.toRadians(90));

        // Final action to close out the trajectory
        Action trajectoryActionCloseOut = tab1.fresh().build();

        Action waitAndArm = drive.actionBuilder(initialPose)
                .afterTime(0, bot.setPidVals(1100,4200))
//                .afterTime(0.05, bot.intake(-0.5))
                .afterTime(0.1, telemetryPacket -> {
                    bot.wrist.setPosition(0.01);
                    return false;
                })
                .afterTime(1, telemetryPacket -> {
                    bot.intakeLeft.setPower(-0.5);
                    bot.intakeRight.setPower(0.5);
                    return false;
                })
                .afterTime(4, bot.setPidVals(700,4200))
                .afterTime(4.75, telemetryPacket -> {
                    bot.intakeLeft.setPower(0.5);
                    bot.intakeRight.setPower(-0.5);
                    return false;
                })
                .afterTime(5.5, bot.setPidVals(700,0))
                .afterTime(7, telemetryPacket -> {
                    bot.intakeLeft.setPower(0);
                    bot.intakeRight.setPower(0);
                    return false;
                })
                .build();

        bot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the start of the op mode
        waitForStart();
        if (isStopRequested()) return;
        bot.wrist.setPosition(0.5);
        Robot.stopPid = false;

        // Execute the defined trajectory
        Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                trajectoryActionChosen,
                                trajectoryActionCloseOut),
                        waitAndArm,
                        bot.getPIDAction()
                )
        );
        bot.stopPidAction();
    }
}
