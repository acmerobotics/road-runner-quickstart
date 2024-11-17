package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Robot;

@Config
@Autonomous(name = "BlueBasketSide", group = "Autonomous")
public class BlueBasketSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(-15, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot bot = new Robot(hardwareMap);

        // Define trajectory using Pose2d for simultaneous right and forward movement
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-8,-45))
                .waitSeconds(6)
                //Arm to high speci and back down
                .strafeToLinearHeading(new Vector2d(-49,-43), Math.toRadians(90))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-50,-45), Math.toRadians(45))
                .waitSeconds(6)
                //intake
                .strafeToLinearHeading(new Vector2d(-60,-45), Math.toRadians(90))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-50,-45), Math.toRadians(45))
                .waitSeconds(6)
                //Arm to high basket
                //outtake
                //Arm back down
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                //intake
                .turn(Math.toRadians(55))
                .waitSeconds(1)
                //Arm to high basket
                //outtake
                //Arm back down
                .turn(Math.toRadians(-55))
                .waitSeconds(2)
                //intake
                .strafeToLinearHeading(new Vector2d(-33,-9), Math.toRadians(0))
                .waitSeconds(3)
                //Arm to high basket
                //outtake
                //Arm back down
                .strafeToLinearHeading(new Vector2d(-56,-48), Math.toRadians(65))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-33,-9), Math.toRadians(0))
                .waitSeconds(1.2);


        // Final action to close out the trajectory
        Action trajectoryActionCloseOut = tab1.fresh().build();

        Action waitAndArm = drive.actionBuilder(initialPose)
                .afterTime(0.1, bot.setPidVals(1080,4100))
//                .afterTime(0.05, bot.intake(-0.5))
                .afterTime(0.2, telemetryPacket -> {
                    bot.wrist.setPosition(0.01);
                    return false;
                })
                .afterTime(1, telemetryPacket -> {
                    bot.intakeLeft.setPower(0.5);
                    bot.intakeRight.setPower(-0.5);
                    return false;
                })
                .afterTime(3, telemetryPacket -> {
                    bot.wrist.setPosition(0.01);
                    return false;
                })
                .afterTime(4, bot.setPidVals(600,4200))
                .afterTime(4.75, telemetryPacket -> {
                    bot.intakeLeft.setPower(-0.5);
                    bot.intakeRight.setPower(0.5);
                    return false;
                })
                .afterTime(5.5, bot.setPidVals(700,0))
                .afterTime(6.5, telemetryPacket -> {
                    bot.intakeLeft.setPower(0);
                    bot.intakeRight.setPower(0);
                    return false;
                })
                .afterTime(8, bot.setPidVals(0,0))
                .afterTime(8.5, telemetryPacket -> {
                    bot.wrist.setPosition(0.07);
                    return false;
                })
                .afterTime(10, bot.setPidVals(0, 1800))
                .afterTime(10.1, telemetryPacket -> {
                    bot.intakeLeft.setPower(1);
                    bot.intakeRight.setPower(-1);
                    return false;
                })
                .afterTime(11.5, telemetryPacket -> {
                    bot.intakeLeft.setPower(0.3);
                    bot.intakeRight.setPower(-0.3);
                    return false;
                })
                .afterTime(12.5, bot.setPidVals(0,0))
                .afterTime(12.7, telemetryPacket -> {
                    bot.wrist.setPosition(0.5);
                    return false;
                })
                .afterTime(13.25,bot.setPidVals(2100,0))
                .afterTime(13.75, bot.setPidVals(2100,6000))
                .afterTime(17, telemetryPacket -> {
                    bot.intakeLeft.setPower(-0.4);
                    bot.intakeRight.setPower(0.4);
                    return false;
                })
                .afterTime(17.5, telemetryPacket -> {
                    bot.wrist.setPosition(0.07);
                    return false;
                })
                .afterTime(18.5, bot.setPidVals(2100,0))
                .afterTime(20, bot.setPidVals(0,0))
                .afterTime(22, bot.setPidVals(0, 1800))
                .afterTime(22.1, telemetryPacket -> {
                    bot.intakeLeft.setPower(1);
                    bot.intakeRight.setPower(-1);
                    return false;
                })
                .afterTime(23.5, telemetryPacket -> {
                    bot.intakeLeft.setPower(0.3);
                    bot.intakeRight.setPower(-0.3);
                    return false;
                })
                .afterTime(24.5, bot.setPidVals(0,0))
                .afterTime(24.7, telemetryPacket -> {
                    bot.wrist.setPosition(0.5);
                    return false;
                })
                .afterTime(25.25,bot.setPidVals(2100,0))
                .afterTime(25.75, bot.setPidVals(2100,6000))
                .afterTime(28.75, telemetryPacket -> {
                    bot.intakeLeft.setPower(-0.4);
                    bot.intakeRight.setPower(0.4);
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

