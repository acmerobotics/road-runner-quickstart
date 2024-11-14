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
@Autonomous(name = "RedBasketSide", group = "Autonomous")
public class RedBasketSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(-15, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot bot = new Robot(hardwareMap);

        // Define trajectory using Pose2d for simultaneous right and forward movement
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-9,-40))
                .waitSeconds(5)
                //Arm to high speci and back down
                .strafeToLinearHeading(new Vector2d(-56,-48), Math.toRadians(65))
                .waitSeconds(3)
                //intake
                .turn(Math.toRadians(30))
                .waitSeconds(1)
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
                .afterTime(0,bot.setPidVals(600,3600))
                .afterTime(5, bot.setPidVals(0,3600))
                .build();

        bot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Wait for the start of the op mode
        waitForStart();

        bot.setPidValues(0,0);
        if (isStopRequested()) return;
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
