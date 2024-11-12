package org.firstinspires.ftc.teamcode.Auto.FinalPathsNoSpeci;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@Autonomous
public class BasketRedNoSpeciFinal extends LinearOpMode {
    @Override
    public void runOpMode() {
        Intaker intake = new Intaker(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        // start
        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-21.66, 9.5), Math.toRadians(45))
                .waitSeconds(1);
        TrajectoryActionBuilder firstBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-11.77, 19.43), Math.toRadians(90))
                .waitSeconds(1);
        TrajectoryActionBuilder forwardALil = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-11.77, 19.43), Math.toRadians(90))
                .waitSeconds(1);
        TrajectoryActionBuilder secondBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-7.12, -41.617), Math.toRadians(-180))
                .waitSeconds(1);
        TrajectoryActionBuilder thirdBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-32.81, -27.32), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder parking = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-48, -30), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-48, 0), Math.toRadians(-90));

        Action toBasket = basket.build();
        Action foward = forwardALil.build();
        Action block1 = firstBlock.build();
        Action block2 = secondBlock.build();
        Action block3 = thirdBlock.build();
        Action park = parking.build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(
                //claw.flop(),
                intake.flop(),
                toBasket,
                slides.slideTopBasket(),
                claw.flip(),
                new SleepAction(0.5),
                claw.flop(),
                new SleepAction(1),
                slides.retract(),
                new SleepAction(1),
                block1,
                extendo.extend(),
                intake.flip(),
                new SleepAction(1),
                intake.intake(),
                new SleepAction(2),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                new SleepAction(1),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                toBasket,
                new SleepAction(2)

                /*

                new SleepAction(1),
                intake.off(),
                toBasket,
                slides.slideTopBasket(),
                claw.flip(),
                claw.flop(),
                slides.retract(),
                block2,
                extendo.extend(),
                intake.flip(),
                intake.intake(),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                toBasket,
                slides.slideTopBasket(),
                claw.flip(),
                claw.flop(),
                slides.retract(),
                block3,
                extendo.extend(),
                intake.flip(),
                intake.intake(),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                toBasket,
                slides.slideTopBasket(),
                claw.flip(),
                claw.flop(),
                slides.retract(),
                park

                 */

        ));
    }
}



