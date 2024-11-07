package org.firstinspires.ftc.teamcode.Auto.Paths;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class FarBlocktoObservationRedFinal extends LinearOpMode {
    Intaker intake = new Intaker(hardwareMap);
    Claw claw = new Claw(hardwareMap);
    Slides slides = new Slides(hardwareMap);
    Extendo extendo = new Extendo(hardwareMap);
    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder Model = drive.actionBuilder(StartPose1)
                //speci
                .strafeToLinearHeading(new Vector2d(-29.46,-12.24), Math.toRadians(0))
                .waitSeconds(1)
                //first block
                .strafeToLinearHeading(new Vector2d(-9.42,41.09), Math.toRadians(180))
                .waitSeconds(1)
                //extake
                .strafeToLinearHeading(new Vector2d(-12.55,37.21), Math.toRadians(0))
                .waitSeconds(1)
                //second block
                .strafeToLinearHeading(new Vector2d(-12.54,51.44), Math.toRadians(180))
                .waitSeconds(1)
                // extake
                .strafeToLinearHeading(new Vector2d(-14.55,37.21), Math.toRadians(0))
                .waitSeconds(1)
                // third block
                .strafeToLinearHeading(new Vector2d(-28.81,38.9), Math.toRadians(90))
                .waitSeconds(1)
                // extake
                .strafeToLinearHeading(new Vector2d(-14.55,37.21), Math.toRadians(0))
                .waitSeconds(4)
                // park
                .strafeToLinearHeading(new Vector2d(4.85,40), Math.toRadians(180));
        TrajectoryActionBuilder speciToTopBar = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-29.46,-12.24), Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder backALil = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-31.26, 11.07), Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder firstBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-9.42,41.09), Math.toRadians(180))
                .waitSeconds(1);
        TrajectoryActionBuilder backToObservation = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-12.55,37.21), Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder secondBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-12.54,51.44), Math.toRadians(180))
                .waitSeconds(1);
        TrajectoryActionBuilder thirdBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-28.81,38.9), Math.toRadians(90))
                .waitSeconds(1);
        TrajectoryActionBuilder lastObservation = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-14.55,37.21), Math.toRadians(0))
                .waitSeconds(4);
        TrajectoryActionBuilder parking = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(4.85,40), Math.toRadians(180));


        Action speci = speciToTopBar.build();
        Action back = backALil.build();
        Action block1 = firstBlock.build();
        Action toObservation = backToObservation.build();
        Action block2 = secondBlock.build();
        Action block3 = thirdBlock.build();
        Action finalObs = lastObservation.build();
        Action park = parking.build();


        waitForStart();
        Actions.runBlocking(new SequentialAction(
                claw.flop(),
                speci,
                slides.slideTopBar(),
                back,
                slides.retract(),
                block1,
                intake.flip(),
                intake.intake(),
                intake.flop(),
                toObservation,
                intake.flip(),
                intake.extake(),
                intake.flop(),
                block2,
                intake.flip(),
                intake.intake(),
                intake.flop(),
                toObservation,
                intake.flip(),
                intake.extake(),
                intake.flop(),
                block3,
                intake.flip(),
                intake.intake(),
                intake.flop(),
                finalObs,
                intake.flip(),
                intake.extake(),
                intake.flop(),
                park

        ));

    }
}
