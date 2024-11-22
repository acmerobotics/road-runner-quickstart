package org.firstinspires.ftc.teamcode.opModes.comp.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenGrabber;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "HighSpecimen", group = "AAA_COMP")
public final class HighSpecimen extends LinearOpMode {
    Robot robot = new Robot(true, AutoConstants.STARTING_POSITION);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.drivebase.setActiveDriveMode(Drivebase.DriveMode.TO_POSITION);
        robot.scoringSystem.setActiveScoringState(ScoringSystem.ScoringState.AUTO_PERIOD);

        AIMPad aimPad1 = new AIMPad(gamepad1);
        AIMPad aimPad2 = new AIMPad(gamepad2);

        AtomicBoolean isFinished = new AtomicBoolean(true);

        while (!isStarted() && !isStopRequested()) {
            robot.telemetry(telemetry);
            telemetry.update();
        }

        while (opModeIsActive()) {

            //DRIVE TO DROP PIXEL
            Actions.runBlocking(
                    new ParallelAction(
                            (telemetryPacket) -> { // Drive to High Drop
                                robot.loop(aimPad1, aimPad2);
                                robot.telemetry(telemetry);
                                telemetry.update();
                                return isFinished.get();
                            },
                            new SequentialAction(
                                    (telemetryPacket) -> { // Drive to High Drop Prep Spot
                                        long startTime = System.currentTimeMillis();
                                        long timeout = 4000; // Timeout in milliseconds
                                        robot.scoringSystem.specimenGrabber.setGrabberState(SpecimenGrabber.GrabberState.GRAB);
                                        robot.drivebase.setTargetPose(AutoConstants.HIGH_SPECIMEN_DROP_PREP);
                                        return false;
                                    },
                                    new SleepAction(2),
                                    (telemetryPacket) -> { // Raise Lifts
                                        robot.scoringSystem.outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_HIGH);
                                        return false;
                                    },
                                    new SleepAction(2.5),
                                    (telemetryPacket) -> { // Drive to High Drop
                                        long startTime = System.currentTimeMillis();
                                        long timeout = 2000; // Timeout in milliseconds
                                        robot.drivebase.setTargetPose(AutoConstants.HIGH_SPECIMEN_DROP);
                                        return false;
                                    },
                                    new SleepAction(2),
                                    (telemetryPacket) -> { // Score Specimen
                                        robot.scoringSystem.outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_HIGH_DROP);
                                        return false;
                                    },
                                    new SleepAction(1),
                                    (telemetryPacket) -> { // Release
                                        robot.scoringSystem.specimenGrabber.setGrabberState(SpecimenGrabber.GrabberState.RELEASE);
                                        return false;
                                    },
                                    new SleepAction(1),
                                    (telemetryPacket) -> { // Drive to Park
                                        long startTime = System.currentTimeMillis();
                                        long timeout = 6000; // Timeout in milliseconds
                                        robot.drivebase.setTargetPose(AutoConstants.PARK);
                                        robot.scoringSystem.outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.RESET);
                                        return false;
                                    },
                                    new SleepAction(6),
                                    (telemetryPacket) -> {;// Finish
                                        isFinished.set(false);
                                        return false;
                                    }
                            )
                    )

            );
            break;
        }
    }
}
