package org.firstinspires.ftc.teamcode.opModes.comp.auto.qual2;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.v1.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.v1.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.v1.Robot;

@Autonomous(name = "Sample 1+2", group = "AAA_COMP", preselectTeleOp="BlueTeleOp")
public final class Sample1_2 extends LinearOpMode {

    Qual2AutoConstants constants = new Qual2AutoConstants();
    Robot robot = new Robot(false, constants.RED_STARTING_POSITION);

    AIMPad aimPad1;
    AIMPad aimPad2;

    boolean isAutoComplete = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        aimPad1 = new AIMPad(gamepad1);
        aimPad2 = new AIMPad(gamepad2);

        Action driveToPreloadDrop = robot.drivebase.drive.actionBuilder(constants.RED_STARTING_POSITION)
                .strafeTo(constants.RED_PRELOAD_DROP.position)
                .build();

        Action driveToPickUp1 = robot.drivebase.drive.actionBuilder(constants.RED_PRELOAD_DROP)
                .splineToLinearHeading(constants.RED_POST_DROP, constants.RED_POST_DROP_TANGENT)
                .splineToLinearHeading(constants.RED_PICKUP_ONE, constants.RED_PICKUP_ONE_TANGENT)
                .build();

        Action driveToPickUp2 = robot.drivebase.drive.actionBuilder(constants.RED_HIGH_BASKET)
                .splineToLinearHeading(constants.RED_PICKUP_TWO, constants.RED_PICKUP_TWO_TANGENT)
                .build();

        Action driveToBucket1 = robot.drivebase.drive.actionBuilder(constants.RED_PICKUP_ONE)
                .splineToLinearHeading(constants.RED_HIGH_BASKET, constants.RED_HIGH_BASKET_TANGENT)
                .build();

        Action driveToBucket2 = robot.drivebase.drive.actionBuilder(constants.RED_PICKUP_TWO)
                .splineToLinearHeading(constants.RED_HIGH_BASKET, constants.RED_HIGH_BASKET_TANGENT)
                .build();

        while (!isStarted() && !isStopRequested()) {
            robot.scoringSystem.resetMechs();

            robot.loop(aimPad1, aimPad2);

            aimPad1.update(gamepad1);
            aimPad2.update(gamepad2);

            robot.telemetry(telemetry);

            telemetry.update();
        }

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            (telemetryPacket) -> { // robot loop
                                robot.loop(aimPad1, aimPad2);
                                return isAutoComplete;
                            },
                            new SequentialAction(
                                    new ParallelAction(
                                        driveToPreloadDrop,
                                        new SequentialAction(
                                            new SleepAction(0.5),
                                            (telemetryPacket) -> { // raise lifts
                                                robot.scoringSystem.outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_HIGH);
                                                return robot.scoringSystem.outtakeSystem.outtakeSlides.isAtTargetPosition();
                                            }
                                        )
                                    ),
                                    (telemetryPacket) -> { // drop lifts to high specimen drop height
                                        robot.scoringSystem.outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_HIGH_DROP);
                                        return robot.scoringSystem.outtakeSystem.outtakeSlides.isAtTargetPosition();
                                    },
                                    new ParallelAction(
                                        driveToPickUp1,
                                        (telemetryPacket) -> { // lower lifts and release SG
                                            robot.scoringSystem.specimenGrabber.release();
                                            robot.scoringSystem.intakeSystem.multiAxisArm.searchingDownOpen();
                                            robot.scoringSystem.outtakeSystem.reset();
                                            return robot.scoringSystem.outtakeSystem.outtakeSlides.isAtTargetPosition();
                                        }
                                    ),
                                    new SequentialAction(
                                        (telemetryPacket) -> { // extend lower lifts and set to searching position
                                            robot.scoringSystem.intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.MEDIUM);
                                            return robot.scoringSystem.intakeSystem.intakeSlides.isAtTargetPosition();
                                        },
                                        (telemetryPacket) -> { // grab
                                            robot.scoringSystem.intakeSystem.multiAxisArm.searchingDownClosed();
                                            return false;
                                        },
                                        new SleepAction(0.25)
                                    ),
                                    new ParallelAction(
                                        driveToBucket1,
                                        new SequentialAction(
                                            (telemetryPacket) -> { // retract lower lifts and set arm to transition position
                                                robot.scoringSystem.intakeSystem.multiAxisArm.resetClosed();
                                                robot.scoringSystem.intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.RESET);
                                                return robot.scoringSystem.intakeSystem.intakeSlides.isAtTargetPosition();
                                            },
                                            (telemetryPacket) -> { // transition to outtake
                                                robot.scoringSystem.intakeSystem.multiAxisArm.resetOpen();
                                                return false;
                                            },
                                            new SleepAction(0.25),
                                            (telemetryPacket) -> { // raise lifts
                                                robot.scoringSystem.intakeSystem.multiAxisArm.searchingDownOpen();
                                                robot.scoringSystem.outtakeSystem.outtake.armOut();
                                                robot.scoringSystem.outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.TALL);
                                                return robot.scoringSystem.outtakeSystem.outtakeSlides.isAtTargetPosition();
                                            }
                                        )
                                    ),
                                    (telemetryPacket) -> { // drop
                                        robot.scoringSystem.outtakeSystem.outtake.bucketOut();
                                        return false;
                                    },
                                    new SleepAction(0.5),
                                    //TODO SECOND BLOCK
                                    new ParallelAction(
                                            driveToPickUp2,
                                            (telemetryPacket) -> { // lower lifts and release SG
                                                robot.scoringSystem.outtakeSystem.reset();
                                                robot.scoringSystem.intakeSystem.multiAxisArm.searchingDownOpen();
                                                return robot.scoringSystem.outtakeSystem.outtakeSlides.isAtTargetPosition();
                                            },
                                            (telemetryPacket) -> { // extend lower lifts
                                                robot.scoringSystem.intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.MEDIUM);
                                                return robot.scoringSystem.intakeSystem.intakeSlides.isAtTargetPosition();
                                            }
                                    ),
                                    new SequentialAction(
                                            (telemetryPacket) -> { // grab
                                                robot.scoringSystem.intakeSystem.multiAxisArm.searchingDownClosed();
                                                return false;
                                            },
                                            new SleepAction(0.25)
                                    ),
                                    new ParallelAction(
                                            driveToBucket2,
                                            new SequentialAction(
                                                    (telemetryPacket) -> { // retract lower lifts and set arm to transition position
                                                        robot.scoringSystem.intakeSystem.multiAxisArm.resetClosed();
                                                        robot.scoringSystem.intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.RESET);
                                                        return robot.scoringSystem.intakeSystem.intakeSlides.isAtTargetPosition();
                                                    },
                                                    (telemetryPacket) -> { // transition to outtake
                                                        robot.scoringSystem.intakeSystem.multiAxisArm.resetOpen();
                                                        return false;
                                                    },
                                                    new SleepAction(0.25),
                                                    (telemetryPacket) -> { // raise lifts
                                                        robot.scoringSystem.intakeSystem.multiAxisArm.resetAvoid();
                                                        robot.scoringSystem.outtakeSystem.outtake.armOut();
                                                        robot.scoringSystem.outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.TALL);
                                                        return robot.scoringSystem.outtakeSystem.outtakeSlides.isAtTargetPosition();
                                                    }
                                            )
                                    ),
                                    (telemetryPacket) -> { // drop
                                        robot.scoringSystem.outtakeSystem.outtake.bucketOut();
                                        return false;
                                    },
                                    new SleepAction(0.5),
                                    (telemetryPacket) -> { // retract lifts
                                        robot.scoringSystem.resetMechs();
                                        return robot.scoringSystem.intakeSystem.intakeSlides.isAtTargetPosition() && robot.scoringSystem.outtakeSystem.outtakeSlides.isAtTargetPosition();
                                    },
                                    (telemetryPacket) -> { // end auto
                                        isAutoComplete = true;
                                        return false;
                                    }
                            )
                    )
            );
        }
    }
}