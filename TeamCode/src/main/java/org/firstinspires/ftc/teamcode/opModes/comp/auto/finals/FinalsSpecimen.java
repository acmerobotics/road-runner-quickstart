package org.firstinspires.ftc.teamcode.opModes.comp.auto.finals;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystems.Robot_V2;

@Autonomous(name = "Specimen1_2", group = "AAA_COMP", preselectTeleOp="RedTeleOp")
public class FinalsSpecimen extends LinearOpMode {
    Robot_V2 robot = new Robot_V2(FinalsAutoConstants.STARTING_POSITION, true, 1);

    boolean isDone = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.scoringAssembly.multiAxisArm.hand.close();
        robot.scoringAssembly.multiAxisArm.wrist.rotateHorizontal();
        while (!isStarted() && !isStopRequested()) {
            robot.scoringAssembly.multiAxisArm.hand.loop(new AIMPad(gamepad2));
            robot.scoringAssembly.multiAxisArm.wrist.rotator.loop(new AIMPad(gamepad2));
        }

        Action preHang = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.localizer.getPose())
                .strafeTo(FinalsAutoConstants.PRELOAD_DROP.position)
                .build();

        Action pushBlockOne = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.PRELOAD_DROP)
                .splineToSplineHeading(FinalsAutoConstants.PUSH_ONE_A, FinalsAutoConstants.PUSH_ONE_A_B_TANGENT)
                .splineToLinearHeading(FinalsAutoConstants.PUSH_ONE_B, FinalsAutoConstants.PUSH_ONE_A_B_TANGENT)
                .splineToSplineHeading(FinalsAutoConstants.PUSH_ONE_C, FinalsAutoConstants.PUSH_ONE_C_TANGENT)
                .build();

        Action pushBlockTwo = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.PUSH_ONE_C)
                .splineToLinearHeading(FinalsAutoConstants.PUSH_TWO_A, FinalsAutoConstants.PUSH_TWO_A_TANGENT)
                .splineToSplineHeading(FinalsAutoConstants.PUSH_TWO_B, FinalsAutoConstants.PUSH_TWO_B_TANGENT)
                .build();

        Action pushBlockThree = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.PUSH_TWO_B)
                .splineToLinearHeading(FinalsAutoConstants.PUSH_THREE_A, FinalsAutoConstants.PUSH_THREE_A_TANGENT)
                .splineToSplineHeading(FinalsAutoConstants.PUSH_THREE_B, FinalsAutoConstants.PUSH_THREE_B_C_TANGENT)
                .splineToLinearHeading(FinalsAutoConstants.PUSH_THREE_C, FinalsAutoConstants.PUSH_THREE_B_C_TANGENT)
                .build();

        Action hangOne = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.PUSH_THREE_C)
                .setTangent(FinalsAutoConstants.HANG_ONE_A_SET_TANGENT)
                .splineToLinearHeading(FinalsAutoConstants.HANG_ONE_A, FinalsAutoConstants.HANG_ONE_A_TANGENT)
                .build();

        Action grabTwo = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.HANG_ONE_A)
                .setTangent(FinalsAutoConstants.HANG_ONE_B_SET_TANGENT)
                .splineToLinearHeading(FinalsAutoConstants.HANG_ONE_B, FinalsAutoConstants.HANG_ONE_B_TANGENT)
                .build();

        Action hangTwo = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.HANG_ONE_B)
                .splineToLinearHeading(FinalsAutoConstants.HANG_TWO_A, FinalsAutoConstants.HANG_TWO_A_TANGENT)
                .build();

        Action grabThree = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.HANG_TWO_A)
                .setTangent(FinalsAutoConstants.HANG_TWO_A_SET_TANGENT)
                .splineToLinearHeading(FinalsAutoConstants.HANG_TWO_B, FinalsAutoConstants.HANG_TWO_B_TANGENT)
                .build();

        Action hangThree = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.HANG_TWO_B)
                .splineToLinearHeading(FinalsAutoConstants.HANG_THREE_A, FinalsAutoConstants.HANG_THREE_A_TANGENT)
                .build();

        Action grabFour = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.HANG_TWO_A)
                .setTangent(FinalsAutoConstants.HANG_THREE_A_SET_TANGENT)
                .splineToLinearHeading(FinalsAutoConstants.HANG_THREE_B, FinalsAutoConstants.HANG_THREE_B_TANGENT)
                .build();

        Action hangFour = robot.drivebase.drive.actionBuilder(FinalsAutoConstants.HANG_THREE_B)
                .splineToLinearHeading(FinalsAutoConstants.HANG_FOUR_A, FinalsAutoConstants.HANG_FOUR_A_TANGENT)
                .build();



        while (opModeIsActive()){
            Actions.runBlocking(
                    new ParallelAction(
                            (telemetryPacket) -> { // Drop Purple
                                robot.loop(new AIMPad(gamepad1), new AIMPad(gamepad2));
                                return !isDone;
                            },
                            new SequentialAction(
                                    new ParallelAction(
                                            preHang,
                                            (telemetryPacket) -> { // Raise slide to drop
                                                robot.scoringAssembly.setSpecimenClampedAUTO();
                                                return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                            }
                                    ),
                                    (telemetryPacket) -> { // Clip preload
                                        robot.scoringAssembly.multiAxisArm.toggleSpecimen();
                                        return false;
                                    },
                                    new SleepAction(.5),
                                    (telemetryPacket) -> { // Reset Position
                                        robot.scoringAssembly.resetSpecimen();
                                        return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                    },
                                    pushBlockOne,
                                    pushBlockTwo,
                                    pushBlockThree,
                                    (telemetryPacket) -> { // Grab
                                        robot.scoringAssembly.multiAxisArm.hand.close();
                                        return false;
                                    },
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            hangOne,
                                            (telemetryPacket) -> { // Raise slide to drop
                                                robot.scoringAssembly.setSpecimenClampedAUTO();
                                                return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                            }
                                    ),
                                    (telemetryPacket) -> { // Clip specimen
                                        robot.scoringAssembly.multiAxisArm.toggleSpecimen();
                                        return false;
                                    },
                                    new SleepAction(.5),
                                    (telemetryPacket) -> { // Reset Position
                                        robot.scoringAssembly.resetSpecimen();
                                        return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                    },

                                    //SPECIMEN 2

                                    grabTwo,
                                    (telemetryPacket) -> { // Grab
                                        robot.scoringAssembly.multiAxisArm.hand.close();
                                        return false;
                                    },
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            hangTwo,
                                            (telemetryPacket) -> { // Raise slide to drop
                                                robot.scoringAssembly.setSpecimenClampedAUTO();
                                                return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                            }
                                    ),
                                    (telemetryPacket) -> { // Clip specimen
                                        robot.scoringAssembly.multiAxisArm.toggleSpecimen();
                                        return false;
                                    },
                                    new SleepAction(.5),
                                    (telemetryPacket) -> { // Reset Position
                                        robot.scoringAssembly.resetSpecimen();
                                        robot.scoringAssembly.resetAuto();
                                        return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                    },

                                    //SPECIMEN 3

                                    grabThree,
                                    (telemetryPacket) -> { // Grab
                                        robot.scoringAssembly.multiAxisArm.hand.close();
                                        return false;
                                    },
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            hangThree,
                                            (telemetryPacket) -> { // Raise slide to drop
                                                robot.scoringAssembly.setSpecimenClampedAUTO();
                                                return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                            }
                                    ),
                                    (telemetryPacket) -> { // Clip specimen
                                        robot.scoringAssembly.multiAxisArm.toggleSpecimen();
                                        return false;
                                    },
                                    new SleepAction(.5),
                                    (telemetryPacket) -> { // Reset Position
                                        robot.scoringAssembly.resetSpecimen();
                                        robot.scoringAssembly.resetAuto();
                                        return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                    },

                                    //SPECIMEN 4

                                    grabFour,
                                    (telemetryPacket) -> { // Grab
                                        robot.scoringAssembly.multiAxisArm.hand.close();
                                        return false;
                                    },
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            hangFour,
                                            (telemetryPacket) -> { // Raise slide to drop
                                                robot.scoringAssembly.setSpecimenClampedAUTO();
                                                return !robot.scoringAssembly.areMotorsAtTargetPresets();
                                            }
                                    ),
                                    (telemetryPacket) -> { // Clip specimen
                                        robot.scoringAssembly.multiAxisArm.toggleSpecimen();
                                        return false;
                                    },

                                    (telemetryPacket) -> { // End Auto
                                        isDone = true;
                                        return false;
                                    }
                            )
                    )
            );
            break;
        }
    }
}
