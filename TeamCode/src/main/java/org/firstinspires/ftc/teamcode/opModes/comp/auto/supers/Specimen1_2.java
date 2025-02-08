package org.firstinspires.ftc.teamcode.opModes.comp.auto.supers;

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
public class Specimen1_2 extends LinearOpMode {
    Robot_V2 robot = new Robot_V2(SupersAutoConstants.STARTING_POSITION, true);

    boolean isDone = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.scoringAssembly.multiAxisArm.hand.close();
        while (!isStarted() && !isStopRequested()) {
            robot.scoringAssembly.multiAxisArm.hand.loop(new AIMPad(gamepad2));
        }

        Action preHang = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.localizer.getPose())
                .strafeTo(SupersAutoConstants.PRELOAD_DROP.position)
                .build();

        Action pushBlockOne = robot.drivebase.drive.actionBuilder(SupersAutoConstants.PRELOAD_DROP)
                .strafeTo(SupersAutoConstants.CLEARANCE.position)
                .splineToLinearHeading(SupersAutoConstants.PUSH_ONE_A, SupersAutoConstants.PUSH_ONE_A_C_TANGENT)
                .splineToLinearHeading(SupersAutoConstants.PUSH_ONE_B, SupersAutoConstants.PUSH_ONE_B_D_TANGENT)
                .splineToLinearHeading(SupersAutoConstants.PUSH_ONE_C, SupersAutoConstants.PUSH_ONE_A_C_TANGENT)
                .splineToLinearHeading(SupersAutoConstants.PUSH_ONE_D, SupersAutoConstants.PUSH_ONE_B_D_TANGENT)
                .build();

            Action pushBlockTwo = robot.drivebase.drive.actionBuilder(SupersAutoConstants.PUSH_ONE_D)
                .splineToLinearHeading(SupersAutoConstants.PUSH_TWO_A, SupersAutoConstants.PUSH_TWO_A_TANGENT)
                .splineToLinearHeading(SupersAutoConstants.PUSH_TWO_B, SupersAutoConstants.PUSH_TWO_B_TANGENT)
                .build();

            Action pushBlockThree = robot.drivebase.drive.actionBuilder(SupersAutoConstants.PUSH_TWO_B)
                .splineToLinearHeading(SupersAutoConstants.PUSH_THREE_A, SupersAutoConstants.PUSH_THREE_A_TANGENT)
                .strafeTo(SupersAutoConstants.PUSH_THREE_B.position)
                .splineToLinearHeading(SupersAutoConstants.HANG_ONE_A, SupersAutoConstants.HANG_ONE_A_TANGENT)
                .build();

            Action hangOne = robot.drivebase.drive.actionBuilder(SupersAutoConstants.HANG_ONE_A)
                .splineToLinearHeading(SupersAutoConstants.HANG_ONE_B, SupersAutoConstants.HANG_ONE_B_TANGENT)
                .build();

            Action grabTwo = robot.drivebase.drive.actionBuilder(SupersAutoConstants.HANG_ONE_B)
                    .strafeTo(SupersAutoConstants.HANG_TWO_CLEARANCE.position)
                    .splineToLinearHeading(SupersAutoConstants.HANG_TWO_A, SupersAutoConstants.HANG_TWO_A_TANGENT)
                    .build();

            Action hangTwo = robot.drivebase.drive.actionBuilder(SupersAutoConstants.HANG_TWO_A)
                .splineToLinearHeading(SupersAutoConstants.HANG_TWO_B, SupersAutoConstants.HANG_TWO_B_TANGENT)
                .build();

//            Action hangThree = robot.drivebase.drive.actionBuilder(SupersAutoConstants.HANG_TWO_B)
//                .strafeTo(SupersAutoConstants.HANG_THREE_A.position)
//                .waitSeconds(1)
//                .strafeTo(SupersAutoConstants.HANG_THREE_B.position)
//                .waitSeconds(2)
//                .build();
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


                                    (telemetryPacket) -> { // End Auto
                                        isDone = true;
                                        return false;
                                    }
                            )
//                                    hangTwo,
//                                    hangThree
                    )
            );
            break;
        }
    }
}
