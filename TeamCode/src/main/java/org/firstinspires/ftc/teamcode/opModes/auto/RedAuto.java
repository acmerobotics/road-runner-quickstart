package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="rrAutoTest", group="Tests")
public final class RedAuto extends LinearOpMode {
    Robot robot = new Robot(true);
    int randomization = 1;

    public static final double LEFT_DROP_X = 1;
    public static final double MIDDLE_DROP_X = 12;
    public static final double RIGHT_DROP_X = 22.5;
    public static final double LEFT_DROP_Y = -30;
    public static final double MIDDLE_DROP_Y = -25;
    public static final double RIGHT_DROP_Y = -30;

    public static final double PIXEL_BOARD_PREP_X = 46;
    public static final double PIXEL_BOARD_PREP_Y = -36;

    public static final double DROP2_X = 50;
    public static final double LEFT_DROP2_Y = -29.5;
    public static final double MIDDLE_DROP2_Y = -36;
    public static final double RIGHT_DROP2_Y = -41.5;

    public static final double PARK_X = 48;
    public static final double PARK_Y = -22;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        Action driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                .splineTo(randomization == 1 ? new Vector2d(LEFT_DROP_X, LEFT_DROP_Y)
                        : randomization == 2 ? new Vector2d(MIDDLE_DROP_X, MIDDLE_DROP_Y)
                        : new Vector2d(RIGHT_DROP_X, RIGHT_DROP_Y), Math.PI / 2)
                .build();

        Action driveToPixelBoard = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                .splineTo(new Vector2d(PIXEL_BOARD_PREP_X, PIXEL_BOARD_PREP_Y), Math.PI / 2)
                .build();

        Action driveToYellowDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                .splineTo(randomization == 1 ? new Vector2d(DROP2_X, LEFT_DROP2_Y)
                        : randomization == 2 ? new Vector2d(DROP2_X, MIDDLE_DROP2_Y)
                        : new Vector2d(DROP2_X, RIGHT_DROP2_Y), Math.PI / 2)
                .build();

        Action driveToPark = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                .splineTo(new Vector2d(PARK_X, PARK_Y), Math.PI / 2)
                .build();

        waitForStart();
        while (opModeIsActive()) {
            randomization = robot.drivebase.camera.getTfodElementPos();
            //DRIVE TO DROP PIXEL
            Actions.runBlocking(
                new SequentialAction(
                    driveToPurpleDrop
//                    (telemetryPacket) -> { // Drop Purple
//                        robot.pixelManipulator.claw.releaseLeftServo(robot.pixelManipulator.claw.leftProng);
//                        return false;
//                    },
//                    driveToPixelBoard,
//                    driveToYellowDrop,
//                    (telemetryPacket) -> { // Lift Slides
//                        robot.pixelManipulator.slides.update(PIDSlides.SAFE_EXTENSION_POS);
//                        return robot.pixelManipulator.slides.isAtTargetPosition();
//                    },
//                    (telemetryPacket) -> { // Extend Arm
//                        robot.pixelManipulator.arm.extend();
//                        return robot.pixelManipulator.arm.isExtended;
//                    },
//                    (telemetryPacket) -> { // Drop Yellow
//                        robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.rightProng);
//                        return robot.pixelManipulator.claw.isRightReleased;
//                    },
//                    (telemetryPacket) -> { // Retract Arm
//                        robot.pixelManipulator.arm.retract();
//                        return robot.pixelManipulator.arm.isRetracted;
//                    },
//                    (telemetryPacket) -> { // Retract Slides
//                        robot.pixelManipulator.slides.update(PIDSlides.RESET_POS);
//                        return robot.pixelManipulator.slides.isAtTargetPosition();
//                    },
//                    driveToPark
                )
            );
        }
    }
}
