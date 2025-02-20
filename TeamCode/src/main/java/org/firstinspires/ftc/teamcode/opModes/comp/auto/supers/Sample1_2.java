package org.firstinspires.ftc.teamcode.opModes.comp.auto.supers;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystems.Robot_V2;

@Autonomous(name = "Sample1_2", group = "AAA_COMP", preselectTeleOp="RedTeleOp")
public class Sample1_2 extends LinearOpMode {
    Robot_V2 robot = new Robot_V2(SupersAutoConstantsSample.STARTING_POSITION, true, 1);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        Action preHang = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.localizer.getPose())
                .strafeTo(SupersAutoConstantsSample.PRELOAD_DROP.position)
                .build();

        Action dropBlockOne = robot.drivebase.drive.actionBuilder(SupersAutoConstantsSample.PRELOAD_DROP)
                .splineToLinearHeading(SupersAutoConstantsSample.BLOCK_ONE_A, SupersAutoConstantsSample.BLOCK_ONE_A_TANGENT)
                .splineToLinearHeading(SupersAutoConstantsSample.BLOCK_ONE_B, SupersAutoConstantsSample.BLOCK_ONE_B_TANGENT)
                .waitSeconds(1)
                .splineToLinearHeading(SupersAutoConstantsSample.BLOCK_ONE_C, SupersAutoConstantsSample.BLOCK_ONE_C_TANGENT)
                .waitSeconds(1)
                .build();

        Action dropBlockTwo = robot.drivebase.drive.actionBuilder(SupersAutoConstantsSample.BLOCK_ONE_C)
                .splineToLinearHeading(SupersAutoConstantsSample.BLOCK_TWO_A, SupersAutoConstantsSample.BLOCK_TWO_A_B_TANGENT)
                .waitSeconds(1)
                .splineToLinearHeading(SupersAutoConstantsSample.BLOCK_TWO_B, SupersAutoConstantsSample.BLOCK_TWO_A_B_TANGENT)
                .waitSeconds(1)
                .build();

        Action dropBlockThree = robot.drivebase.drive.actionBuilder(SupersAutoConstantsSample.BLOCK_TWO_B)
                .splineToLinearHeading(SupersAutoConstantsSample.BLOCK_THREE_A, SupersAutoConstantsSample.BLOCK_THREE_A_TANGENT)
                .waitSeconds(1)
                .splineToLinearHeading(SupersAutoConstantsSample.BLOCK_THREE_B, SupersAutoConstantsSample.BLOCK_THREE_B_TANGENT)
                .waitSeconds(1)
                .build();


        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            preHang,
                            new SleepAction(1),
                            dropBlockOne,
                            dropBlockTwo,
                            dropBlockThree
                    )
            );
            break;
        }
    }
}
