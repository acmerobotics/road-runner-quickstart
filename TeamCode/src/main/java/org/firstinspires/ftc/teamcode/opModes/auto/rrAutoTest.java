package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="rrAutoTest", group="Tests")
public final class rrAutoTest extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            Actions.runBlocking(
                robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(60, 0), Math.PI)
                        .build());
        }
    }
}
