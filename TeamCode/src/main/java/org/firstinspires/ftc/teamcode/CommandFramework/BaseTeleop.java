package org.firstinspires.ftc.teamcode.CommandFramework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

public abstract class BaseTeleop extends LinearOpMode {

    protected Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
        waitForStart();

        robot.getScheduler().forceCommand(setupTeleop(robot.getScheduler()));

        while(opModeIsActive())
            robot.update();
    }

    public abstract Command setupTeleop(CommandScheduler scheduler);
}
