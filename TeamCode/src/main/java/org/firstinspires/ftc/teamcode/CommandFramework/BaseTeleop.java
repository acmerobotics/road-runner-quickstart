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

    /**
     * This method is called when the opmode is started. It should return a command that will be run
     * until the opmode is stopped.
     * @param scheduler The scheduler that will be used to run the command.
     * @return The command that will be run until the opmode is stopped.
     */
    public abstract Command setupTeleop(CommandScheduler scheduler);
}
