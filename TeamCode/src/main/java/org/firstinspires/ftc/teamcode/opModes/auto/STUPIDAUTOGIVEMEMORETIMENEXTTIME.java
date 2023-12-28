package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="IHATEMYMECHANICSTEAMGIVEMEMORETIME", group="AUTOOOO")
public final class STUPIDAUTOGIVEMEMORETIMENEXTTIME extends LinearOpMode {
    Robot robot = new Robot(true);
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.pixelManipulator.arm.retract();
            robot.drivebase.drive.setDrivePowers(robot.drivebase.stickOnly(.65, 0, 0));
            sleep(2000);
            robot.drivebase.drive.setDrivePowers(robot.drivebase.stickOnly(0, 0, 0));
            robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.rightProng);
            robot.pixelManipulator.claw.releaseLeftServo(robot.pixelManipulator.claw.leftProng);
            robot.drivebase.drive.setDrivePowers(robot.drivebase.stickOnly(.65, 0, 0));
            sleep(300);
            robot.drivebase.stickOnly(0, 0, 0);
            break;
        }
    }
}
