package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="TwoDrivers", group="Linear Opmode")
public class SampleDriveTele extends LinearOpMode implements Runnable{
    HardwareFile robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareFile(hardwareMap);
        waitForStart();
        run();
    }

    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    public void run() {
        while(opModeIsActive()) {
            robot.driveTrain.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * lyMult,
                            -gamepad1.left_stick_x * lxMult,
                            -gamepad1.right_stick_x * 0.92 * rxMult
                    )
            );
            setMultiplier();
            robot.driveTrain.update();
        }
    }

    private void setMultiplier() {
        if (gamepad1.left_trigger >= 0.3) {
            lxMult = 0.5;
            rxMult = 0.5;
            lyMult = 0.5;
        } else {
            lxMult = 1;
            rxMult = 1;
            lyMult = 1;
        }
        if (gamepad1.right_bumper) {
            lxMult = -lxMult;
            lyMult = -lyMult;
        }
    }
}
