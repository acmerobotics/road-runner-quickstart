
package org.firstinspires.ftc.teamcode.testing.structureOptions.inheritanceStructure;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Hardware example", group="Linear Opmode")

public class hardwareExample extends LinearOpMode {

    Hardware hardware = new Hardware();
    IMUDT drivetrain = hardware.getDrivetrain();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();


    // 0 is robot centric
    // 1 is field centric
    int transMode = 1;
    int rotMode = 1;
    int lBumperPressed = 0;
    int rBumperPressed = 0;


    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);
        drivetrain.setScls(0.6, 0.4);





        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                drivetrain.initIMU();
                drivetrain.setForwards();
            }

            if(gamepad1.left_bumper){
                transMode = (1 - transMode)*(1 - lBumperPressed) + transMode*lBumperPressed;
                lBumperPressed = 1;
            }else{
                lBumperPressed = 0;
            }
            if(gamepad1.right_bumper){
                rotMode = (1 - rotMode)*(1 - rBumperPressed) + rotMode*rBumperPressed;
                rBumperPressed = 1;
            }else{
                rBumperPressed = 0;
            }

            if(gamepad1.left_stick_button){
                drivetrain.setTransScl(1);
            }else{
                drivetrain.setTransScl(0.6);
            }
            if(gamepad1.right_stick_button){
                drivetrain.setRotScl(1);
            }else{
                drivetrain.setRotScl(0.4);
            }


            double robotDirection = drivetrain.getDirection();
            double targetDirection = Math.atan2(-gamepad1.right_stick_x, -gamepad1.right_stick_y);
            double diff = ((targetDirection - robotDirection)/Math.PI + 3) % 2 - 1;
            double dir = Math.max(-1, Math.min(1, diff*8));
            double mag = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

            double rot = dir * mag * rotMode - gamepad1.right_stick_x*(1 - rotMode);
            if(transMode == 1) {
                drivetrain.absoluteDirectionalPow(-gamepad1.left_stick_y, -gamepad1.left_stick_x, rot);
            }else{
                drivetrain.directionalPow(-gamepad1.left_stick_y, -gamepad1.left_stick_x, rot);
            }


            telemetry.addData("trans mode", transMode);
            telemetry.addData("rot mode", rotMode);
            telemetry.addData("current direction", robotDirection);
            telemetry.update();
        }
    }
}
