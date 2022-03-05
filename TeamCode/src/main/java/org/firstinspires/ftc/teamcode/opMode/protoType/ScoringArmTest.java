package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;

@Config
@TeleOp (group = "prototype")
public class ScoringArmTest extends LinearOpMode {
    private ScoringArm scoringArm = new ScoringArm();
    public static double goToDouble = 0.5;

    @Override
    public void runOpMode() throws InterruptedException{
        scoringArm.init(hardwareMap);
        boolean formerA = false;
        boolean formerX = false;

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
//why doesnt my code work
        while(opModeIsActive()){
            scoringArm.run(gamepad1.y);
            scoringArm.goTo(goToDouble);

            telemetry.addData("Left_trigger",gamepad1.left_trigger);
            telemetry.addData("Right_trigger",gamepad1.right_trigger);
            telemetry.update();


        }
    }

}