package org.firstinspires.ftc.teamcode.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "lift")
public class LiftTeleOp extends LinearOpMode {

    int currentStage = 0;

    @Override
    public void runOpMode () {
        DcMotor liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        DcMotor liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        Lift lift = new Lift(liftMotor1, liftMotor2);

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.a) {
                if (currentStage >= 3) {
                    currentStage = 0;
                    lift.retract();
                } else {
                    lift.liftToJunction(currentStage++);
                }
            }
        }

    }
}
