package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop.NeedToTest;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="NewDoubleClaw", group="Linear OpMode")
public class ZNewDoubleClaw extends LinearOpMode {

    private Servo claw;
    private Servo claw2;

    @Override
    public void runOpMode() {

        claw = hardwareMap. get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        while (opModeIsActive()) {

            if (gamepad1.a) {
                claw.setPosition(1);
                claw2.setPosition(1);
            }
            if (gamepad1.b) {
                claw.setPosition(0);
                claw2.setPosition(0);
            }

        }


    }

}
