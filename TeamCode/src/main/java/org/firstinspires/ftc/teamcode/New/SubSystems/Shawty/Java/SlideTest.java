package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SlideTest", group = "Linear OpMode")
public class SlideTest extends LinearOpMode {

    LinearSlides verticalSlides = new LinearSlides(hardwareMap);

    @Override
    public void runOpMode() {
        while (opModeIsActive()){
            verticalSlides.state = LinearSlides.State.IDLE;
            verticalSlides.resetValue = 0;
            verticalSlides.update();

            telemetry.addData("rightEncoder", verticalSlides.rightSlide.rightEncoder);
            telemetry.addData("leftEncoder", verticalSlides.leftSlide.leftEncoder);
            telemetry.update();
        }
    }
}
