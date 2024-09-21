package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SlideTest", group = "Linear OpMode")
public class SlideTest extends LinearOpMode {

    VerticalSlides verticalSlides = new VerticalSlides(hardwareMap);

    @Override
    public void runOpMode() {
        while (opModeIsActive()){
            VerticalSlides.state = VerticalSlides.State.IDLE;

            telemetry.addData("rightEncoder", verticalSlides.rightSlide.rightEncoder);
            telemetry.addData("leftEncoder", verticalSlides.leftSlide.leftEncoder);
            telemetry.update();
        }


    }
}
