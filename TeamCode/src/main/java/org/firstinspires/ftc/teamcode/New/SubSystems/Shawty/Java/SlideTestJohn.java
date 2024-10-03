package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SlideTestJohn", group = "Linear OpMode")
public class SlideTestJohn extends LinearOpMode {

    //                       HNG    BSK  CLP   SUB GND STA
    int[] verticalTargets = {5000, 4000, 3000, 100, 0, 0};
    //verttarget[0] controls hang to mannually lower so we don't hang on air

    //                       HNG BSK CLP SUB  GND  STA
    int[] horizontalTargets = {0, 0, 0, 1000, 1000, 0};
    //horiztarget[3] controls submersible reach, horiztarget[4] controls ground reach if we have to reach out further or less

    LinearSlides verticalSlides = new LinearSlides(hardwareMap);
    LinearSlides horizontalSlides = new LinearSlides(hardwareMap);

    @Override
    public void runOpMode() {

        verticalSlides.state = LinearSlides.State.IDLE;
        verticalSlides.resetValue = 0;
        horizontalSlides.state = LinearSlides.State.IDLE;
        horizontalSlides.resetValue = 0;
        update();
        waitForStart();

        while (opModeIsActive()){

            update();
            telemetry.addData("rightEncoder", verticalSlides.rightSlide.rightEncoder);
            telemetry.addData("leftEncoder", verticalSlides.leftSlide.leftEncoder);
            telemetry.update();

        }
    }

    private void update(){
        verticalSlides.update(verticalTargets);
        horizontalSlides.update(horizontalTargets);
    }

}
