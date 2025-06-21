package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage.newDeprecated;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage.newDeprecated;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "Servo Finder test")
@Config
@Disabled
public class FindTheServoPosistions extends LinearOpMode {


    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlideVelocity(4000, drive.slideRight, drive.slideLeft, drive.wristMotor);
        drive.initArm();


        while (!isStarted()) {
            telemetry.addData("Servo Posistion ", drive.gripServoB.getPosition());
        //    telemetry.addData("claw Posistion" , drive.rightGripServo.getPosition());
            telemetry.addData("claw Posistion" , drive.gripServoF.getPosition());
            telemetry.addData("slide",drive.slideLeft.getCurrentPosition());
            telemetry.update();
        }
       // drive.setWrist(true);
       // sleep(2000);
       // drive.setWrist(false);

    }



}
*/
