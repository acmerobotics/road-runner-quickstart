package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.oldmanuels;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.ManualOp;

/*  this is the new headless it does not really on our hewlpers because thinking ahead each years the helper class will have to be changed.
* */

@TeleOp


public class newHeadlessOp extends OpMode {

    SampleMecanumDrive drive;


    @Override
    public void init(){
       drive = new SampleMecanumDrive(hardwareMap);
       drive.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       drive.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       drive.wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }


    @Override
    public void loop(){



    }

}



