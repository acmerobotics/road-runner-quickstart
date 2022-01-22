package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MeccRobot extends Mechanism{
    private SampleMecanumDrive drive;
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private Lift lift = new Lift();
    private ScoringArm scoring = new ScoringArm();


    private boolean formerB = false;
    private boolean formerA = false;
    private boolean formerX = false;
    private boolean formerY = false;


    Telemetry telemetry;

    public void init(HardwareMap hwMap){
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        acquirer.init(hwMap);
        carousel.init(hwMap);
        lift.init(hwMap);
        scoring.init(hwMap);
    }

    public void init(HardwareMap hwmap, Telemetry telemetry){
        init(hwmap);
        this.telemetry = telemetry;
    }

    public void run(Gamepad gamepad){
        drive(gamepad);
        acquirerControls(gamepad);
        carouselRun(gamepad);
        lift(gamepad);
        lift.update();
        telemetry.update();
    }

    public void drive(Gamepad gamepad){
        Pose2d controls = new Pose2d(
                //Going to test if maybe negative)
                gamepad.left_stick_y,
                gamepad.left_stick_x,
                gamepad.right_stick_x
        );
        telemetry.addData("Left_stick_y",gamepad.left_stick_y);
        telemetry.addData("Left_stick_X",gamepad.left_stick_x);
        telemetry.addData("right_stick_x",gamepad.right_stick_x);


        if(!drive.isBusy()) drive.setWeightedDrivePower(controls);
    }

    public void acquirerControls(Gamepad gamepad){
        acquirerRun(gamepad.left_trigger,gamepad.right_trigger);
    }

    public void acquirerRun(double intake, double outake){
        boolean outaking = outake > 0.5;
        boolean intaking = intake > 0.5;
        acquirer.run(outaking,intaking);
    }

    public void carouselRun(Gamepad gamepad){
        if(gamepad.dpad_left) carousel.run(false,true);
        else if (gamepad.dpad_right) carousel.run(true,false);
        else carousel.run(false,false);
    }

    public void lift(Gamepad gamepad){
        //lift code here
        if(gamepad.left_bumper) lift.retract();
        else if(gamepad.right_bumper) lift.extend();
        else lift.retracting(false);



        //scoring.run((int)lift.getCurrentPosition() == 3);

        telemetry.addData("Stage",lift.getStageLevel());
        telemetry.addData("StageHeight",lift.getCurrentPosition());
        telemetry.addData("StageTarget",lift.getTargetPosition());
        telemetry.addData("Lift Status", lift.movementState());

    }


}
