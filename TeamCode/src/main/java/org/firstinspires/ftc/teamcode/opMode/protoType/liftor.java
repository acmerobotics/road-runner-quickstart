package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.util.BooleanManager;

@TeleOp
public class liftor extends LinearOpMode {
    public boolean formerDpadL = false;
    public boolean formerDpadR = false;
    public Lift lift = new Lift();
    BooleanManager aToggle = new BooleanManager(()->{
        lift.toggle();
    });

    @Override
    public void runOpMode() throws InterruptedException {


        lift.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {

            aToggle.update(gamepad1.b);

            lift.update();

            telemetry.addData("A value", gamepad1.b);
            telemetry.addData("targetPos", lift.getTargetPosition());
            telemetry.addData("currentPos", lift.getCurrentPosition());
            telemetry.update();

        }
    }
}
