package org.firstinspires.ftc.teamcode.az.sample;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OdoTest extends LinearOpMode {

    private DcMotorEx perp;

    @Override
    public void runOpMode() {
        perp = hardwareMap.get(DcMotorEx.class, "backRight");

        perp.setDirection(DcMotorEx.Direction.REVERSE);

        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        perp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Perp Position: ",perp.getCurrentPosition());
            telemetry.update();
        }
    }
    //public final Encoder par0;
}
