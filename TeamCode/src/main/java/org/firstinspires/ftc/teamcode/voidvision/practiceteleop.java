
package org.firstinspires.ftc.teamcode.voidvision;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="teenage Teleop", group="Pushbot")
public class practiceteleop extends Auto_Util {
    practicehwmap robot=new practicehwmap();
//
    private ElapsedTime runtime = new ElapsedTime();

    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double lAPower;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        initAuto();
        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            //Drive

            fwdBackPower = -gamepad1.left_stick_y;
            strafePower =-gamepad1.left_stick_x;
            turnPower=-gamepad1.right_stick_x;
            lAPower =-gamepad2.left_stick_y;

            lfPower = (fwdBackPower - turnPower-strafePower);
            rfPower = (fwdBackPower+turnPower+strafePower);
            lbPower = (fwdBackPower-turnPower+strafePower);
            rbPower = (fwdBackPower+turnPower-strafePower);

        }








    }











}
