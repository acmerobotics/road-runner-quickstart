package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.MeccRobot;
import org.firstinspires.ftc.teamcode.hardware.MeccRobotPrototyping;

@Config
@TeleOp(name="MeccRobotTeleOpProto",group="TeleOp")

public class MeccRobotTeleOpPrototyping extends LinearOpMode{
    private MeccRobotPrototyping robot = new MeccRobotPrototyping();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init(hardwareMap,telemetry,timer);


        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        while(opModeIsActive()){
            robot.run(gamepad1);
            //These are the RATIO positions of the servos
//            telemetry.addData("kF: ",lift.kF);
//            telemetry.addData("kP: ", lift.coeffs.kP);
//              telemetry.addData("liftL: ", .liftLeft.getCurrentPosition());
//              telemetry.addData("liftR: ", lift.liftRight.getCurrentPosition());
            telemetry.update();
        }
    }

}
