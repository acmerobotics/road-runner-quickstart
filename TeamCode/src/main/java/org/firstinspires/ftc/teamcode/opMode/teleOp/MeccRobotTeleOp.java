package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.MeccRobot;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;

@Config
@TeleOp(name="MeccRobotTeleOp",group="TeleOp")

public class MeccRobotTeleOp extends LinearOpMode{
    private MeccRobot robot = new MeccRobot();
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
            robot.run(gamepad1,gamepad2);
            //These are the RATIO positions of the servos
//            telemetry.addData("kF: ",lift.kF);
//            telemetry.addData("kP: ", lift.coeffs.kP);
//              telemetry.addData("liftL: ", .liftLeft.getCurrentPosition());
//              telemetry.addData("liftR: ", lift.liftRight.getCurrentPosition());
            telemetry.update();
        }
    }

}
