package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.hardware.AbsoluteAnalogEncoder;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *                                    The buttons are mapped to match the wheels spatially if you
 *                                    were to rotate the gamepad 45deg°. x/square is the front left
 *                    ________        and each button corresponds to the wheel as you go clockwise
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 *                  \________/
 *
 * Uncomment the @Disabled tag below to use this opmode.
 */
// rip if a person had to make the text image above
//@Disabled
@Config
@TeleOp(group = "drive")
public class MotorDirectionDebugger extends LinearOpMode {

    // NO ERRORS!!!
    // RUNS WITH NO ISSUES
    public static double MOTOR_POWER = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        BrainSTEMRobot robot = new BrainSTEMRobot();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        robot.init(hardwareMap, telemetry);

        waitForStart();

        if (isStopRequested()) return;


        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
//            telemetry.addLine("Press each button to turn on its respective motor");
//            telemetry.addLine();
//            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
//            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
//            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
//            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
//            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
//            telemetry.addLine();

            AbsoluteAnalogEncoder flE = new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).setInverted(false);
            AbsoluteAnalogEncoder frE = new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).setInverted(false);
            AbsoluteAnalogEncoder blE = new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).setInverted(false);
            AbsoluteAnalogEncoder brE = new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).setInverted(false);

            CRServo frontLeftServo = hardwareMap.get(CRServo.class, "FLturn");
            CRServo frontRightServo = hardwareMap.get(CRServo.class, "FRturn");
            CRServo backLeftServo = hardwareMap.get(CRServo.class, "BLturn");
            CRServo backRightServo = hardwareMap.get(CRServo.class, "BRturn");

            if(gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0, 0, 0);
                telemetry.addLine("Running Motor: Front Left");
            } else if(gamepad1.y) {
                drive.setMotorPowers(0, 0, 0, MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
            } else if(gamepad1.b) {
                drive.setMotorPowers(0, 0, MOTOR_POWER, 0);
                telemetry.addLine("Running Motor: Rear Right");
            } else if(gamepad1.a) {
                drive.setMotorPowers(0, MOTOR_POWER, 0, 0);
                telemetry.addLine("Running Motor: Rear Left");
            } else {
                drive.setMotorPowers(0, 0, 0, 0);
                telemetry.addLine("Running Motor: None");
            }

//            if(gamepad1.left_stick_y > 0.2) {
//
//                while (flE.getCurrentPosition() < 0) {
//                    frontLeftServo.setPower(0.5);
//                    telemetry.addData("loop ", "while");
//                    telemetry.update();
//                }
//                telemetry.addData("finished", "finished");
//                telemetry.update();
//            }

            telemetry.addData("FL Encoder :", (normalizeRadians( (flE.getCurrentPosition()) )) );
            telemetry.addData("FR Encoder :", (normalizeRadians( (frE.getCurrentPosition()) )) );
            telemetry.addData("BL Encoder :", (normalizeRadians( (blE.getCurrentPosition()) )) );
            telemetry.addData("BR Encoder :", (normalizeRadians( (brE.getCurrentPosition()) )) );

            telemetry.update();
        }

    }


}
