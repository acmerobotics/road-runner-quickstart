package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class autoCommands {



    public static void driveForward(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(speed);
        lB.setPower(speed);
        rF.setPower(speed);
        rB.setPower(speed);
    }

    public static void driveBackward(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(-speed);
        lB.setPower(-speed);
        rF.setPower(-speed);
        rB.setPower(-speed);
    }

    public static void turnLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(-speed);
        lB.setPower(-speed);
        rF.setPower(speed);
        rB.setPower(speed);

    }

    public static void turnRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(speed);
        lB.setPower(speed);
        rF.setPower(-speed);
        rB.setPower(-speed);
    }

    public static void strafeLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(-speed);
        lB.setPower(speed);
        rF.setPower(speed);
        rB.setPower(-speed);
    }

    public static void strafeRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(speed);
        lB.setPower(-speed);
        rF.setPower(-speed);
        rB.setPower(speed);
    }

    public static void driveForwardLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(speed);
        lB.setPower(speed);
        rF.setPower(0);
        rB.setPower(0);
    }

    public static void driveForwardRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(speed);
        rB.setPower(speed);

    }

    public static void driveBackwardLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(-speed);
        lB.setPower(-speed);
        rF.setPower(0);
        rB.setPower(0);

    }

    public static void driveBackwardRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(-speed);
        rB.setPower(-speed);

    }

    public static void stop(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB) {
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }
}