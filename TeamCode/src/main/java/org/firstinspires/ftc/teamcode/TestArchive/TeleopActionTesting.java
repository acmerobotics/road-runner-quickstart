package org.firstinspires.ftc.teamcode.TestArchive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

class Things {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    ElapsedTime time = new ElapsedTime();

    boolean actionOn = false;

    public Things(HardwareMap HW) {
        FL = HW.get(DcMotor.class, "FL");
        FR = HW.get(DcMotor.class, "FR");
        BL = HW.get(DcMotor.class, "BL");
        BR = HW.get(DcMotor.class, "BR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean getOn() {
        return actionOn;
    }

    public class SpinRight implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket Packet) {
            if (!init) {
                actionOn = true;
                FR.setPower(0.3);
                BR.setPower(0.3);
                FL.setPower(-0.3);
                BL.setPower(-0.3);
                time.reset();
                init = true;
            }

            if (time.seconds() > 2.0) {
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                actionOn = false;
                return false;

            }
            return true;


        }
    }

    public Action spinRight() {
        return new SpinRight();
    }

    public class SpinLeft implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket Packet) {
            if (!init) {
                actionOn = true;
                FR.setPower(-0.3);
                BR.setPower(-0.3);
                FL.setPower(0.3);
                BL.setPower(0.3);
                time.reset();
                init = true;
            }

            if (time.seconds() > 2) {
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                actionOn = false;
                return false;
            }

            return true;


        }
    }

    public Action spinLeft() {
        return new SpinLeft();
    }
}

@TeleOp
public class TeleopActionTesting extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();



    @Override
    public void runOpMode() {

        Things drivea = new Things(hardwareMap);

        boolean lastA = false;
        boolean lastB = false;

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            double y = gamepad1.left_stick_y;
            double x = -1 * gamepad1.left_stick_x;
            double rx = -1 * gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.a && !drivea.getOn()) {
                runningActions.add(drivea.spinRight());

            }

            if (gamepad1.b && !drivea.getOn()) {
                runningActions.add(new SequentialAction(
                        drivea.spinLeft(),
                        drivea.spinRight(),
                        drivea.spinLeft()
                ));

            }

            if (!drivea.getOn()) {
                drivea.FL.setPower(frontLeftPower / 1.5);
                drivea.BL.setPower(backLeftPower / 1.5);
                drivea.FR.setPower(frontRightPower / 1.5);
                drivea.BR.setPower(backRightPower / 1.5);

            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }

    }

}

